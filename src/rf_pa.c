/* SPDX-License-Identifier: GPL-2.0-only */
/**
 * Copyright (C) 2025 Vitaliy N <vitaliy.nimych@gmail.com>
 */
#include "main.h"
#include "rf_pa.h"
#include "vtx_msp.h"
#include <stdbool.h>
#include <string.h>

/* Entire rest of this file is gated on USE_PA -- see rf_pa.h. On a board
 * with no PA feature enabled, this translation unit compiles to nothing. */
#if defined(USE_PA)

/* Frequency breakpoints the calibration/detector tables are indexed
 * against. TODO: match these to your actual calibration sweep points. */
static const uint16_t g_cal_freq_mhz[RF_PA_CAL_FREQ_POINTS] = {
    5658, 5695, 5760, 5800, 5840, 5905, 5945
};

/* NOTE: placeholder values, calibrate against a real power meter.
 * calibration[] = DAC mV setpoint per breakpoint (open-loop / PID target).
 * detector[]    = 0 everywhere below -> runs fully open-loop until you
 * populate real VDET readings here. RAM-only: not persisted across
 * resets (this project doesn't have a flash/EEPROM module yet -- add
 * one and a read/write pair here if you want calibration to survive
 * power cycles).
 *
 * SAFETY: Q2 (SSM3J56MFV) is P-channel, source on 3V3_RF (~3.3V), so
 * Vgs = DAC_mv - 3300. Per its datasheet, |Vgs(th)| max = 1V, and
 * Rds(on) is already down to 480mOhm at Vgs=-2.5V (660mOhm at just
 * -1.8V) -- i.e. it is SUBSTANTIALLY conducting well before Vgs reaches
 * -2.5V. An earlier version of this table used 800-2400mV (Vgs -2.5V to
 * -0.9V) for ALL four levels, which is deep in the fully-on region for
 * every single one -- that is almost certainly what caused a large
 * current draw the moment any non-OFF level was requested.
 *
 * All four levels below now default to the SAME safe DAC starting point,
 * 3200mV (Vgs=-100mV, comfortably above the 1V max threshold -> Q2
 * should be at or very near cutoff). This is NOT a working calibration --
 * every level will produce roughly the same (minimal) output until you
 * sweep each one down individually against a real power meter, watching
 * bench current the whole time. Move in small steps (e.g. 50mV) and
 * expect the transition from "off" to "conducting" to happen over a
 * fairly narrow band given how low this device's threshold is -- don't
 * assume the useful range spans anywhere near VDD down to 0V.
 *
 * ext_pa_enable: RTC76401 is a fixed ~29dB gain block (see its datasheet)
 * with no meaningful intermediate bias state -- it should only be engaged
 * for levels that actually need that much gain. 20mW is RTC6705's own
 * output alone (ext_pa_enable=false); 100/200/800mW engage the boost
 * stage. Getting this wrong (unconditionally enabling it for any non-OFF
 * level) is what caused a ~750mA jump on the very first non-pit-mode
 * level previously. */
rf_pa_cal_t g_rf_pa_table[RF_PA_PWR_COUNT] = {
    [RF_PA_PWR_OFF]   = { 0,   false, {0,0,0,0,0,0,0},       {0,0,0,0,0,0,0} },
    [RF_PA_PWR_20mW]  = { 20,  false, {3200,3200,3200,3200,3200,3200,3200}, {0,0,0,0,0,0,0} },
    [RF_PA_PWR_100mW] = { 100, true,  {3200,3200,3200,3200,3200,3200,3200}, {0,0,0,0,0,0,0} },
    [RF_PA_PWR_200mW] = { 200, true,  {3200,3200,3200,3200,3200,3200,3200}, {0,0,0,0,0,0,0} },
    [RF_PA_PWR_800mW] = { 800, true,  {3200,3200,3200,3200,3200,3200,3200}, {0,0,0,0,0,0,0} },
};

#ifndef PA_CONTROL_Kp
#define PA_CONTROL_Kp        0.6f   /* TODO: tune on bench */
#endif
#ifndef PA_CONTROL_Ki
#define PA_CONTROL_Ki        0.05f
#endif
#ifndef PA_CONTROL_Kd
#define PA_CONTROL_Kd        0.0f
#endif
#ifndef PA_CONTROL_OFFSET_MV
#define PA_CONTROL_OFFSET_MV 0u
#endif

static uint16_t g_vref_mv = 0;
static float    rf_detector_target = 0;
static double   rf_detector = 0;
static float    pa_control_i = 0;
static float    pa_control_last_deviation = 0;
static rf_pa_power_t g_active_level = RF_PA_PWR_OFF;

static inline void dac_ch2_write_mv(uint16_t mv)
{
    uint32_t dac_raw = DAC12BIT_FROM_MV(mv);
    if (dac_raw > 4095u) dac_raw = 4095u;
    LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_2, dac_raw);
    LL_DAC_TrigSWConversion(DAC1, LL_DAC_CHANNEL_2);
    g_vref_mv = mv;
}

static float lerp(float x, float in_min, float in_max, float out_min, float out_max)
{
    if (in_max == in_min) return out_min;
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static uint8_t cal_freq_index(uint16_t freq)
{
    if (freq < g_cal_freq_mhz[0]) freq = g_cal_freq_mhz[0];
    if (freq > g_cal_freq_mhz[RF_PA_CAL_FREQ_POINTS - 1]) freq = g_cal_freq_mhz[RF_PA_CAL_FREQ_POINTS - 1];
    for (uint8_t i = 0; i < RF_PA_CAL_FREQ_POINTS - 1; i++) {
        if (freq < g_cal_freq_mhz[i + 1]) return i;
    }
    return RF_PA_CAL_FREQ_POINTS - 2;
}

static uint16_t get_calibration_mv(rf_pa_power_t level, uint16_t freq)
{
    uint8_t i = cal_freq_index(freq);
    return (uint16_t)lerp(freq, g_cal_freq_mhz[i], g_cal_freq_mhz[i + 1],
                           g_rf_pa_table[level].calibration[i],
                           g_rf_pa_table[level].calibration[i + 1]);
}

static uint16_t get_detector_target(rf_pa_power_t level, uint16_t freq)
{
    uint8_t i = cal_freq_index(freq);
    return (uint16_t)lerp(freq, g_cal_freq_mhz[i], g_cal_freq_mhz[i + 1],
                           g_rf_pa_table[level].detector[i],
                           g_rf_pa_table[level].detector[i + 1]);
}

void rf_pa_init(void)
{
    /* Enable DAC1 ch2 if not already enabled by user init (DAC1_Init()
     * in dac.c, called from video_overlay_init(), already configured
     * PA5 as analog/buffered/GPIO-connected -- this just arms channel 2
     * and zeroes it). */
    LL_DAC_Enable(DAC1, LL_DAC_CHANNEL_2);

#if defined(PA_RTC76401)
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB); /* harmless if already on */
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = PA_ON_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(PA_ON_GPIO_Port, &GPIO_InitStruct);
#endif

    rf_pa_disable(); // keep PA off at boot
}

/* Q2 is P-channel; DAC near VDD (Vgs~0) is OFF, DAC near 0 (Vgs~-3.3V) is
 * the MOST conductive state it can be in -- the inverse of the "0 = off"
 * convention this file's DAC-bias approach was originally adapted from.
 * Getting this backwards means the "disabled" state was actually driving
 * near-maximum bias into RTC6705's PAOUT1 continuously. */
#define VTX_BIAS_OFF_MV 3300u

/* External/boost PA GPIO control only -- does NOT touch the DAC bias.
 * No-op on boards without a separate boost-enable pin (e.g. PA_GENERIC). */
static inline void rf_pa_boost_on(void)
{
#if defined(PA_RTC76401)
    LL_GPIO_SetOutputPin(PA_ON_GPIO_Port, PA_ON_Pin);
#endif
}

static inline void rf_pa_boost_off(void)
{
#if defined(PA_RTC76401)
    LL_GPIO_ResetOutputPin(PA_ON_GPIO_Port, PA_ON_Pin);
#endif
}

void rf_pa_enable(void)
{
    dac_ch2_write_mv(g_vref_mv);
    if (g_active_level != RF_PA_PWR_OFF && g_rf_pa_table[g_active_level].ext_pa_enable) {
        rf_pa_boost_on();
    } else {
        rf_pa_boost_off();
    }
}

void rf_pa_disable(void)
{
    rf_pa_boost_off();
    dac_ch2_write_mv(VTX_BIAS_OFF_MV);
}

void rf_pa_restore(void)
{
    if (g_active_level == RF_PA_PWR_OFF) {
        rf_pa_disable();
    } else {
        rf_pa_enable();
    }
}

void rf_pa_set_vref_mv(uint16_t mv)
{
    dac_ch2_write_mv(mv);
}

uint16_t rf_pa_get_vref_mv(void)
{
    return g_vref_mv;
}

uint16_t rf_pa_read_vdet_mv(void)
{
    return adc_read_mv(ADC_CH_PA_VDET);
}

static uint16_t rf_pa_read_vdet_raw(void)
{
    return adc_read_raw(ADC_CH_PA_VDET);
}

uint16_t rf_pa_set_power_level(rf_pa_power_t level)
{
    if (level >= RF_PA_PWR_COUNT) {
        level = RF_PA_PWR_OFF;
    }
    g_active_level = level;

    if (level == RF_PA_PWR_OFF) {
        rf_detector_target = 0;
        rf_detector = 0;
        pa_control_i = 0;
        pa_control_last_deviation = 0;
        rf_pa_disable();
        return 0;
    }

    uint16_t freq = vtx_get_config()->frequency;

    rf_detector_target = get_detector_target(level, freq);
    pa_control_i = 0;
    pa_control_last_deviation = 0;

    uint16_t mv = get_calibration_mv(level, freq);
    g_vref_mv = mv; /* rf_pa_enable() below writes this out */
    rf_pa_enable();

    if (rf_detector_target != 0) {
        /* rf_pa_loop() takes over from here and trims g_vref_mv */
    }
    return mv;
}

/**
 * @brief Call periodically (e.g. every main-loop iteration). Runs the DAC
 * bias PID loop against the active level's detector target -- only when
 * that level has a non-zero target, i.e. it's been calibrated with a real
 * VDET reading. Levels left at detector[]==0 stay open-loop indefinitely.
 */
void rf_pa_loop(void)
{
    static uint32_t last_detector_loop = 0;
    static uint32_t last_control_loop = 0;

    if ((HAL_GetTick() - last_detector_loop) >= 1) {
        rf_detector = rf_detector * 0.99 + rf_pa_read_vdet_raw() * 0.01;
        last_detector_loop = HAL_GetTick();
    }

    if (g_active_level == RF_PA_PWR_OFF) {
        pa_control_i = 0;
        return;
    }

    if (rf_detector_target && (HAL_GetTick() - last_control_loop) >= 5) {
        float deviation = rf_detector_target - rf_detector;
        float p = deviation * PA_CONTROL_Kp;
        pa_control_i += deviation * PA_CONTROL_Ki;
        float d = (pa_control_last_deviation - deviation) * PA_CONTROL_Kd;

        float mv = PA_CONTROL_OFFSET_MV + pa_control_i + p + d;
        if (mv < 0) mv = 0;
        rf_pa_set_vref_mv((uint16_t)mv);

        last_control_loop = HAL_GetTick();
        pa_control_last_deviation = deviation;
    }
}

#endif //USE_PA
