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

/* Frequency breakpoints the calibration/detector tables (in each target's
 * *_power.c) are indexed against. TODO: match these to your actual
 * calibration sweep points -- this array should really live alongside
 * the table data it indexes rather than here, but is left generic for
 * now since both current targets use the same breakpoints. */
static const uint16_t g_cal_freq_mhz[VTX_CAL_FREQ_POINTS] = {
    5658, 5695, 5760, 5800, 5840, 5905, 5945
};

/* Gains operate on a VDET deviation in mV (rf_detector_target - rf_detector,
 * both mV). Board-specific -- each USE_PA target header (e.g.
 * targets/generic_vtx_pa_rtc76401.h) MUST define all three; there's no
 * value that's a genuine no-op for a PID gain (Kp=1 still very much
 * drives the loop, and Kp=Ki=Kd=0 is really just "silently disabled"
 * wearing a different hat) -- so a missing definition is a compile
 * error rather than a silently-inherited default. */
#if !defined(PA_CONTROL_Kp)
#error "PA_CONTROL_Kp not defined -- set it in your board's target header (see targets/generic_vtx_pa_rtc76401.h)"
#endif
#if !defined(PA_CONTROL_Ki)
#error "PA_CONTROL_Ki not defined -- set it in your board's target header (see targets/generic_vtx_pa_rtc76401.h)"
#endif
#if !defined(PA_CONTROL_Kd)
#error "PA_CONTROL_Kd not defined -- set it in your board's target header (see targets/generic_vtx_pa_rtc76401.h)"
#endif

#ifndef PA_CONTROL_OFFSET_MV
#define PA_CONTROL_OFFSET_MV 0u
#endif

/* Q2 is P-channel; DAC near VDD (Vgs~0) is OFF, DAC near 0 (Vgs~-3.3V) is
 * the MOST conductive state it can be in. Getting this backwards means
 * "disabled" ends up driving near-maximum bias continuously. */
#define VTX_BIAS_OFF_MV 3300u

static uint16_t g_vref_mv = 0;
static float    rf_detector_target = 0;
static double   rf_detector = 0;
static float    pa_control_i = 0;
static float    pa_control_last_deviation = 0;
static const vtx_power_level_t *g_active_level = NULL;

static inline void dac_ch2_write_mv(uint16_t mv)
{
    uint32_t dac_raw = DAC12BIT_FROM_MV(mv);
    if (dac_raw > 4095u) dac_raw = 4095u;
    LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_2, dac_raw);
    LL_DAC_TrigSWConversion(DAC1, LL_DAC_CHANNEL_2);
    g_vref_mv = mv;
}

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

static float lerp(float x, float in_min, float in_max, float out_min, float out_max)
{
    if (in_max == in_min) return out_min;
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static uint8_t cal_freq_index(uint16_t freq)
{
    if (freq < g_cal_freq_mhz[0]) freq = g_cal_freq_mhz[0];
    if (freq > g_cal_freq_mhz[VTX_CAL_FREQ_POINTS - 1]) freq = g_cal_freq_mhz[VTX_CAL_FREQ_POINTS - 1];
    for (uint8_t i = 0; i < VTX_CAL_FREQ_POINTS - 1; i++) {
        if (freq < g_cal_freq_mhz[i + 1]) return i;
    }
    return VTX_CAL_FREQ_POINTS - 2;
}

static uint16_t get_calibration_mv(const vtx_power_level_t *lvl, uint16_t freq)
{
    uint8_t i = cal_freq_index(freq);
    return (uint16_t)lerp(freq, g_cal_freq_mhz[i], g_cal_freq_mhz[i + 1],
                           lvl->calibration[i], lvl->calibration[i + 1]);
}

static uint16_t get_detector_target(const vtx_power_level_t *lvl, uint16_t freq)
{
    uint8_t i = cal_freq_index(freq);
    return (uint16_t)lerp(freq, g_cal_freq_mhz[i], g_cal_freq_mhz[i + 1],
                           lvl->detector[i], lvl->detector[i + 1]);
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

void rf_pa_disable(void)
{
    g_active_level = NULL;
    rf_pa_boost_off();
    dac_ch2_write_mv(VTX_BIAS_OFF_MV);
    rf_detector_target = 0;
    rf_detector = 0;
    pa_control_i = 0;
    pa_control_last_deviation = 0;
}

void rf_pa_apply_level(const vtx_power_level_t *lvl)
{
    g_active_level = lvl;

    if (!lvl) {
        rf_pa_boost_off();
        dac_ch2_write_mv(VTX_BIAS_OFF_MV);
        rf_detector_target = 0;
        rf_detector = 0;
        pa_control_i = 0;
        pa_control_last_deviation = 0;
        return;
    }

    uint16_t freq = vtx_get_config()->frequency;

    rf_detector_target = get_detector_target(lvl, freq);
    pa_control_i = 0;
    pa_control_last_deviation = 0;

    dac_ch2_write_mv(get_calibration_mv(lvl, freq));

    if (lvl->ext_pa_enable) {
        rf_pa_boost_on();
    } else {
        rf_pa_boost_off();
    }
    /* if rf_detector_target != 0, rf_pa_loop() takes over trimming the
     * DAC bias from here */
}

void rf_pa_restore(void)
{
    rf_pa_apply_level(g_active_level); // NULL-safe: re-applies "off" if that was the last state
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

/**
 * @brief Call periodically (e.g. every main-loop iteration). Runs the DAC
 * bias PID loop against the active level's detector target (mV) -- only
 * when that level has a non-zero target, i.e. it's been calibrated with
 * a real VDET reading.
 */
void rf_pa_loop(void)
{
    static uint32_t last_detector_loop = 0;
    static uint32_t last_control_loop = 0;

    if ((HAL_GetTick() - last_detector_loop) >= 1) {
        rf_detector = rf_detector * 0.99 + rf_pa_read_vdet_mv() * 0.01;
        last_detector_loop = HAL_GetTick();
    }

    if (!g_active_level) {
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
        dac_ch2_write_mv((uint16_t)mv);

        last_control_loop = HAL_GetTick();
        pa_control_last_deviation = deviation;
    }
}

#endif //USE_PA
