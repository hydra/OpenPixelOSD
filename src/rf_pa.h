/* SPDX-License-Identifier: GPL-2.0-only */
/**
 * Copyright (C) 2025 Vitaliy N <vitaliy.nimych@gmail.com>
 */
#ifndef RF_PA_H
#define RF_PA_H

/* Entire file is gated on USE_PA (set by a board header alongside its PA
 * feature, e.g. PA_GENERIC or PA_RTC76401 -- see targets/). A board with
 * an RTC6705 but no PA of any kind simply never defines USE_PA, and this
 * header then declares nothing at all -- not even empty stubs. Callers
 * (main.c, vtx_msp.c) must guard their own use of these symbols the same
 * way. */
#if defined(USE_PA)

#include <stdint.h>
#include <stdbool.h>

typedef enum {
  RF_PA_PWR_OFF = 0,   // ~0 mW (PA disabled)
  RF_PA_PWR_20mW,      // ~20 mW
  RF_PA_PWR_100mW,     // ~100 mW
  RF_PA_PWR_200mW,     // ~200 mW
  RF_PA_PWR_800mW,     // ~800 mW
  RF_PA_PWR_COUNT
} rf_pa_power_t;

/* Frequency breakpoints the calibration/detector tables are indexed
 * against (MHz). TODO: match to your actual calibration sweep points. */
#define RF_PA_CAL_FREQ_POINTS 7

typedef struct {
    uint16_t mW;                                 // nominal target, informational only
    uint16_t calibration[RF_PA_CAL_FREQ_POINTS];  // DAC mV per freq breakpoint (open-loop / PID setpoint)
    uint16_t detector[RF_PA_CAL_FREQ_POINTS];     // target raw VDET ADC reading per freq breakpoint; 0 = no closed loop for this level
} rf_pa_cal_t;

extern rf_pa_cal_t g_rf_pa_table[RF_PA_PWR_COUNT]; // index 0 (OFF) unused

void rf_pa_init(void);
void rf_pa_enable(bool on);
uint16_t rf_pa_read_vdet_mv(void);
uint16_t rf_pa_get_vref_mv(void);
void rf_pa_set_vref_mv(uint16_t mv);
uint16_t rf_pa_set_power_level(rf_pa_power_t level);

/* Call periodically (e.g. every main-loop iteration) to run the DAC bias
 * PID loop against the active level's detector target, when it has one. */
void rf_pa_loop(void);

#endif //USE_PA
#endif //RF_PA_H
