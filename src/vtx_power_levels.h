/* SPDX-License-Identifier: GPL-2.0-only */
/**
 * vtx_power_levels.h — canonical VTX power-level table shape.
 *
 * The struct and the table itself live here rather than in rf_pa.h
 * because the RTC6705 register step is meaningful with or without an
 * external boost PA (USE_PA) -- a board with no PA feature at all still
 * has RTC6705's own 4-step register giving real, distinct output levels.
 *
 * The TABLE CONTENTS are a board hardware fact (which RTC6705 register /
 * boost combinations actually exist), not something generic code should
 * guess at -- each target defines its own in a *_power.c file (see
 * targets/generic_power.c, targets/generic_vtx_pa_power.c,
 * targets/generic_vtx_pa_rtc76401_power.c), selected by CMakeLists.txt
 * alongside its pin header.
 *
 * vtx_msp.c consumes this table to answer Betaflight's VTXTABLE queries
 * and to realize whichever index gets selected. rf_pa.c (when USE_PA)
 * consumes individual entries via rf_pa_apply_level() to drive the DAC
 * bias / boost-enable / detector-target mechanics -- it does not own or
 * define the table.
 */
#ifndef VTX_POWER_LEVELS_H
#define VTX_POWER_LEVELS_H
#include <stdint.h>
#include <stdbool.h>
#include "rtc6705.h"

#define VTX_CAL_FREQ_POINTS 7

typedef struct {
    uint16_t        mW;             // advertised to the FC, informational only
    rtc6705_power_t rtc6705_level;  // RTC6705 register step -- always meaningful, RTC6705 is always present under BUILD_VARIANT_VTX
#if defined(USE_PA)
    bool     ext_pa_enable;                     // does this level engage the external boost PA stage (e.g. RTC76401)?
    uint16_t calibration[VTX_CAL_FREQ_POINTS];   // DAC mV per freq breakpoint (open-loop / PID setpoint)
    uint16_t detector[VTX_CAL_FREQ_POINTS];      // target raw VDET ADC reading per freq breakpoint; 0 = open loop for this level
#endif
} vtx_power_level_t;

extern const vtx_power_level_t g_vtx_power_levels[];
extern const uint8_t g_vtx_power_level_count; // number of real levels; no separate "OFF" entry -- pitmode/off is handled outside the table

#endif //VTX_POWER_LEVELS_H
