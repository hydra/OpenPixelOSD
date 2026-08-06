/* SPDX-License-Identifier: GPL-2.0-only */
/**
 * targets/generic_vtx_pa_power.c — power table for the baseline PA
 * (PA_GENERIC): a single DAC-biased amplifier, no separate boost-enable
 * GPIO (rf_pa_boost_on()/off() are no-ops without PA_RTC76401, so
 * ext_pa_enable's value here doesn't matter functionally -- set true for
 * clarity).
 *
 * PLACEHOLDER calibration values -- this board's actual bias topology
 * was never fully characterized in this codebase (the original DAC-based
 * rf_pa.c approach was inherited from a different, unspecified PA design).
 * Do not trust these numbers without a real bench sweep against a power
 * meter, the same way the RTC76401 table required.
 */
#include "main.h"
#include "vtx_power_levels.h"

const vtx_power_level_t g_vtx_power_levels[] = {
    { 25,  RTC6705_PA_7dBm,  true, {800,800,800,800,800,800,800},   {0,0,0,0,0,0,0} },
    { 100, RTC6705_PA_11dBm, true, {1400,1400,1400,1400,1400,1400,1400}, {0,0,0,0,0,0,0} },
    { 200, RTC6705_PA_13dBm, true, {1800,1800,1800,1800,1800,1800,1800}, {0,0,0,0,0,0,0} },
    { 800, RTC6705_PA_13dBm, true, {2400,2400,2400,2400,2400,2400,2400}, {0,0,0,0,0,0,0} },
};

const uint8_t g_vtx_power_level_count = sizeof(g_vtx_power_levels) / sizeof(g_vtx_power_levels[0]);
