/* SPDX-License-Identifier: GPL-2.0-only */
/**
 * targets/generic_vtx_pa_rtc76401_power.c — power table for the RTC76401
 * external PA board.
 *
 * RTC76401's 29dB gain figure is small-signal (measured at Pin=-30dBm per
 * its datasheet) -- RTC6705's own four settings (+3/+7/+11/+13dBm) are
 * 33-43dB above that reference point, well past RTC76401's P1dB knee
 * (~+5dBm in, estimated from OP1dB=+33dBm output at 28dB compressed
 * gain). In practice this means:
 *   - PA off: RTC6705's own output, exact math from its register spec.
 *   - PA on: clamps toward Psat=+34dBm (2.5W) for anything at or above
 *     ~+7dBm drive -- RTC6705's 11dBm/13dBm settings buy NO additional
 *     output once the boost stage is on, just extra DC current and heat.
 *
 * Full combination space (8 entries: 4 RTC6705 settings x PA off/on) is
 * documented where this table was derived -- most of the 4 "PA on"
 * combinations collapse to the same ~2.5W ceiling, so exposing all 4
 * to the FC would just be duplicate/misleading entries. This table
 * instead picks the meaningfully DISTINCT points:
 *   1) 2mW    -- RTC6705 3dBm,  PA off (minimum)
 *   2) 20mW   -- RTC6705 13dBm, PA off (RTC6705's own maximum, PA fully off)
 *   3) 1400mW -- RTC6705 3dBm,  PA on  (lowest drive that still meaningfully
 *                engages gain before the knee -- least overdriven boosted option)
 *   4) 2500mW -- RTC6705 7dBm,  PA on  (lowest drive that reaches Psat --
 *                using 11dBm or 13dBm here would be pure waste)
 * There is a real, unavoidable gap between #2 and #3 (~20mW to ~1.4W) --
 * this hardware cannot produce anything in that range. The old 100mW/
 * 200mW labels never corresponded to an achievable output on this board.
 *
 * mW figures for the PA-on rows are ESTIMATES from datasheet compression
 * points, not measurements -- confirm with a real power meter. The
 * calibration[] DAC values remain at the safe 3200mV baseline (see
 * rf_pa.c's SSM3J56MFV notes) -- NOT a working calibration, still needs
 * a real bench sweep per level exactly as before.
 */
#include "main.h"
#include "vtx_power_levels.h"

const vtx_power_level_t g_vtx_power_levels[] = {
    { 2,    RTC6705_PA_3dBm, false, {3200,3200,3200,3200,3200,3200,3200}, {0,0,0,0,0,0,0} },
    { 20,   RTC6705_PA_13dBm, false, {3200,3200,3200,3200,3200,3200,3200}, {0,0,0,0,0,0,0} },
    { 1400, RTC6705_PA_3dBm, true,  {3200,3200,3200,3200,3200,3200,3200}, {0,0,0,0,0,0,0} },
    { 2500, RTC6705_PA_7dBm, true,  {3200,3200,3200,3200,3200,3200,3200}, {0,0,0,0,0,0,0} },
};

const uint8_t g_vtx_power_level_count = sizeof(g_vtx_power_levels) / sizeof(g_vtx_power_levels[0]);
