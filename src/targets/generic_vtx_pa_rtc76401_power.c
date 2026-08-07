/* SPDX-License-Identifier: GPL-2.0-only */
/**
 * targets/generic_vtx_pa_rtc76401_power.c — power table for the RTC76401
 * external PA board.
 *
 * RTC6705 register: PA5G_PW is not a gain ladder on this board. Each bit
 * independently gates a separate physical output pin (bit0=PAOUT1,
 * bit1=PAOUT2), standard RTC6705 architecture for boards that combine
 * both outputs for extra power. This board only routes PAOUT1 externally
 * (PAOUT2 is N/C), so only bit0 matters:
 *   RTC6705_PA_3dBm  (00) and RTC6705_PA_11dBm (10) -> PAOUT1 off (identical externally)
 *   RTC6705_PA_7dBm  (01) and RTC6705_PA_13dBm (11) -> PAOUT1 on  (identical externally)
 * Only two RTC6705-alone states are available here, not four.
 *
 * DAC bias (Q2/SSM3J56MFV, PAOUT1 injection): the off->on transition
 * happens within roughly 2800-3200mV; below ~2400mV it's fully saturated
 * and further gate drive does nothing more. The useful trim range is
 * that ~400mV window, not the full 0-3300mV span.
 *
 * RTC76401 current draw: system total with the boost stage on is
 * ~730-790mA depending on DAC position; the board's own PA-off baseline
 * is ~310mA running (~251mA with the MCU halted). RTC76401's own
 * contribution is therefore roughly 440-480mA at DAC=3200mV (Q2 off, so
 * this is close to pure quiescent current), against a 372mA typical
 * quiescent spec -- about 15-30% over. Current rises further as the DAC
 * moves deeper into Q2's conduction band, consistent with RTC76401
 * having very little headroom between small-signal gain and P1dB per
 * its own datasheet. A 500mA-1A supply is not enough headroom to
 * characterize this PA anywhere near real output; use one rated for at
 * least 2-3A for further calibration work, and confirm actual VREF
 * voltage at the RTC76401 pin under load (recommended range 2.8-3.3V)
 * as part of that work.
 *
 * DAC calibration values below are structural starting points based on
 * the current-draw transition points described above, not power-meter
 * verified mW output -- confirm each against a real power meter before
 * trusting the mW label.
 */
#include "main.h"
#include "vtx_power_levels.h"

const vtx_power_level_t g_vtx_power_levels[] = {
    { 1,   RTC6705_PA_3dBm, false, {3200,3200,3200,3200,3200,3200,3200}, {0,0,0,0,0,0,0} }, // PAOUT1 off, Q2 off -- matches pit-mode baseline (~310mA)
    { 2,   RTC6705_PA_3dBm, false, {2400,2400,2400,2400,2400,2400,2400}, {0,0,0,0,0,0,0} }, // PAOUT1 off, Q2 off -- with a more DC bias
    { 5,   RTC6705_PA_7dBm, false, {3200,3200,3200,3200,3200,3200,3200}, {0,0,0,0,0,0,0} }, // PAOUT1 on, Q2 off -- RTC6705's own drive alone, no boost
    { 10,   RTC6705_PA_7dBm, false, {2400,2400,2400,2400,2400,2400,2400}, {0,0,0,0,0,0,0} }, // PAOUT1 on, Q2 off -- with more DC bias
    { 82,  RTC6705_PA_3dBm, true,  {3200,3200,3200,3200,3200,3200,3200}, {0,0,0,0,0,0,0} }, // boost on, Q2 off -- mostly RTC76401 quiescent current (~730mA total, ~440-480mA delta) -- UNVALIDATED mW, verify against power meter
    { 150, RTC6705_PA_3dBm, true,  {2800,2800,2800,2800,2800,2800,2800}, {0,0,0,0,0,0,0} }, // boost on, Q2 starting to conduct (~790mA total, confirmed on a 500mA-1A supply) -- UNVALIDATED mW, verify against power meter
};

const uint8_t g_vtx_power_level_count = sizeof(g_vtx_power_levels) / sizeof(g_vtx_power_levels[0]);
