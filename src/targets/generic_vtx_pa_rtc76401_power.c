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
 * VDET (VPD) ceiling: bench testing found actual VPD saturates around
 * ~350mV on this board -- driving the DAC further into Q2's conduction
 * range past that point produces no further rise. detector[] targets
 * below are left at 0 (open loop) pending a clean, controlled
 * measurement of the real achievable VDET range per level; a prior
 * attempt using datasheet-interpolated targets above this ceiling (see
 * DAC calibration values below) caused the closed loop to chase an
 * unreachable target and walk the DAC through its full range.
 *
 * DAC calibration values below are structural starting points based on
 * the current-draw transition points described above, not power-meter
 * verified mW output -- confirm each against a real power meter before
 * trusting the mW label.
 *
 * Of the DAC starting points below, only 3200mV and 2800mV have been
 * bench-confirmed WITH the boost stage on (730mA and 790mA respectively,
 * both within a 500mA-1A supply's budget). 3000mV and 2900mV are
 * bracketed between those two known-safe points, not individually
 * confirmed with boost on -- verify current draw at each before trusting
 * them.
 */
#include "main.h"
#include "vtx_power_levels.h"

const vtx_power_level_t g_vtx_power_levels[] = {
    { 1,   RTC6705_PA_3dBm, false, {3200,3200,3200,3200,3200,3200,3200}, {0,0,0,0,0,0,0} }, // PAOUT1 off, Q2 off -- matches pit-mode baseline (~310mA)
    { 2,   RTC6705_PA_3dBm, false, {2400,2400,2400,2400,2400,2400,2400}, {0,0,0,0,0,0,0} }, // PAOUT1 off, Q2 off -- with a more DC bias
    { 5,   RTC6705_PA_7dBm, false, {3200,3200,3200,3200,3200,3200,3200}, {0,0,0,0,0,0,0} }, // PAOUT1 on, Q2 off -- RTC6705's own drive alone, no boost
    { 10,  RTC6705_PA_7dBm, false, {2400,2400,2400,2400,2400,2400,2400}, {0,0,0,0,0,0,0} }, // PAOUT1 on, Q2 off -- with more DC bias
    { 50,  RTC6705_PA_3dBm, true,  {3200,3200,3200,3200,3200,3200,3200}, {100,100,100,100,100,100,100} }, // boost on, DAC bench-confirmed (~730mA) -- Vpd target interpolated, UNVALIDATED against a power meter
    { 100, RTC6705_PA_3dBm, true,  {3000,3000,3000,3000,3000,3000,3000}, {150,150,150,150,150,150,150} }, // boost on, DAC NOT individually confirmed -- verify current draw before trusting; Vpd target interpolated, UNVALIDATED
    { 150, RTC6705_PA_3dBm, true,  {2900,2900,2900,2900,2900,2900,2900}, {200,200,200,200,200,200,200} }, // boost on, DAC NOT individually confirmed -- verify current draw before trusting; Vpd target interpolated, UNVALIDATED
    { 200, RTC6705_PA_3dBm, true,  {2800,2800,2800,2800,2800,2800,2800}, {300,300,300,300,300,300,300} }, // boost on, DAC bench-confirmed (~790mA) -- Vpd target interpolated, UNVALIDATED against a power meter
};

const uint8_t g_vtx_power_level_count = sizeof(g_vtx_power_levels) / sizeof(g_vtx_power_levels[0]);
