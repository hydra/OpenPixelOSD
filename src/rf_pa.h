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
 * way.
 *
 * This file owns PA MECHANICS only (DAC bias, boost-enable GPIO, VDET
 * read, the closed-loop trim) -- it does NOT own the power-level table.
 * See vtx_power_levels.h for that; vtx_msp.c looks up the level for
 * whatever index the FC selected and hands a pointer to
 * rf_pa_apply_level(). */
#if defined(USE_PA)

#include <stdint.h>
#include <stdbool.h>
#include "vtx_power_levels.h"

void rf_pa_init(void);

/* Applies lvl's DAC bias and boost-enable state. Pass NULL for fully off
 * (pitmode or no VTX config received yet). Remembers lvl for
 * rf_pa_restore()/rf_pa_loop(). */
void rf_pa_apply_level(const vtx_power_level_t *lvl);

/* Blunt full-off: boost GPIO low, DAC to its safe off point. Used for
 * gating during a retune (see rtc6705.c) and equivalent to
 * rf_pa_apply_level(NULL). */
void rf_pa_disable(void);

/* Re-applies whatever level was last passed to rf_pa_apply_level() (NULL
 * included). Used by rtc6705.c to restore state after gating the PA off
 * during a retune, WITHOUT unconditionally forcing anything on. */
void rf_pa_restore(void);

uint16_t rf_pa_read_vdet_mv(void);
uint16_t rf_pa_get_vref_mv(void);
void rf_pa_set_vref_mv(uint16_t mv);

/* Call periodically (e.g. every main-loop iteration) to run the DAC bias
 * PID loop against the active level's detector target, when it has one. */
void rf_pa_loop(void);

#endif //USE_PA
#endif //RF_PA_H
