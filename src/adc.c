/* SPDX-License-Identifier: GPL-2.0-only */
/**
 * Copyright (C) 2025 Vitaliy N <vitaliy.nimych@gmail.com>
 */
#include "main.h"

/* If your VDDA is not 3.3 V, override this (in mV) for better temp approximation */
#ifndef ADC_VDDA_ASSUMED_mV
#define ADC_VDDA_ASSUMED_mV         3300u
#endif

#define USE_VREF_IN_CHANNEL         1

/* This file gates purely on feature flags (USE_PA, USE_ADC2), never on a
 * target/board name -- see targets/target.h and the board headers under
 * targets/ for where those flags get set.
 *
 * USE_PA   -- ADC_CH_PA_VDET exists in adc_ch_t at all (see target headers).
 * USE_ADC2 -- PA_VDET's detector isn't reachable from ADC1 on this board,
 *             so a second ADC instance is brought up for it. Independent
 *             of which PA feature is active -- adc.c doesn't care why
 *             ADC2 is needed, only that it is.
 *
 * adc_read_raw() has exactly one code path regardless of feature state:
 * it looks ADC_CH_* up in g_adc_ch_map[], a compile-time table of
 * {buffer, offset} pairs. Only the TABLE CONTENTS differ per feature
 * state -- the lookup logic doesn't branch. Peripheral bring-up in
 * adc_init() legitimately does still differ (different hardware needs
 * different init), but the read path does not. */

typedef struct {
    volatile uint16_t *buf;
    uint8_t offset;
} adc_ch_map_t;

/* ADC1's sequence length depends on whether PA_VDET rides along on it
 * (USE_PA && !USE_ADC2) or lives elsewhere (no PA, or PA_VDET on ADC2).
 * ADC2's length is 0 unless USE_ADC2 pulls it in -- currently PA_VDET is
 * its only possible consumer, but the length is tracked independently of
 * that so a future non-PA reason to want ADC2 isn't blocked on PA_VDET
 * existing. */
#if defined(USE_PA) && !defined(USE_ADC2)
#define ADC1_SEQ_LEN 4 /* RESERVED, PA_VDET, TEMP, VREFINT */
#else
#define ADC1_SEQ_LEN 3 /* RESERVED, TEMP, VREFINT */
#endif

#if defined(USE_ADC2)
#define ADC2_SEQ_LEN 1 /* PA_VDET, currently the sole ADC2 consumer */
#else
#define ADC2_SEQ_LEN 0
#endif

static volatile uint16_t g_adc1_dma_buf[ADC1_SEQ_LEN];
#if defined(USE_ADC2)
static volatile uint16_t g_adc2_dma_buf[ADC2_SEQ_LEN];
#endif

/* Compile-time map: adc_ch_t -> (buffer, offset). Rebuilt per feature
 * state below; adc_read_raw() consumes it uniformly either way. */
static const adc_ch_map_t g_adc_ch_map[ADC_CH_COUNT] = {
    [ADC_CH_RESERVED] = { g_adc1_dma_buf, 0 },
#if defined(USE_PA)
#if defined(USE_ADC2)
    [ADC_CH_PA_VDET]  = { g_adc2_dma_buf, 0 },
    [ADC_CH_TEMP]     = { g_adc1_dma_buf, 1 },
    [ADC_CH_VREF_INT] = { g_adc1_dma_buf, 2 },
#else
    [ADC_CH_PA_VDET]  = { g_adc1_dma_buf, 1 },
    [ADC_CH_TEMP]     = { g_adc1_dma_buf, 2 },
    [ADC_CH_VREF_INT] = { g_adc1_dma_buf, 3 },
#endif
#else
    [ADC_CH_TEMP]     = { g_adc1_dma_buf, 1 },
    [ADC_CH_VREF_INT] = { g_adc1_dma_buf, 2 },
#endif
};

#if defined(USE_ADC2)
/* ADC2, single channel, free-running via its own DMA channel --
 * independent of ADC1's sequence below. */
static void adc2_vdet_init(void)
{
    LL_ADC_InitTypeDef ADC_InitStruct = {0};
    LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
    LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = {0};

    /* DMA1_Channel5 -- confirmed free: DMA1 channels 1/2/6 are used by
     * tim.c/dac.c/video_gen.c, DMA1_Channel3 by dma.c (mem2mem), DMA1_4
     * by ADC1 below, DMA2_1/2 by uart.c. */
    LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_5, LL_DMAMUX_REQ_ADC2);
    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_5, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_5, LL_DMA_PRIORITY_LOW);
    LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_5, LL_DMA_MODE_CIRCULAR);
    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_5, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_5, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_5, LL_DMA_PDATAALIGN_HALFWORD);
    LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_5, LL_DMA_MDATAALIGN_HALFWORD);
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_5, (uint32_t)&ADC2->DR);
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_5, (uint32_t)g_adc2_dma_buf);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_5, ADC2_SEQ_LEN);

    ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
    ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
    ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
    LL_ADC_Init(ADC2, &ADC_InitStruct);

    ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
    ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE; // single channel
    ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
    ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_CONTINUOUS;
    ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_UNLIMITED;
    ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;
    LL_ADC_REG_Init(ADC2, &ADC_REG_InitStruct);
    LL_ADC_SetGainCompensation(ADC2, 0);
    LL_ADC_SetOverSamplingScope(ADC2, LL_ADC_OVS_DISABLE);

    ADC_CommonInitStruct.CommonClock = LL_ADC_CLOCK_SYNC_PCLK_DIV4;
    ADC_CommonInitStruct.Multimode = LL_ADC_MULTI_INDEPENDENT;
    LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC2), &ADC_CommonInitStruct);

    LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_1, ADC_PA_VDET_Channel);
    LL_ADC_SetChannelSamplingTime(ADC2, ADC_PA_VDET_Channel, LL_ADC_SAMPLINGTIME_92CYCLES_5);
    LL_ADC_SetChannelSingleDiff(ADC2, ADC_PA_VDET_Channel, LL_ADC_SINGLE_ENDED);

    NVIC_SetPriority(DMA1_Channel5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(DMA1_Channel5_IRQn);

    if (LL_ADC_IsEnabled(ADC2)) {
        LL_ADC_Disable(ADC2);
        while (LL_ADC_IsEnabled(ADC2)) { /* wait */ }
    }

    LL_ADC_EnableInternalRegulator(ADC2);
    for (volatile uint32_t i = 0; i < 2000; ++i) { __NOP(); } // crude ~>20 us

    LL_ADC_StartCalibration(ADC2, LL_ADC_SINGLE_ENDED);
    while (LL_ADC_IsCalibrationOnGoing(ADC2)) { /* wait */ }
    for (volatile uint32_t i = 0; i < 2000; ++i) { __NOP(); }

    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_5);

    LL_ADC_Enable(ADC2);
    while (!LL_ADC_IsActiveFlag_ADRDY(ADC2)) { /* wait */ }

    LL_ADC_REG_StartConversion(ADC2);
}
#endif

void adc_init(void)
{
    LL_ADC_InitTypeDef ADC_InitStruct = {0};
    LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
    LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = {0};
    LL_ADC_INJ_InitTypeDef ADC_INJ_InitStruct = {0};

    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    LL_RCC_SetADCClockSource(LL_RCC_ADC12_CLKSOURCE_SYSCLK);

    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC12);

    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
#if defined(USE_PA)
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA); // needed if ADC_PA_VDET_Pin is on GPIOA (PA4 board variant)
#endif

    GPIO_InitStruct.Pin = ADC_RESERVED_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(ADC_RESERVED_GPIO_Port, &GPIO_InitStruct);

#if defined(USE_PA)
    GPIO_InitStruct.Pin = ADC_PA_VDET_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(ADC_PA_VDET_GPIO_Port, &GPIO_InitStruct);
#endif

    /* ADC1 DMA Init */

    /* ADC1 Init */
    LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_4, LL_DMAMUX_REQ_ADC1);

    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_4, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

    LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_4, LL_DMA_PRIORITY_LOW);

    LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_4, LL_DMA_MODE_CIRCULAR);

    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_4, LL_DMA_PERIPH_NOINCREMENT);

    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_4, LL_DMA_MEMORY_INCREMENT);

    LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_4, LL_DMA_PDATAALIGN_HALFWORD);

    LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_4, LL_DMA_MDATAALIGN_HALFWORD);

    LL_DMA_SetPeriphAddress        (DMA1, LL_DMA_CHANNEL_4, (uint32_t)&ADC1->DR);
    LL_DMA_SetMemoryAddress        (DMA1, LL_DMA_CHANNEL_4, (uint32_t)g_adc1_dma_buf);
    LL_DMA_SetDataLength           (DMA1, LL_DMA_CHANNEL_4, ADC1_SEQ_LEN);

    /** Common config */
    ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
    ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
    ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
    LL_ADC_Init(ADC1, &ADC_InitStruct);
    ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
#if ADC1_SEQ_LEN == 4
    ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_ENABLE_4RANKS;
#else
    ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_ENABLE_3RANKS;
#endif
    ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
    ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_CONTINUOUS;
    ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_UNLIMITED;
    ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;
    LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
    LL_ADC_SetGainCompensation(ADC1, 0);
    LL_ADC_SetOverSamplingScope(ADC1, LL_ADC_OVS_DISABLE);
    ADC_CommonInitStruct.CommonClock = LL_ADC_CLOCK_SYNC_PCLK_DIV4;
    ADC_CommonInitStruct.Multimode = LL_ADC_MULTI_INDEPENDENT;
    LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);
    ADC_INJ_InitStruct.TriggerSource = LL_ADC_INJ_TRIG_SOFTWARE;
#if ADC1_SEQ_LEN == 4
    ADC_INJ_InitStruct.SequencerLength = LL_ADC_INJ_SEQ_SCAN_ENABLE_4RANKS;
#else
    ADC_INJ_InitStruct.SequencerLength = LL_ADC_INJ_SEQ_SCAN_ENABLE_3RANKS;
#endif
    ADC_INJ_InitStruct.SequencerDiscont = LL_ADC_INJ_SEQ_DISCONT_DISABLE;
    ADC_INJ_InitStruct.TrigAuto = LL_ADC_INJ_TRIG_FROM_GRP_REGULAR;
    LL_ADC_INJ_Init(ADC1, &ADC_INJ_InitStruct);
    LL_ADC_INJ_SetQueueMode(ADC1, LL_ADC_INJ_QUEUE_DISABLE);

    /* Disable ADC deep power down (enabled by default after reset state) */
    LL_ADC_DisableDeepPowerDown(ADC1);
    /* Enable ADC internal voltage regulator */
    LL_ADC_EnableInternalRegulator(ADC1);
    /* Delay for ADC internal voltage regulator stabilization. */
    /* Compute number of CPU cycles to wait for, from delay in us. */
    /* Note: Variable divided by 2 to compensate partially */
    /* CPU processing cycles (depends on compilation optimization). */
    /* Note: If system core clock frequency is below 200kHz, wait time */
    /* is only a few CPU processing cycles. */
    uint32_t wait_loop_index;
    wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
    while(wait_loop_index != 0)
    {
        wait_loop_index--;
    }

#if ADC1_SEQ_LEN == 4
    LL_ADC_REG_SetSequencerLength(ADC1, LL_ADC_REG_SEQ_SCAN_ENABLE_4RANKS);
#else
    LL_ADC_REG_SetSequencerLength(ADC1, LL_ADC_REG_SEQ_SCAN_ENABLE_3RANKS);
#endif

    // ADC1 rank order below determines g_adc1_dma_buf[] layout. This must
    // stay consistent with the offsets baked into g_adc_ch_map[] above --
    // both are driven off the same USE_PA/USE_ADC2 flags so they can't
    // drift independently.

    /** Configure Regular Channel */
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, ADC_RESERVED_Channel);
    LL_ADC_SetChannelSamplingTime(ADC1, ADC_RESERVED_Channel, LL_ADC_SAMPLINGTIME_92CYCLES_5);
    LL_ADC_SetChannelSingleDiff(ADC1, ADC_RESERVED_Channel, LL_ADC_SINGLE_ENDED);

    /** Configure Injected Channel */
    LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_1, ADC_RESERVED_Channel);
    LL_ADC_SetChannelSamplingTime(ADC1, ADC_RESERVED_Channel, LL_ADC_SAMPLINGTIME_92CYCLES_5);
    LL_ADC_SetChannelSingleDiff(ADC1, ADC_RESERVED_Channel, LL_ADC_SINGLE_ENDED);

#if defined(USE_PA) && !defined(USE_ADC2)
    /** Configure Regular Channel */
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_2, ADC_PA_VDET_Channel);
    LL_ADC_SetChannelSamplingTime(ADC1, ADC_PA_VDET_Channel, LL_ADC_SAMPLINGTIME_92CYCLES_5);
    LL_ADC_SetChannelSingleDiff(ADC1, ADC_PA_VDET_Channel, LL_ADC_SINGLE_ENDED);

    /** Configure Injected Channel */
    LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_2, ADC_PA_VDET_Channel);
    LL_ADC_SetChannelSamplingTime(ADC1, ADC_PA_VDET_Channel, LL_ADC_SAMPLINGTIME_92CYCLES_5);
    LL_ADC_SetChannelSingleDiff(ADC1, ADC_PA_VDET_Channel, LL_ADC_SINGLE_ENDED);
#endif

    /** Configure Regular Channel */
    LL_ADC_REG_SetSequencerRanks(ADC1, ADC1_SEQ_LEN == 4 ? LL_ADC_REG_RANK_3 : LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_TEMPSENSOR_ADC1);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_TEMPSENSOR_ADC1, LL_ADC_SAMPLINGTIME_247CYCLES_5);
    LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_TEMPSENSOR_ADC1, LL_ADC_SINGLE_ENDED);
    LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_PATH_INTERNAL_TEMPSENSOR | LL_ADC_PATH_INTERNAL_VREFINT);

    /** Configure Injected Channel */
    LL_ADC_INJ_SetSequencerRanks(ADC1, ADC1_SEQ_LEN == 4 ? LL_ADC_INJ_RANK_3 : LL_ADC_INJ_RANK_2, LL_ADC_CHANNEL_TEMPSENSOR_ADC1);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_TEMPSENSOR_ADC1, LL_ADC_SAMPLINGTIME_247CYCLES_5);
    LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_TEMPSENSOR_ADC1, LL_ADC_SINGLE_ENDED);

    /** Configure Regular Channel */
    LL_ADC_REG_SetSequencerRanks(ADC1, ADC1_SEQ_LEN == 4 ? LL_ADC_REG_RANK_4 : LL_ADC_REG_RANK_3, LL_ADC_CHANNEL_VREFINT);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_VREFINT, LL_ADC_SAMPLINGTIME_247CYCLES_5);
    LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_VREFINT, LL_ADC_SINGLE_ENDED);

    /** Configure Injected Channel */
    LL_ADC_INJ_SetSequencerRanks(ADC1, ADC1_SEQ_LEN == 4 ? LL_ADC_INJ_RANK_4 : LL_ADC_INJ_RANK_3, LL_ADC_CHANNEL_VREFINT);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_VREFINT, LL_ADC_SAMPLINGTIME_247CYCLES_5);
    LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_VREFINT, LL_ADC_SINGLE_ENDED);


    NVIC_SetPriority(DMA1_Channel4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
    NVIC_EnableIRQ(DMA1_Channel4_IRQn);

    /* Make sure ADC is disabled before configuration */
    if (LL_ADC_IsEnabled(ADC1)) {
        LL_ADC_Disable(ADC1);
        while (LL_ADC_IsEnabled(ADC1)) { /* wait */ }
    }

    /* Enable ADC internal regulator and wait t ADCVREG_STUP (~20 µs) */
    LL_ADC_EnableInternalRegulator(ADC1);
    for (volatile uint32_t i = 0; i < 2000; ++i) { __NOP(); } // crude ~>20 µs

    /* Calibrate (single-ended) */
    LL_ADC_StartCalibration(ADC1, LL_ADC_SINGLE_ENDED);
    while (LL_ADC_IsCalibrationOnGoing(ADC1)) { /* wait */ }
    for (volatile uint32_t i = 0; i < 2000; ++i) { __NOP(); } // stabilization

    /* Enable DMA */
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_4);

    /* Enable ADC and wait ready */
    LL_ADC_Enable(ADC1);
    while (!LL_ADC_IsActiveFlag_ADRDY(ADC1)) { /* wait */ }

    /* Temp sensor stabilization (>10 µs) */
    for (volatile uint32_t i = 0; i < 2000; ++i) { __NOP(); }

    LL_ADC_REG_StartConversion(ADC1);

#if defined(USE_ADC2)
    adc2_vdet_init();
#endif
}

uint16_t adc_read_raw(adc_ch_t ch)
{
    if ((uint32_t)ch >= ADC_CH_COUNT) return 0;
    return g_adc_ch_map[ch].buf[g_adc_ch_map[ch].offset];
}

uint16_t adc_read_mv(adc_ch_t ch)
{
    return (uint16_t)DAC12BIT_TO_MV(adc_read_raw(ch));
}

uint32_t adc_read_vdda_mv(void)
{
    const uint16_t vref_cal = *VREFINT_CAL_ADDR;
    const uint16_t vref_raw = adc_read_raw(ADC_CH_VREF_INT);
    if (vref_raw == 0) return 3300u;
    // VDDA ≈ 3000mV * VREFINT_CAL / VREFINT_NOW
    return (uint32_t)3000u * (uint32_t)vref_cal / (uint32_t)vref_raw;
}

float adc_read_mcu_temp_c(void)
{
    const uint16_t ts_raw = adc_read_raw(ADC_CH_TEMP);
#if USE_VREF_IN_CHANNEL
    const uint32_t vdda_mv = adc_read_vdda_mv();
    const int32_t t = __LL_ADC_CALC_TEMPERATURE(vdda_mv, ts_raw, LL_ADC_RESOLUTION_12B);
#else
    const int32_t t = __LL_ADC_CALC_TEMPERATURE(ADC_VDDA_ASSUMED_mV, ts_raw, LL_ADC_RESOLUTION_12B);
#endif

    return (float)t;
}
