/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _FSL_PDB_H_
#define _FSL_PDB_H_

#include "fsl_common.h"

/*!
 * @addtogroup pdb
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define FSL_PDB_DRIVER_VERSION (MAKE_VERSION(2, 0, 0)) /*!< Version 2.0.0. */

/*!
 * @brief PDB flags.
 */
enum _pdb_status_flags
{
    kPDB_LoadOKFlag     = PDB_SC_LDOK_MASK,  /*!< This flag is automatically cleared when the values in buffers are
                                                  loaded into the internal registers after the LDOK bit is set or the
                                                  PDBEN is cleared. */
    kPDB_DelayEventFlag = PDB_SC_PDBIF_MASK, /*!< PDB timer delay event flag. */
};

/*!
 * @brief PDB ADC PreTrigger channel flags.
 */
enum _pdb_adc_pretrigger_flags
{
    /* PDB PreTrigger channel match flags. */
    kPDB_ADCPreTriggerChannel0Flag      = PDB_S_CF(1U << 0), /*!< Pre-Trigger 0 flag. */
    kPDB_ADCPreTriggerChannel1Flag      = PDB_S_CF(1U << 1), /*!< Pre-Trigger 1 flag. */
#if (PDB_DLY_COUNT > 2)
    kPDB_ADCPreTriggerChannel2Flag      = PDB_S_CF(1U << 2), /*!< Pre-Trigger 2 flag. */
    kPDB_ADCPreTriggerChannel3Flag      = PDB_S_CF(1U << 3), /*!< Pre-Trigger 3 flag. */
#endif /* PDB_DLY_COUNT > 2 */
#if (PDB_DLY_COUNT > 4)
    kPDB_ADCPreTriggerChannel4Flag      = PDB_S_CF(1U << 4), /*!< Pre-Trigger 4 flag. */
    kPDB_ADCPreTriggerChannel5Flag      = PDB_S_CF(1U << 5), /*!< Pre-Trigger 5 flag. */
    kPDB_ADCPreTriggerChannel6Flag      = PDB_S_CF(1U << 6), /*!< Pre-Trigger 6 flag. */
    kPDB_ADCPreTriggerChannel7Flag      = PDB_S_CF(1U << 7), /*!< Pre-Trigger 7 flag. */
#endif /* PDB_DLY_COUNT > 4 */

    /* PDB PreTrigger channel error flags. */
    kPDB_ADCPreTriggerChannel0ErrorFlag = PDB_S_ERR(1U << 0), /*!< Pre-Trigger 0 Error. */
    kPDB_ADCPreTriggerChannel1ErrorFlag = PDB_S_ERR(1U << 1), /*!< Pre-Trigger 1 Error. */
#if (PDB_DLY_COUNT > 2)
    kPDB_ADCPreTriggerChannel2ErrorFlag = PDB_S_ERR(1U << 2), /*!< Pre-Trigger 2 Error. */
    kPDB_ADCPreTriggerChannel3ErrorFlag = PDB_S_ERR(1U << 3), /*!< Pre-Trigger 3 Error. */
#endif /* PDB_DLY_COUNT > 2 */
#if (PDB_DLY_COUNT > 4)
    kPDB_ADCPreTriggerChannel4ErrorFlag = PDB_S_ERR(1U << 4), /*!< Pre-Trigger 4 Error. */
    kPDB_ADCPreTriggerChannel5ErrorFlag = PDB_S_ERR(1U << 5), /*!< Pre-Trigger 5 Error. */
    kPDB_ADCPreTriggerChannel6ErrorFlag = PDB_S_ERR(1U << 6), /*!< Pre-Trigger 6 Error. */
    kPDB_ADCPreTriggerChannel7ErrorFlag = PDB_S_ERR(1U << 7), /*!< Pre-Trigger 7 Error. */
#endif /* PDB_DLY_COUNT > 4 */
};

/*!
 * @brief PDB buffer interrupts.
 */
enum _pdb_interrupt_enable
{
    kPDB_SequenceErrorInterruptEnable = PDB_SC_PDBEIE_MASK, /*!< PDB sequence error interrupt enable. */
    kPDB_DelayInterruptEnable         = PDB_SC_PDBIE_MASK,  /*!< PDB delay interrupt enable. */
};

/*!
 * @brief PDB load value mode.
 *
 * Selects the mode to load the internal values after doing the load operation (write 1 to PDBx_SC[LDOK]).
 * These values are for:
 *  - PDB counter (PDBx_MOD, PDBx_IDLY)
 *  - ADC trigger (PDBx_CHnDLYm)
 *  - DAC trigger (PDBx_DACINTx)
 *  - CMP trigger (PDBx_POyDLY)
 */
typedef enum _pdb_load_value_mode
{
    kPDB_LoadValueImmediately                     = 0U, /*!< Load immediately after 1 is written to LDOK. */
    kPDB_LoadValueOnCounterOverflow               = 1U, /*!< Load when the PDB counter overflows (reaches the MOD
                                                             register value). */
    kPDB_LoadValueOnTriggerInput                  = 2U, /*!< Load a trigger input event is detected. */
    kPDB_LoadValueOnCounterOverflowOrTriggerInput = 3U, /*!< Load either when the PDB counter overflows or a trigger
                                                             input is detected. */
} pdb_load_value_mode_t;

/*!
 * @brief Prescaler divider.
 *
 * Counting uses the peripheral clock divided by multiplication factor selected by times of MULT.
 */
typedef enum _pdb_prescaler_divider
{
    kPDB_PrescalerDivider1   = 0U, /*!< Divider x1. */
    kPDB_PrescalerDivider2   = 1U, /*!< Divider x2. */
    kPDB_PrescalerDivider4   = 2U, /*!< Divider x4. */
    kPDB_PrescalerDivider8   = 3U, /*!< Divider x8. */
    kPDB_PrescalerDivider16  = 4U, /*!< Divider x16. */
    kPDB_PrescalerDivider32  = 5U, /*!< Divider x32. */
    kPDB_PrescalerDivider64  = 6U, /*!< Divider x64. */
    kPDB_PrescalerDivider128 = 7U, /*!< Divider x128. */
} pdb_prescaler_divider_t;

/*!
 * @brief Multiplication factor select for prescaler.
 *
 * Selects the multiplication factor of the prescaler divider for the counter clock.
 */
typedef enum _pdb_divider_multiplication_factor
{
    kPDB_DividerMultiplicationFactor1  = 0U, /*!< Multiplication factor is 1. */
    kPDB_DividerMultiplicationFactor10 = 1U, /*!< Multiplication factor is 10. */
    kPDB_DividerMultiplicationFactor20 = 2U, /*!< Multiplication factor is 20. */
    kPDB_DividerMultiplicationFactor40 = 3U, /*!< Multiplication factor is 40. */
} pdb_divider_multiplication_factor_t;

/*!
 * @brief Trigger input source
 *
 * Selects the trigger input source for the PDB. The trigger input source can be internal or external (EXTRG pin), or
 * the software trigger. Refer to chip configuration details for the actual PDB input trigger connections.
 */
typedef enum _pdb_trigger_input_source
{
    kPDB_TriggerInput0   = 0U,  /*!< Trigger-In 0. */
    kPDB_TriggerInput1   = 1U,  /*!< Trigger-In 1. */
    kPDB_TriggerInput2   = 2U,  /*!< Trigger-In 2. */
    kPDB_TriggerInput3   = 3U,  /*!< Trigger-In 3. */
    kPDB_TriggerInput4   = 4U,  /*!< Trigger-In 4. */
    kPDB_TriggerInput5   = 5U,  /*!< Trigger-In 5. */
    kPDB_TriggerInput6   = 6U,  /*!< Trigger-In 6. */
    kPDB_TriggerInput7   = 7U,  /*!< Trigger-In 7. */
    kPDB_TriggerInput8   = 8U,  /*!< Trigger-In 8. */
    kPDB_TriggerInput9   = 9U,  /*!< Trigger-In 9. */
    kPDB_TriggerInput10  = 10U, /*!< Trigger-In 10. */
    kPDB_TriggerInput11  = 11U, /*!< Trigger-In 11. */
    kPDB_TriggerInput12  = 12U, /*!< Trigger-In 12. */
    kPDB_TriggerInput13  = 13U, /*!< Trigger-In 13. */
    kPDB_TriggerInput14  = 14U, /*!< Trigger-In 14. */
    kPDB_TriggerSoftware = 15U, /*!< Trigger-In 15. */
} pdb_trigger_input_source_t;

/*!
 * @brief PDB module configuration.
 */
typedef struct _pdb_config
{
    pdb_load_value_mode_t               loadValueMode;               /*!< Select the load value mode. */
    pdb_prescaler_divider_t             prescalerDivider;            /*!< Select the prescaler divider. */
    pdb_divider_multiplication_factor_t dividerMultiplicationFactor; /*!< Multiplication factor select for prescaler. */
    pdb_trigger_input_source_t          triggerInputSource;          /*!< Select the trigger input source. */
    bool                                enableContinuousMode;        /*!< Enable the PDB operation in Continuous mode.*/
} pdb_config_t;

/*!
 * @brief PDB ADC Pre-Trigger configuration.
 */
typedef struct _pdb_adc_pretrigger_config
{
    uint32_t enablePreTriggerMask;          /*!< PDB Channel Pre-Trigger Enable. */
    uint32_t enableOutputMask;              /*!< PDB Channel Pre-Trigger Output Select.
                                                 PDB channel's corresponding pre-trigger asserts when the counter
                                                 reaches the channel delay register. */
    uint32_t enableBackToBackOperationMask; /*!< PDB Channel Pre-Trigger Back-to-Back Operation Enable.
                                                 Back-to-back operation enables the ADC conversions complete to trigger
                                                 the next PDB channel pre-trigger and trigger output, so that the ADC
                                                 conversions can be triggered on next set of configuration and results
                                                 registers.*/
} pdb_adc_pretrigger_config_t;

/*!
 * @brief PDB DAC trigger configuration.
 */
typedef struct _pdb_dac_trigger_config
{
    bool enableExternalTriggerInput; /*!< Enables the external trigger for DAC interval counter. */
    bool enableIntervalTrigger;      /*!< Enables the DAC interval trigger. */
} pdb_dac_trigger_config_t;

/*******************************************************************************
 * API
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name Initialization
 * @{
 */

/*!
 * @brief Initialization for the PDB module.
 *
 * This function is to make the initialization for PDB module. The operations includes are:
 *  - Enable the clock for PDB instance.
 *  - Configure the PDB module.
 *  - Enable the PDB module.
 *
 * @param base PDB peripheral base address.
 * @param config Pointer to configuration structure. See to "pdb_config_t".
 */
void PDB_Init(PDB_Type *base, const pdb_config_t *config);

/*!
 * @brief De-initialization for the PDB module.
 *
 * @param base PDB peripheral base address.
 */
void PDB_Deinit(PDB_Type *base);

/*!
 * @brief Initialize PDB user configure structure.
 *
 * This function initializes the user configure structure to default value. the default value are:
 * @code
 *   config->loadValueMode = kPDB_LoadValueImmediately;
 *   config->prescalerDivider = kPDB_PrescalerDivider1;
 *   config->dividerMultiplicationFactor = kPDB_DividerMultiplicationFactor1;
 *   config->triggerInputSource = kPDB_TriggerSoftware;
 *   config->enableContinuousMode = false;
 * @endcode
 * @param config Pointer to configuration structure. See to "pdb_config_t".
 */
void PDB_GetDefaultConfig(pdb_config_t *config);

/*!
 * @brief Enable the PDB module.
 *
 * @param base PDB peripheral base address.
 * @param enable Enable the module or not.
 */
static inline void PDB_Enable(PDB_Type *base, bool enable)
{
    if (enable)
    {
        base->SC |= PDB_SC_PDBEN_MASK;
    }
    else
    {
        base->SC &= ~PDB_SC_PDBEN_MASK;
    }
}

/* @} */

/*!
 * @name Basic Counter
 * @{
 */

/*!
 * @brief Do trigger the PDB counter by software.
 *
 * @param base PDB peripheral base address.
 */
static inline void PDB_DoSoftwareTrigger(PDB_Type *base)
{
    base->SC |= PDB_SC_SWTRIG_MASK;
}

/*!
 * @brief Do load the counter values.
 *
 * This function is to load the counter values from their internal buffer.
 * See to "pdb_load_value_mode_t" about PDB's load mode.
 *
 * @param base PDB peripheral base address.
 */
static inline void PDB_DoLoadValues(PDB_Type *base)
{
    base->SC |= PDB_SC_LDOK_MASK;
}

/*!
 * @brief Enable the DMA for the PDB module.
 *
 * @param base PDB peripheral base address.
 * @param enable Enable the feature or not.
 */
static inline void PDB_EnableDMA(PDB_Type *base, bool enable)
{
    if (enable)
    {
        base->SC |= PDB_SC_DMAEN_MASK;
    }
    else
    {
        base->SC &= ~PDB_SC_DMAEN_MASK;
    }
}

/*!
 * @brief Enable the interrupts for the PDB module.
 *
 * @param base PDB peripheral base address.
 * @param mask Mask value for interrupts. See to "_pdb_interrupt_enable".
 */
static inline void PDB_EnableInterrupts(PDB_Type *base, uint32_t mask)
{
    assert(0U == (mask & ~(PDB_SC_PDBEIE_MASK | PDB_SC_PDBIE_MASK)));

    base->SC |= mask;
}

/*!
 * @brief Disable the interrupts for the PDB module.
 *
 * @param base PDB peripheral base address.
 * @param mask Mask value for interrupts. See to "_pdb_interrupt_enable".
 */
static inline void PDB_DisableInterrupts(PDB_Type *base, uint32_t mask)
{
    assert(0U == (mask & ~(PDB_SC_PDBEIE_MASK | PDB_SC_PDBIE_MASK)));

    base->SC &= ~mask;
}

/*!
 * @brief  Get the status flags of the PDB module.
 *
 * @param  base PDB peripheral base address.
 *
 * @return      Mask value for asserted flags. See to "_pdb_status_flags".
 */
static inline uint32_t PDB_GetStatusFlags(PDB_Type *base)
{
    return base->SC & (PDB_SC_PDBIF_MASK | PDB_SC_LDOK_MASK);
}

/*!
 * @brief Clear the status flags of the PDB module.
 *
 * @param base PDB peripheral base address.
 * @param mask Mask value of flags. See to "_pdb_status_flags".
 */
static inline void PDB_ClearStatusFlags(PDB_Type *base, uint32_t mask)
{
    assert(0U == (mask & ~PDB_SC_PDBIF_MASK));

    base->SC &= ~mask;
}

/*!
 * @brief  Specify the period of the counter.
 *
 * @param  base  PDB peripheral base address.
 * @param  value Setting value for the modulus. 16-bit is available.
 */
static inline void PDB_SetModulusValue(PDB_Type *base, uint32_t value)
{
    base->MOD = PDB_MOD_MOD(value);
}

/*!
 * @brief  Get PDB counter's current value.
 *
 * @param  base PDB peripheral base address.
 *
 * @return      PDB counter's current value.
 */
static inline uint32_t PDB_GetCounterValue(PDB_Type *base)
{
    return base->CNT;
}

/*!
 * @brief Set the value for PDB counter delay event.
 *
 * @param base  PDB peripheral base address.
 * @param value Setting value for PDB counter delay event. 16-bit is available.
 */
static inline void PDB_SetCounterDelayValue(PDB_Type *base, uint32_t value)
{
    base->IDLY = PDB_IDLY_IDLY(value);
}
/* @} */

/*!
 * @name ADC Pre-Trigger
 * @{
 */

/*!
 * @brief Configure the ADC PreTrigger in PDB module.
 *
 * @param base    PDB peripheral base address.
 * @param channel Channel index for ADC instance.
 * @param config  Pointer to configuration structure. See to "pdb_adc_pretrigger_config_t".
 */
static inline void PDB_SetADCPreTriggerConfig(PDB_Type *base, uint32_t channel, pdb_adc_pretrigger_config_t *config)
{
    assert(channel < PDB_C1_COUNT);
    assert(NULL != config);

    base->CH[channel].C1 = PDB_C1_BB(config->enableBackToBackOperationMask) | PDB_C1_TOS(config->enableOutputMask) |
                           PDB_C1_EN(config->enableOutputMask);
}

/*!
 * @brief Set the value for ADC Pre-Trigger delay event.
 *
 * This function is to set the value for ADC Pre-Trigger delay event. IT Specifies the delay value for the channel's
 * corresponding pre-trigger. The pre-trigger asserts when the PDB counter is equal to the setting value here. 
 *
 * @param base       PDB peripheral base address.
 * @param channel    Channel index for ADC instance.
 * @param preChannel Channel group index for ADC instance.
 * @param value      Setting value for ADC Pre-Trigger delay event. 16-bit is available.
 */
static inline void PDB_SetADCPreTriggerDelayValue(PDB_Type *base, uint32_t channel, uint32_t preChannel, uint32_t value)
{
    assert(channel < PDB_C1_COUNT);
    assert(preChannel < PDB_DLY_COUNT);

    base->CH[channel].DLY[preChannel] = PDB_DLY_DLY(value);
}

/*!
 * @brief  Get the ADC Pre-Trigger's status flags.
 *
 * @param  base    PDB peripheral base address.
 * @param  channel Channel index for ADC instance.
 *
 * @return         Mask value for asserted flags. See to "_pdb_adc_pretrigger_flags".
 */
static inline uint32_t PDB_GetADCPreTriggerStatusFlags(PDB_Type *base, uint32_t channel)
{
    assert(channel < PDB_C1_COUNT);

    return base->CH[channel].S;
}

/*!
 * @brief Clear the ADC Pre-Trigger's status flags.
 *
 * @param base    PDB peripheral base address.
 * @param channel Channel index for ADC instance.
 * @param mask    Mask value for flags. See to "_pdb_adc_pretrigger_flags".
 */
static inline void PDB_ClearADCPreTriggerStatusFlags(PDB_Type *base, uint32_t channel, uint32_t mask)
{
    assert(channel < PDB_C1_COUNT);

    base->CH[channel].S &= ~mask;
}

/* @} */

#if defined(FSL_FEATURE_PDB_HAS_DAC) && FSL_FEATURE_PDB_HAS_DAC
/*!
 * @name DAC Interval Trigger
 * @{
 */

/*!
 * @brief Configure the DAC trigger in PDB module.
 *
 * @param base    PDB peripheral base address.
 * @param channel Channel index for DAC instance.
 * @param config  Pointer to configuration structure. See to "pdb_dac_trigger_config_t".
 */
void PDB_SetDACTriggerConfig(PDB_Type *base, uint32_t channel, pdb_dac_trigger_config_t *config);

/*!
 * @brief Set value for the DAC interval event.
 *
 * This fucntion is to set the value for DAC interval event. DAC interval trigger would trigger the DAC module to update
 * buffer when the DAC interval counter is equal to the setting value here.
 *
 * @param base    PDB peripheral base address.
 * @param channel Channel index for DAC instance.
 * @param value   Setting value for the DAC interval event.
 */
static inline void PDB_SetDACTriggerIntervalValue(PDB_Type *base, uint32_t channel, uint32_t value)
{
    assert(channel < PDB_INT_COUNT);

    base->DAC[channel].INT = PDB_INT_INT(value);
}

/* @} */
#endif /* FSL_FEATURE_PDB_HAS_DAC */

/*!
 * @name Pulse-Out Trigger
 * @{
 */

/*!
 * @brief Enable the pulse out trigger channels.
 *
 * @param base        PDB peripheral base address.
 * @param channelMask Channel mask value for multiple pulse out trigger channel.
 * @param enable Enable the feature or not.
 */
static inline void PDB_EnablePulseOutTrigger(PDB_Type *base, uint32_t channelMask, bool enable)
{
    if (enable)
    {
        base->POEN |= PDB_POEN_POEN(channelMask);
    }
    else
    {
        base->POEN &= ~(PDB_POEN_POEN(channelMask));
    }
}

/*!
 * @brief Setting event values for pulse out trigger.
 *
 * This function is to set event values for pulse output trigger.
 * These pulse output trigger delay values specify the delay for the PDB Pulse-Out. Pulse-Out goes high when the PDB
 * counter is equal to the pulse output high value (value1). Pulse-Out goes low when the PDB counter is equal to the
 * pulse output low value (value2).
 *
 * @param base    PDB peripheral base address.
 * @param channel Channel index for pulse out trigger channel.
 * @param value1  Setting value for pulse out high.
 * @param value2  Setting value for pulse out low.
 */
static inline void PDB_SetPulseOutTriggerDelayValue(PDB_Type *base, uint32_t channel, uint32_t value1, uint32_t value2)
{
    assert(channel < PDB_PODLY_COUNT);

    base->PODLY[channel] = PDB_PODLY_DLY1(value1) | PDB_PODLY_DLY2(value2);
}

/* @} */
#if defined(__cplusplus)
}
#endif
/*
 * @}
 */
#endif /* _FSL_PDB_H_ */
