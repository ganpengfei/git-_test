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

#ifndef _FSL_CMP_H_
#define _FSL_CMP_H_

#include "fsl_common.h"

/*!
 * @addtogroup cmp
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define FSL_CMP_DRIVER_VERSION (MAKE_VERSION(2, 0, 0)) /*!< Version 2.0.0. */

/*!
* @brief Interrupt enable/disable mask.
*/
enum _cmp_interrupt_enable
{
    kCMP_OutputRisingInterruptEnable = CMP_SCR_IER_MASK,  /*!< Comparator interrupt enable rising. */
    kCMP_OutputFallingInterruptEnable = CMP_SCR_IEF_MASK, /*!< Comparator interrupt enable falling. */
};

/*!
 * @brief Status flags' mask.
 */
enum _cmp_status_flags
{
    kCMP_OutputRisingEventFlag = CMP_SCR_CFR_MASK,  /*!< Rising-edge on compare output has occurred. */
    kCMP_OutputFallingEventFlag = CMP_SCR_CFF_MASK, /*!< Falling-edge on compare output has occurred. */
    kCMP_OutputAssertEventFlag = CMP_SCR_COUT_MASK, /*!< Return the current value of the analog comparator output. */
};

/*!
 * @brief CMP Hysteresis mode.
 */
typedef enum _cmp_hysteresis_mode
{
    kCMP_HysteresisLevel0 = 0U, /*!< Hysteresis level 0. */
    kCMP_HysteresisLevel1 = 1U, /*!< Hysteresis level 1. */
    kCMP_HysteresisLevel2 = 2U, /*!< Hysteresis level 2. */
    kCMP_HysteresisLevel3 = 3U, /*!< Hysteresis level 3. */
} cmp_hysteresis_mode_t;

/*!
 * @brief CMP Voltage Reference source.
 */
typedef enum _cmp_reference_voltage_source
{
    kCMP_VrefSourceVin1 = 0U, /*!< Vin1 is selected as resistor ladder network supply reference Vin. */
    kCMP_VrefSourceVin2 = 1U, /*!< Vin2 is selected as resistor ladder network supply reference Vin. */
} cmp_reference_voltage_source_t;

/*!
 * @brief Configure the comparator.
 */
typedef struct _cmp_config
{
    bool enableCmp;                       /*!< Enable the CMP module. */
    cmp_hysteresis_mode_t hysteresisMode; /*!< CMP Hysteresis mode. */
    bool enableHighSpeed;                 /*!< Enable High Speed (HS) comparison mode. */
    bool enableInvertOutput;              /*!< Enable inverted comparator output. */
    bool useUnfilteredOutput;             /*!< Set compare output(COUT) to equal COUTA(true) or COUT(false). */
    bool enablePinOut;                    /*!< The comparator output is available on the associated pin. */
#if defined(FSL_FEATURE_CMP_HAS_TRIGGER_MODE) && FSL_FEATURE_CMP_HAS_TRIGGER_MODE
    bool enableTriggerMode; /*!< Enable the trigger mode. */
#endif                      /* FSL_FEATURE_CMP_HAS_TRIGGER_MODE */
} cmp_config_t;

/*!
 * @brief Configure the filter.
 */
typedef struct _cmp_filter_config
{
#if defined(FSL_FEATURE_CMP_HAS_EXTERNAL_SAMPLE_SUPPORT) && FSL_FEATURE_CMP_HAS_EXTERNAL_SAMPLE_SUPPORT
    bool enableSample;    /*!< Using external SAMPLE as sampling clock input, or using divided bus clock. */
#endif                    /* FSL_FEATURE_CMP_HAS_EXTERNAL_SAMPLE_SUPPORT */
    uint8_t filterCount;  /*!< Filter Sample Count. Available range is 1-7, 0 would cause the filter disabled.*/
    uint8_t filterPeriod; /*!< Filter Sample Period. The divider to bus clock. Available range is 0-255. */
} cmp_filter_config_t;

/*!
 * @brief Configure the internal DAC.
 */
typedef struct _cmp_dac_config
{
    cmp_reference_voltage_source_t referenceVoltageSource; /*!< Supply voltage reference source. */
    uint8_t DACValue;                                      /*!< Value for DAC Output Voltage. Available range is 0-63.*/
} cmp_dac_config_t;

#if defined(__cplusplus)
extern "C" {
#endif

/*******************************************************************************
 * API
 ******************************************************************************/

/*!
 * @name Initialization
 * @{
 */

/*!
 * @brief Initialization for the CMP.
 *
 * This function is to initialize the CMP module. The operations included are:
 * - Enables the clock for CMP module.
 * - Configure the comparator.
 * - Enable the CMP module.
 * Note: For some devices, multiple CMP instance shares the same clock gate. So in this case, to enable the clock for
 * any instance would enable all the CMPs. User needs to check the chip reference manual for clock assignment of CMP.
 *
 * @param base   CMP peripheral base address.
 * @param config Pointer to configuration structure.
 */
void CMP_Init(CMP_Type *base, const cmp_config_t *config);

/*!
 * @brief De-initialization for the CMP module.
 *
 * This function makes the de-initialization for the CMP module. The operations includes are:
 * - Disable the CMP module.
 * - Disable the clock for CMP module.
 *
 * This function is to de-initialize the CMP module. It disables the clock for the CMP.
 * Note: For some devices, multiple CMP instance shares the same clock gate. So in this case, before disabling the
 * clock for CMP, please make sure all the CMP instances are not used.
 *
 * @param base CMP peripheral base address.
 */
void CMP_Deinit(CMP_Type *base);

/*!
 * @brief Enable/Disable the CMP module.
 *
 * @param base CMP peripheral base address.
 * @param enable Enable the module or not.
 */
static inline void CMP_Enable(CMP_Type *base, bool enable)
{
    if (enable)
    {
        base->CR1 |= CMP_CR1_EN_MASK;
    }
    else
    {
        base->CR1 &= ~CMP_CR1_EN_MASK;
    }
}

/*!
* @brief Initializes CMP user configure structure.
*
* This function initializes the user configure structure to default value. the default value are:
* @code
*   config->enableCmp           = true;
*   config->hysteresisMode      = kCMP_HysteresisLevel0;
*   config->enableHighSpeed     = false;
*   config->enableInvertOutput  = false;
*   config->useUnfilteredOutput = false;
*   config->enablePinOut        = false;
*   config->enableTriggerMode   = false;
* @endcode
* @param config Pointer to configuration structure.
*/
void CMP_GetDefaultConfig(cmp_config_t *config);

/*!
 * @brief  Set the input channels for comparator.
 *
 * This function is to set the input channels for comparator.
 * Note: It is not available to set the two input channel as same in application. When the user selects the same input
 * from analog mux to the positive and negative port, the comparator is disabled automatically.
 *
 * @param  base            CMP peripheral base address.
 * @param  positiveChannel Positive side input channel number. Available range is 0-7.
 * @param  negativeChannel Negative side input channel number. Available range is 0-7.
 */
void CMP_SetInputChannels(CMP_Type *base, uint8_t positiveChannel, uint8_t negativeChannel);

/* @} */

/*!
 * @name Advanced Features
 * @{
 */

#if defined(FSL_FEATURE_CMP_HAS_DMA) && FSL_FEATURE_CMP_HAS_DMA
/*!
 * @brief Enable/Disable the DMA request for rising/falling event.
 *
 * This function enables/disables the DMA request for rising/falling event. Either event would trigger the generation of
 * the DMA
 * request from CMP if the DMA feature is enabled. Both events would be ignored for generating the DMA request from CMP
 * if the
 * DMA is disabled.
 *
 * @param base CMP peripheral base address.
 * @param enable Enable the feature or not.
 */
void CMP_EnableDMA(CMP_Type *base, bool enable);
#endif /* FSL_FEATURE_CMP_HAS_DMA */

#if defined(FSL_FEATURE_CMP_HAS_WINDOW_MODE) && FSL_FEATURE_CMP_HAS_WINDOW_MODE
/*!
 * @brief Enable/Disable the window mode.
 *
 * @param base CMP peripheral base address.
 * @param enable Enable the feature or not.
 */
static inline void CMP_EnableWindowMode(CMP_Type *base, bool enable)
{
    if (enable)
    {
        base->CR1 |= CMP_CR1_WE_MASK;
    }
    else
    {
        base->CR1 &= ~CMP_CR1_WE_MASK;
    }
}
#endif /* FSL_FEATURE_CMP_HAS_WINDOW_MODE */

#if defined(FSL_FEATURE_CMP_HAS_PASS_THROUGH_MODE) && FSL_FEATURE_CMP_HAS_PASS_THROUGH_MODE
/*!
 * @brief Enable/Disable the pass through mode.
 *
 * @param base CMP peripheral base address.
 * @param enable Enable the feature or not.
 */
static inline void CMP_EnablePassThroughMode(CMP_Type *base, bool enable)
{
    if (enable)
    {
        base->MUXCR |= CMP_MUXCR_PSTM_MASK;
    }
    else
    {
        base->MUXCR &= ~CMP_MUXCR_PSTM_MASK;
    }
}
#endif /* FSL_FEATURE_CMP_HAS_PASS_THROUGH_MODE */

/*!
 * @brief  Configure the filter.
 *
 * @param  base   CMP peripheral base address.
 * @param  config Pointer to configuration structure.
 */
void CMP_SetFilterConfig(CMP_Type *base, const cmp_filter_config_t *config);

/*!
 * @brief Configure the internal DAC.
 *
 * @param base   CMP peripheral base address.
 * @param config Pointer to configuration structure. "NULL" is for disabling the feature.
 */
void CMP_SetDACConfig(CMP_Type *base, const cmp_dac_config_t *config);

/*!
 * @brief Enable the interrupts.
 *
 * @param base    CMP peripheral base address.
 * @param mask    Mask value for interrupts. See to "_cmp_interrupt_enable".
 */
void CMP_EnableInterrupts(CMP_Type *base, uint32_t mask);

/*!
 * @brief Disable the interrupts.
 *
 * @param base    CMP peripheral base address.
 * @param mask    Mask value for interrupts. See to "_cmp_interrupt_enable".
 */
void CMP_DisableInterrupts(CMP_Type *base, uint32_t mask);

/* @} */

/*!
 * @name Results
 * @{
 */

/*!
 * @brief  Get the status flags.
 *
 * @param  base     CMP peripheral base address.
 *
 * @return          Mask value for the asserted flags. See to "_cmp_status_flags".
 */
uint32_t CMP_GetStatusFlags(CMP_Type *base);

/*!
 * @brief Clear the status flags.
 *
 * @param base     CMP peripheral base address.
 * @param mask     Mask value for the flags. See to "_cmp_status_flags".
 */
void CMP_ClearStatusFlags(CMP_Type *base, uint32_t mask);

/* @} */
#if defined(__cplusplus)
}
#endif
/*
 * @}
 */
#endif /* _FSL_CMP_H_ */
