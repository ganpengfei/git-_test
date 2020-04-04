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
#ifndef _FSL_LPUART_H_
#define _FSL_LPUART_H_

#include "fsl_common.h"

/*!
 * @addtogroup lpuart
 * @{
 */

/*! @file*/

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief LPUART driver version */
#define FSL_LPUART_DRIVER_VERSION (MAKE_VERSION(2, 0, 0)) /*!< Version 2.0.0. */

/*! @brief Error codes for the LPUART driver. */
enum _lpuart_status
{
    kStatus_LPUART_TxBusy = MAKE_STATUS(kStatusGroup_LPUART, 0),              /*!< TX busy */
    kStatus_LPUART_RxBusy = MAKE_STATUS(kStatusGroup_LPUART, 1),              /*!< RX busy */
    kStatus_LPUART_TxIdle = MAKE_STATUS(kStatusGroup_LPUART, 2),              /*!< LPUART transmitter is idle. */
    kStatus_LPUART_RxIdle = MAKE_STATUS(kStatusGroup_LPUART, 3),              /*!< LPUART receiver is idle. */
    kStatus_LPUART_TxWatermarkTooLarge = MAKE_STATUS(kStatusGroup_LPUART, 4), /*!< TX FIFO watermark too large  */
    kStatus_LPUART_RxWatermarkTooLarge = MAKE_STATUS(kStatusGroup_LPUART, 5), /*!< RX FIFO watermark too large  */
    kStatus_LPUART_FlagCannotClearManually =
        MAKE_STATUS(kStatusGroup_LPUART, 6),                    /*!< Some flag can not manually clear */
    kStatus_LPUART_Error = MAKE_STATUS(kStatusGroup_LPUART, 7), /*!< Error happens on LPUART. */
    kStatus_LPUART_RxRingBufferOverrun =
        MAKE_STATUS(kStatusGroup_LPUART, 8), /*!< LPUART RX software ring buffer overrun. */
    kStatus_LPUART_RxHardwareOverrun = MAKE_STATUS(kStatusGroup_LPUART, 9) /*!< LPUART RX receiver overrun. */
};

/*<! @brief LPUART parity mode. */
typedef enum _lpuart_parity_mode
{
    kLPUART_ParityDisabled = 0x0U, /*!< Parity disabled */
    kLPUART_ParityEven = 0x2U,     /*!< Parity enabled, type even, bit setting: PE|PT = 10 */
    kLPUART_ParityOdd = 0x3U,      /*!< Parity enabled, type odd,  bit setting: PE|PT = 11 */
} lpuart_parity_mode_t;

/*! @brief LPUART stop bit count. */
typedef enum _lpuart_stop_bit_count
{
    kLPUART_OneStopBit = 0U, /*!< One stop bit */
    kLPUART_TwoStopBit = 1U, /*!< Two stop bits */
} lpuart_stop_bit_count_t;

/*!
 * @brief LPUART interrupt configuration structure, default settings all disabled.
 *
 * This structure contains the settings for all of the LPUART interrupt configurations.
 */
enum _lpuart_interrupt_enable
{
#if defined(FSL_FEATURE_LPUART_HAS_LIN_BREAK_DETECT) && FSL_FEATURE_LPUART_HAS_LIN_BREAK_DETECT
    kLPUART_LinBreakInterruptEnable = (LPUART_BAUD_LBKDIE_MASK >> 8), /*!< LIN break detect. */
#endif
    kLPUART_RxActiveEdgeInterruptEnable = (LPUART_BAUD_RXEDGIE_MASK >> 8), /*!< Receive Active Edge. */
    kLPUART_TxDataRegEmptyInterruptEnable = (LPUART_CTRL_TIE_MASK),        /*!< Transmit data register empty. */
    kLPUART_TransmissionCompleteInterruptEnable = (LPUART_CTRL_TCIE_MASK), /*!< Transmission complete. */
    kLPUART_RxDataRegFullInterruptEnable = (LPUART_CTRL_RIE_MASK),         /*!< Receiver data register full. */
    kLPUART_IdleLineInterruptEnable = (LPUART_CTRL_ILIE_MASK),             /*!< Idle line. */
    kLPUART_RxOverrunInterruptEnable = (LPUART_CTRL_ORIE_MASK),            /*!< Receiver Overrun. */
    kLPUART_NoiseErrorInterruptEnable = (LPUART_CTRL_NEIE_MASK),           /*!< Noise error flag. */
    kLPUART_FramingErrorInterruptEnable = (LPUART_CTRL_FEIE_MASK),         /*!< Framing error flag. */
    kLPUART_ParityErrorInterruptEnable = (LPUART_CTRL_PEIE_MASK),          /*!< Parity error flag. */
#if defined(FSL_FEATURE_LPUART_HAS_FIFO) && FSL_FEATURE_LPUART_HAS_FIFO
    kLPUART_TxFifoOverflowInterruptEnable = (LPUART_FIFO_TXOFE_MASK >> 8),  /*!< Transmit FIFO Overflow. */
    kLPUART_RxFifoUnderflowInterruptEnable = (LPUART_FIFO_RXUFE_MASK >> 8), /*!< Receive FIFO Underflow. */
#endif
};

/*!
 * @brief LPUART status flags.
 *
 * This provides constants for the LPUART status flags for use in the LPUART functions.
 */
enum _lpuart_flags
{
    kLPUART_TxDataRegEmptyFlag =
        (LPUART_STAT_TDRE_MASK), /*!< Transmit data register empty flag, sets when transmit buffer is empty */
    kLPUART_TransmissionCompleteFlag =
        (LPUART_STAT_TC_MASK), /*!< Transmission complete flag, sets when transmission activity complete */
    kLPUART_RxDataRegFullFlag =
        (LPUART_STAT_RDRF_MASK), /*!< Receive data register full flag, sets when the receive data buffer is full */
    kLPUART_IdleLineFlag = (LPUART_STAT_IDLE_MASK), /*!< Idle line detect flag, sets when idle line detected */
    kLPUART_RxOverrunFlag = (LPUART_STAT_OR_MASK),  /*!< Receive Overrun, sets when new data is received before data is
                                                       read from receive register */
    kLPUART_NoiseErrorFlag = (LPUART_STAT_NF_MASK), /*!< Receive takes 3 samples of each received bit.  If any of these
                                                       samples differ, noise flag sets */
    kLPUART_FramingErrorFlag =
        (LPUART_STAT_FE_MASK), /*!< Frame error flag, sets if logic 0 was detected where stop bit expected */
    kLPUART_ParityErrorFlag = (LPUART_STAT_PF_MASK), /*!< If parity enabled, sets upon parity error detection */
#if defined(FSL_FEATURE_LPUART_HAS_LIN_BREAK_DETECT) && FSL_FEATURE_LPUART_HAS_LIN_BREAK_DETECT
    kLPUART_LinBreakFlag = (LPUART_STAT_LBKDIF_MASK), /*!< LIN break detect interrupt flag, sets when LIN break char
                                                         detected and LIN circuit enabled */
#endif
    kLPUART_RxActiveEdgeFlag =
        (LPUART_STAT_RXEDGIF_MASK), /*!< Receive pin active edge interrupt flag, sets when active edge detected */
    kLPUART_RxActiveFlag =
        (LPUART_STAT_RAF_MASK), /*!< Receiver Active Flag (RAF), sets at beginning of valid start bit */
#if defined(FSL_FEATURE_LPUART_HAS_ADDRESS_MATCHING) && FSL_FEATURE_LPUART_HAS_ADDRESS_MATCHING
    kLPUART_DataMatch1Flag = LPUART_STAT_MA1F_MASK, /*!< The next character to be read from LPUART_DATA matches MA1*/
    kLPUART_DataMatch2Flag = LPUART_STAT_MA2F_MASK, /*!< The next character to be read from LPUART_DATA matches MA2*/
#endif
#if defined(FSL_FEATURE_LPUART_HAS_EXTENDED_DATA_REGISTER_FLAGS) && FSL_FEATURE_LPUART_HAS_EXTENDED_DATA_REGISTER_FLAGS
    kLPUART_NoiseErrorInRxDataRegFlag =
        (LPUART_DATA_NOISY_MASK >> 10), /*!< NOISY bit, sets if noise detected in current data word */
    kLPUART_ParityErrorInRxDataRegFlag =
        (LPUART_DATA_PARITYE_MASK >> 10), /*!< PARITYE bit, sets if noise detected in current data word */
#endif
#if defined(FSL_FEATURE_LPUART_HAS_FIFO) && FSL_FEATURE_LPUART_HAS_FIFO
    kLPUART_TxFifoEmptyFlag = (LPUART_FIFO_TXEMPT_MASK >> 16), /*!< TXEMPT bit, sets if transmit buffer is empty */
    kLPUART_RxFifoEmptyFlag = (LPUART_FIFO_RXEMPT_MASK >> 16), /*!< RXEMPT bit, sets if receive buffer is empty */
    kLPUART_TxFifoOverflowFlag =
        (LPUART_FIFO_TXOF_MASK >> 16), /*!< TXOF bit, sets if transmit buffer overflow occurred */
    kLPUART_RxFifoUnderflowFlag =
        (LPUART_FIFO_RXUF_MASK >> 16), /*!< RXUF bit, sets if receive buffer underflow occurred */
#endif
};

/*! @brief LPUART configure structure. */
typedef struct _lpuart_config
{
    uint32_t baudRate_Bps;           /*!< LPUART baud rate  */
    lpuart_parity_mode_t parityMode; /*!< Parity mode, disabled (default), even, odd */
#if defined(FSL_FEATURE_LPUART_HAS_STOP_BIT_CONFIG_SUPPORT) && FSL_FEATURE_LPUART_HAS_STOP_BIT_CONFIG_SUPPORT
    lpuart_stop_bit_count_t stopBitCount; /*!< Number of stop bits, 1 stop bit (default) or 2 stop bits  */
#endif
#if defined(FSL_FEATURE_LPUART_HAS_FIFO) && FSL_FEATURE_LPUART_HAS_FIFO
    uint8_t txFifoWatermark; /*!< TX FIFO watermark */
    uint8_t rxFifoWatermark; /*!< RX FIFO watermark */
#endif
    bool enableTx; /*!< Enable TX */
    bool enableRx; /*!< Enable RX */
} lpuart_config_t;

/*! @brief LPUART transfer structure. */
typedef struct _lpuart_transfer
{
    uint8_t *data;   /*!< The buffer of data to be transfer.*/
    size_t dataSize; /*!< The byte count to be transfer. */
} lpuart_transfer_t;

/* Forward declaration of the handle typedef. */
typedef struct _lpuart_handle lpuart_handle_t;

/*! @brief LPUART transfer callback function. */
typedef void (*lpuart_transfer_callback_t)(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData);

/*! @brief LPUART handle structure. */
struct _lpuart_handle
{
    uint8_t *volatile txData;   /*!< Address of remaining data to send. */
    volatile size_t txDataSize; /*!< Size of the remaining data to send. */
    uint8_t *volatile rxData;   /*!< Address of remaining data to receive. */
    volatile size_t rxDataSize; /*!< Size of the remaining data to receive. */

    uint8_t *rxRingBuffer;              /*!< Start address of the receiver ring buffer. */
    size_t rxRingBufferSize;            /*!< Size of the ring buffer. */
    volatile uint16_t rxRingBufferHead; /*!< Index for the driver to store received data into ring buffer. */
    volatile uint16_t rxRingBufferTail; /*!< Index for the user to get data from the ring buffer. */

    lpuart_transfer_callback_t callback; /*!< Callback function. */
    void *userData;                      /*!< Lpuart callback function parameter.*/

    volatile uint8_t txState; /*!< TX transfer state. */
    volatile uint8_t rxState; /*!< RX transfer state */
};

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /* _cplusplus */

/*!
 * @name Initialization and deinitialization
 * @{
 */

/*!
* @brief Initialize an LPUART instance with user configuration structure and peripheral clock.
*
* This function configures the LPUART module wiht user-defined settings. User can configure the configuration
* structure themselves and user can also get the defaul configuration by LPUART_GetDefaultConfig() function.
* Example below show how to use this API to configure the LPUART.
* @code
*  lpuart_config_t lpuartConfig;
*  lpuartConfig.baudRate_Bps = 115200U;
*  lpuartConfig.parityMode = kLPUART_ParityDisabled;
*  lpuartConfig.stopBitCount = kLPUART_OneStopBit;
*  lpuartConfig.txFifoWatermark = 0;
*  lpuartConfig.rxFifoWatermark = 1;
*  LPUART_Init(LPUART1, &lpuartConfig, 20000000U);
* @endcode
*
* @param base LPUART peripheral base address.
* @param config Pointer to user-defined configuration structure.
* @param sourceClockHz LPUART clock source freqency in HZ.
*/
void LPUART_Init(LPUART_Type *base, const lpuart_config_t *config, uint32_t srcClock_Hz);

/*!
 * @brief Deinitializes an LPUART instance.
 *
 * This function waits for TX complete, disables TX and RX, then disables the LPUART clock.
 *
 * @param base LPUART peripheral base address.
 */
void LPUART_Deinit(LPUART_Type *base);

/*!
 * @brief Get the default configuration structure.
 *
 * This function initializes the LPUART configure structure to default value. The default
 * value are:
 *   lpuartConfig->baudRate_Bps = 115200U;
 *   lpuartConfig->parityMode = kLPUART_ParityDisabled;
 *   lpuartConfig->stopBitCount = kLPUART_OneStopBit;
 *   lpuartConfig->txFifoWatermark = 0;
 *   lpuartConfig->rxFifoWatermark = 1;
 *   lpuartConfig->enableTx = false;
 *   lpuartConfig->enableRx = false;
 *
 * @param config Pointer to configuration structure.
 */
void LPUART_GetDefaultConfig(lpuart_config_t *config);

/*!
 * @brief Set LPUART instance baudrate.
 *
 * This function configures the LPUART module baudrate. This function is used to update
 * the LPUART module baudrate after the LPUART module is initialized by the LPUART_Init.
 * @code
 *  LPUART_SetBaudRate(LPUART1, 115200U, 20000000U);
 * @endcode
 *
 * @param base LpUART peripheral base address.
 * @param baudRate_Bps LPUART baudrate to be set.
 * @param srcClock_Hz LPUART clock source freqency in HZ.
 */
void LPUART_SetBaudRate(LPUART_Type *base, uint32_t baudRate_Bps, uint32_t srcClock_Hz);

/* @} */

/*!
 * @name Status
 * @{
 */

/*!
 * @brief Get LPUART status flags.
 *
 * This function get all LPUART status flags, the flags are returned as the logical
 * OR value of the enumerators @ref _lpuart_status_flags. To check specific status,
 * compare the return value with enumerators in @ref _lpuart_status_flags.
 * For example, to check whether the TX is empty:
 * @code
 *     if (kLPUART_TxDataRegEmptyFlag & LPUART_GetStatusFlags(LPUART1))
 *     {
 *         ...
 *     }
 * @endcode
 *
 * @param base LPUART peripheral base address.
 * @return LPUART status flags whic are ORed by the enumerators in the _lpuart_status_flags_t.
 */
uint32_t LPUART_GetStatusFlags(LPUART_Type *base);

/*!
 * @brief Clear status flags with provide mask.
 *
 * This function clears LPUART status flags with provided mask. Automatically cleared flag
 * can't be clear by this function.
 * Some flags can only clear or set by the hardware itself, these flags are:
 *    kLPUART_TxDataRegEmptyFlag, kLPUART_TransmissionCompleteFlag, kLPUART_RxDataRegFullFlag,
 *    kLPUART_RxActiveFlag, kLPUART_NoiseErrorInRxDataRegFlag, kLPUART_ParityErrorInRxDataRegFlag,
 *    kLPUART_TxFifoEmptyFlag,kLPUART_RxFifoEmptyFlag
 *
 * @param base LPUART peripheral base address.
 * @param mask the status flags to be cleared. User can use the enumerators in the
 *  _lpuart_status_flag_t to do the OR operation and get the mask.
 * @return 0 succeed, others failed.
 * @retval kStatus_LPUART_FlagCannotClearManually The flag can't be cleared by this function but
 *         it is cleared automatically by hardware.
 * @retval kStatus_Success Status in the mask are cleared.
 */
status_t LPUART_ClearStatusFlags(LPUART_Type *base, uint32_t mask);

/* @} */

/*!
 * @name Interrupts
 * @{
 */

/*!
 * @brief Enable LPUART interrupts according to provided mask.
 *
 * This function enable the LPUART interrupts according to provided mask. The mask
 * is a logical OR of enumeration members, see @ref _lpuart_interrupt_enable.
 * For example, to enable TX empty interrupt and RX full interrupt, do like this.
 * @code
 *     LPUART_EnableInterrupts(LPUART1,kLPUART_TxDataRegEmptyInterruptEnable | kLPUART_RxDataRegFullInterruptEnable);
 * @endcode
 *
 * @param base LPUART peripheral base address.
 * @param mask The interrupts to enable. Logical OR of @ref _uart_interrupt_enable.
 */
void LPUART_EnableInterrupts(LPUART_Type *base, uint32_t mask);

/*!
 * @brief Disable LPUART interrupts according to provided mask.
 *
 * This function disable the LPUART interrupts according to provided mask.The mask
 * is a logical OR of enumeration members, see @ref _lpuart_interrupt_enable.
 * For example, to disable TX empty interrupt and RX full interrupt, do like this.
 * @code
 *     LPUART_DisableInterrupts(LPUART1,kLPUART_TxDataRegEmptyInterruptEnable | kLPUART_RxDataRegFullInterruptEnable);
 * @endcode
 *
 * @param base LPUART peripheral base address.
 * @param mask The interrupts to disable. Logical OR of @ref _lpuart_interrupt_enable.
 */
void LPUART_DisableInterrupts(LPUART_Type *base, uint32_t mask);

/*!
 * @brief Get enabled LPUART interrupts.
 *
 * This function get enabled LPUART interrupts, the enabled interrupts are returned
 * as the logical OR value of the enumerators @ref _lpuart_interrupt_enable. To check
 * specific interrupts enable status, compare the return value with enumerators
 * in @ref _lpuart_interrupt_enable.
 * For example, to check whether TX empty interrupt is enabled:
 * @code
 *     uint32_t enabledInterrupts = LPUART_GetEnabledInterrupts(LPUART1);
 *
 *     if (kLPUART_TxDataRegEmptyInterruptEnable & enabledInterrupts)
 *     {
 *         ...
 *     }
 * @endcode
 *
 * @param base LPUART peripheral base address.
 * @return LPUART interrupt flags which are logical OR of the enumerators in @ref _lpuart_interrupt_enable.
 */
uint32_t LPUART_GetEnabledInterrupts(LPUART_Type *base);

#if defined(FSL_FEATURE_LPUART_HAS_DMA_ENABLE) && FSL_FEATURE_LPUART_HAS_DMA_ENABLE
/*!
 * @brief Get LPUART data register address
 *
 * This function return the LPUART data register address, which is mainly used by DMA/eDMA case
 *
 * @param base LPUART peripheral base address.
 * @return LPUART data register address which are used both by transmitter and receiver.
 */
static inline uint32_t LPUART_GetDataRegisterAddress(LPUART_Type *base)
{
    return (uint32_t) & (base->DATA);
}

/*!
 * @brief Enable or disable LPUART transmiter DMA request.
 *
 * This function enables or disables the transmit data register empty flag, STAT[TDRE], to generate DMA requests.
 *
 * @param base LPUART peripheral base address.
 * @param enable True to enable, false to disable.
 */
static inline void LPUART_EnableTxDMA(LPUART_Type *base, bool enable)
{
    if (enable)
    {
        base->BAUD |= LPUART_BAUD_TDMAE_MASK;
        base->CTRL |= LPUART_CTRL_TIE_MASK;
    }
    else
    {
        base->BAUD &= ~LPUART_BAUD_TDMAE_MASK;
        base->CTRL &= ~LPUART_CTRL_TIE_MASK;
    }
}

/*!
 * @brief Enable or disable LPUART receiver DMA
 *
 * This function enables or disables the receiver data register full flag, STAT[RDRF], to generate DMA requests.
 *
 * @param base LPUART peripheral base address.
 * @param enable True to enable, false to disable.
 */
static inline void LPUART_EnableRxDMA(LPUART_Type *base, bool enable)
{
    if (enable)
    {
        base->BAUD |= LPUART_BAUD_RDMAE_MASK;
        base->CTRL |= LPUART_CTRL_RIE_MASK;
    }
    else
    {
        base->BAUD &= ~LPUART_BAUD_RDMAE_MASK;
        base->CTRL &= ~LPUART_CTRL_RIE_MASK;
    }
}

/* @} */
#endif /* FSL_FEATURE_LPUART_HAS_DMA_ENABLE */

/*!
 * @name Bus Operations
 * @{
 */

/*!
 * @brief Enable or disable LPUART TX.
 *
 * This function enable or disable the LPUART ThisX.
 *
 * @param base LPUART peripheral base address.
 * @param enable True to enable, false to disable.
 */
static inline void LPUART_EnableTx(LPUART_Type *base, bool enable)
{
    if (enable)
    {
        base->CTRL |= LPUART_CTRL_TE_MASK;
    }
    else
    {
        base->CTRL &= ~LPUART_CTRL_TE_MASK;
    }
}

/*!
 * @brief Enable or disable LPUART RX.
 *
 * This function enables or disables the LPUART receiver.
 *
 * @param base LPUART peripheral base address.
 * @param enable True to enable, false to disable.
 */
static inline void LPUART_EnableRx(LPUART_Type *base, bool enable)
{
    if (enable)
    {
        base->CTRL |= LPUART_CTRL_RE_MASK;
    }
    else
    {
        base->CTRL &= ~LPUART_CTRL_RE_MASK;
    }
}

/*!
 * @brief Write to TX register using blocking method.
 *
 * This function polls the TX register, waits for the TX register empty or TX FIFO
 * has empty room then writes data to the TX buffer.
 *
 * @note This function does not check whether all the data has been sent out to bus,
 * so before disable TX, check kLPUART_TransmissionCompleteFlag to ensure the TX is
 * finished.
 *
 * @param base LPUART peripheral base address.
 * @param data Start addresss of the data to write.
 * @param length Size of the data to write.
 */
void LPUART_WriteBlocking(LPUART_Type *base, const uint8_t *data, size_t length);

/*!
 * @brief Write to TX register.
 *
 * This function writes data to the TX register directly, upper layer must make
 * sure the TX register is empty or TX FIFO has empty room before calling this function.
 *
 * @param base LPUART peripheral base address.
 * @param data Data write to the TX register.
 */
static inline void LPUART_WriteByte(LPUART_Type *base, uint8_t data)
{
    base->DATA = data;
}

/*!
* @brief Read RX data register using blocking method.
 *
 * This function polls the RX register, waits for the RX register full or RX FIFO
 * has data then read data from TX register.
 *
 * @param base LPUART peripheral base address.
 * @param data Start addresss of the buffer to store the received data.
 * @param length Size of the buffer.
 */
void LPUART_ReadBlocking(LPUART_Type *base, uint8_t *data, size_t length);

/*!
 * @brief Read RX register.
 *
 * This function reads data from the TX register directly, upper layer must make
 * sure the RX register is full or TX FIFO has data before calling this function.
 *
 * @param base LPUART peripheral base address.
 * @return Data read from data register.
 */
static inline uint8_t LPUART_ReadByte(LPUART_Type *base)
{
    return base->DATA;
}

/* @} */

/*!
 * @name Transactional
 * @{
 */

/*!
 * @brief Initialize the LPUART handle.
 *
 * This function initializes the LPUART handle which can be used for other LPUART
 * transactional APIs. Usually, for a specified LPUART instance, user only need
 * to call this API once to get the initialized handle.
 *
 * LPUART driver supports the "backaround" receiveing, which means that user could setup
 * a RX ring buffer optionally. Data received are stored into the ring buffer even that
 * user don't call the LPUART_ReceiveNonBlocking() API. If there are already data received
 * in the ring buffer, user can get the received data from the ring buffer directly.
 * The ring buffer is disabled if pass NULL as @p ringBuffer.
 *
 * @param base LPUART peripheral base address.
 * @param handle LPUART handle pointer.
 * @param ringBuffer Start address of ring buffer for backaround receiveing. Pass NULL to disable the ring buffer.
 * @param ringBufferSize size of the ringbuffer.
 */
void LPUART_CreateHandle(LPUART_Type *base,
                         lpuart_handle_t *handle,
                         lpuart_transfer_callback_t callback,
                         void *userData);

/*!
 * @brief Transmit a buffer of data using the interrupt method
 *
 * This function send data using interrupt method. This is non-blocking function,
 * returns directly without waiting for all data written to TX register. When
 * all data are written to TX register in ISR, LPUART driver calls the callback
 * function and pass @ref kStatus_LPUART_TxIdle as status parameter.
 *
 * @note The kStatus_LPUART_TxIdle is passed to upper layer when all data written
 * to TX register, but not ensure all the data sent out. So before disable TX,
 * check kLPUART_TransmissionCompleteFlag to ensure the TX is finished.
 *
 * @param base LPUART peripheral base address.
 * @param handle LPUART handle pointer.
 * @param xfer LPUART transfer sturcture, refer to #lpuart_transfer_t.
 * @retval kStatus_Success Sucessully start the data transmission.
 * @retval kStatus_LPUART_TxBusy Previous transmission still not finished, data not all written to TX register yet.
 */
status_t LPUART_SendNonBlocking(LPUART_Type *base, lpuart_handle_t *handle, lpuart_transfer_t *xfer);

/*!
 * @brief Set up the RX ring buffer.
 *
 * This function sets up the RX ring buffer to specific UART handle.
 *
 * When RX ring buffer is used, data received are stored into the ring buffer even that
 * user don't call the UART_ReceiveNonBlocking() API. If there are already data received
 * in the ring buffer, user can get the received data from the ring buffer directly.
 *
 * @note When using RX ring buffer, one byte is reserved for internal use. In other
 * words, if @p ringBufferSize is 32, then only 31 bytes are used for saving data.
 *
 * @param base LPUART peripheral base address.
 * @param handle LPUART handle pointer.
 * @param ringBuffer Start address of ring buffer for backaround receiveing. Pass NULL to disable the ring buffer.
 * @param ringBufferSize size of the ringbuffer.
 */
void LPUART_StartRingBuffer(LPUART_Type *base, lpuart_handle_t *handle, uint8_t *ringBuffer, size_t ringBufferSize);

/*!
 * @brief Abort the background transfer and uninstall ring buffer.
 *
 * This function abort the background transfer and uninstall the ringbuffer.
 *
 * @param base LPUART peripheral base address.
 * @param handle LPUART handle pointer.
 */
void LPUART_StopRingBuffer(LPUART_Type *base, lpuart_handle_t *handle);

/*!
 * @brief Abort interrupt driven data transmit.
 *
 * This function aborts interrupt driven data sending. User can get the remainBtyes to know
 * how many bytes still not sent out.
 *
 * @param base LPUART peripheral base address.
 * @param handle LPUART handle pointer.
 */
void LPUART_AbortSend(LPUART_Type *base, lpuart_handle_t *handle);

/*!
 * @brief Get the number of remaining bytes not send.
 *
 * This function gets the number of remaining bytes not send driven by interrupt.
 *
 * @param base LPUART peripheral base address.
 * @param handle LPUART handle pointer.
 * @return The number of bytes not send.
 */
static inline size_t LPUART_GetSendRemainingBytes(LPUART_Type *base, lpuart_handle_t *handle)
{
    return handle->txDataSize;
}

/*!
 * @brief Receive a buffer of data using the interrupt method.
 *
 * This function receives data using interrupt method. This is non-blocking function
 * and will return without necessarily wait all data are received.
 * If RX ring buffer is used and not empty, the data in ring buffer is copied and
 * the parameter @p receivedBytes shows how many bytes are copied from ring buffer.
 * After copy, if the data in ring buffer is not enough for read, the receive
 * request is saved by LPUART driver, when new data arrived, the receive request
 * is serviced first. When all data received, LPUART driver notifies upper layer
 * through callback function, pass status parameter @ref kStatus_UART_RxIdle.
 * For example, upper layer needs 10 bytes but there are only 5 bytes in ring buffer,
 * then the 5 bytes are copied to xfer->data, this function returns with the
 * parameter @p receivedBytes set to 5. For the left 5 bytes, new arrived data is
 * saved from xfer->data[5], when 5 bytes received, LPUART driver notifies upper layer.
 * If RX ring buffer is not enabled, this function enable RX and RX interrupt
 * to receive data to xfer->data. When all data received, upper layer is notified.
 *
 * @param base LPUART peripheral base address.
 * @param handle LPUART handle pointer.
 * @param xfer LPUART transfer sturcture, refer to #uart_transfer_t.
 * @param receivedBytes Bytes received from the ring buffer directly.
 * @retval kStatus_Success Sucessully queue the transfer into transmit queue.
 * @retval kStatus_LPUART_RxBusy Previous receive requst is not finished.
 */
status_t LPUART_ReceiveNonBlocking(LPUART_Type *base,
                                   lpuart_handle_t *handle,
                                   lpuart_transfer_t *xfer,
                                   size_t *receivedBytes);

/*!
 * @brief Abort interrupt driven data receiving.
 *
 * This function aborts interrupt driven data receiving. User can get the remainBytes to know
 * how many bytes not received yet.
 *
 * @param base LPUART peripheral base address.
 * @param handle LPUART handle pointer.
 */
void LPUART_AbortReceive(LPUART_Type *base, lpuart_handle_t *handle);

/*!
 * @brief Get the number of remaining bytes not received.
 *
 * This function gets the number of remaining bytes not received driven by interrupt.
 *
 * @param base LPUART peripheral base address.
 * @param handle LPUART handle pointer.
 * @return The number of bytes not received.
 */
static inline size_t LPUART_GetReceiveRemainingBytes(LPUART_Type *base, lpuart_handle_t *handle)
{
    return handle->rxDataSize;
}

/*!
 * @brief LPUART IRQ handle function
 *
 * This function handles the LPUART transmit and receive IRQ request.
 *
 * @param base LPUART peripheral base address.
 * @param handle LPUART handle pointer.
 */
void LPUART_HandleIRQ(LPUART_Type *base, lpuart_handle_t *handle);

/*!
 * @brief LPUART Error IRQ handle function
 *
 * This function handle the LPUART error IRQ request.
 *
 * @param base LPUART peripheral base address.
 * @param handle LPUART handle pointer.
 */
void LPUART_HandleErrorIRQ(LPUART_Type *base, lpuart_handle_t *handle);

/* @} */

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* _FSL_LPUART_H_ */
