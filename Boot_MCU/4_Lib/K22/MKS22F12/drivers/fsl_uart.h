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
#ifndef _FSL_UART_H_
#define _FSL_UART_H_

#include "fsl_common.h"

/*!
 * @addtogroup uart
 * @{
 */

/*! @file*/

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief UART driver version */
#define FSL_UART_DRIVER_VERSION (MAKE_VERSION(2, 0, 0)) /*!< Version 2.0.0. */

/*! @brief Error codes for the UART driver. */
enum _uart_status
{
    kStatus_UART_TxBusy = MAKE_STATUS(kStatusGroup_UART, 0),              /*!< Transmitter is busy. */
    kStatus_UART_RxBusy = MAKE_STATUS(kStatusGroup_UART, 1),              /*!< Receiver is busy. */
    kStatus_UART_TxIdle = MAKE_STATUS(kStatusGroup_UART, 2),              /*!< UART transmitter is idle. */
    kStatus_UART_RxIdle = MAKE_STATUS(kStatusGroup_UART, 3),              /*!< UART receiver is idle. */
    kStatus_UART_TxWatermarkTooLarge = MAKE_STATUS(kStatusGroup_UART, 4), /*!< TX fifo watermark too large  */
    kStatus_UART_RxWatermarkTooLarge = MAKE_STATUS(kStatusGroup_UART, 5), /*!< RX fifo watermark too large  */
    kStatus_UART_FlagCannotClearManually =
        MAKE_STATUS(kStatusGroup_UART, 6),                                /*!< UART flag can't be manually cleared. */
    kStatus_UART_Error = MAKE_STATUS(kStatusGroup_UART, 7),               /*!< Error happens on UART. */
    kStatus_UART_RxRingBufferOverrun = MAKE_STATUS(kStatusGroup_UART, 8), /*!< UART RX software ring buffer overrun. */
    kStatus_UART_RxHardwareOverrun = MAKE_STATUS(kStatusGroup_UART, 9)    /*!< UART RX receiver overrun. */
};

/*! @brief UART parity mode. */
typedef enum _uart_parity_mode
{
    kUART_ParityDisabled = 0x0U, /*!< Parity disabled */
    kUART_ParityEven = 0x2U,     /*!< Parity enabled, type even, bit setting: PE|PT = 10 */
    kUART_ParityOdd = 0x3U,      /*!< Parity enabled, type odd,  bit setting: PE|PT = 11 */
} uart_parity_mode_t;

/*! @brief UART stop bit count. */
typedef enum _uart_stop_bit_count
{
    kUART_OneStopBit = 0U, /*!< One stop bit */
    kUART_TwoStopBit = 1U, /*!< Two stop bits */
} uart_stop_bit_count_t;

/*!
 * @brief UART interrupt configuration structure, default settings all disabled.
 *
 * This structure contains the settings for all of the UART interrupt configurations.
 */
enum _uart_interrupt_enable
{
#if defined(FSL_FEATURE_UART_HAS_LIN_BREAK_DETECT) && FSL_FEATURE_UART_HAS_LIN_BREAK_DETECT
    kUART_LinBreakInterruptEnable = (UART_BDH_LBKDIE_MASK), /*!< LIN break detect interrupt. */
#endif
    kUART_RxActiveEdgeInterruptEnable = (UART_BDH_RXEDGIE_MASK),   /*!< RX active edge interrupt. */
    kUART_TxDataRegEmptyInterruptEnable = (UART_C2_TIE_MASK << 8), /*!< Transmit data register empty interrupt. */
    kUART_TransmissionCompleteInterruptEnable = (UART_C2_TCIE_MASK << 8), /*!< Transmission complete interrupt. */
    kUART_RxDataRegFullInterruptEnable = (UART_C2_RIE_MASK << 8),         /*!< Receiver data register full interrupt. */
    kUART_IdleLineInterruptEnable = (UART_C2_ILIE_MASK << 8),             /*!< Idle line interrupt. */
    kUART_RxOverrunInterruptEnable = (UART_C3_ORIE_MASK << 16),           /*!< Receiver overrun interrupt. */
    kUART_NoiseErrorInterruptEnable = (UART_C3_NEIE_MASK << 16),          /*!< Noise error flag interrupt. */
    kUART_FramingErrorInterruptEnable = (UART_C3_FEIE_MASK << 16),        /*!< Framing error flag interrupt. */
    kUART_ParityErrorInterruptEnable = (UART_C3_PEIE_MASK << 16),         /*!< Parity error flag interrupt. */
#if defined(FSL_FEATURE_UART_HAS_FIFO) && FSL_FEATURE_UART_HAS_FIFO
    kUART_RxFifoOverflowInterruptEnable = (UART_CFIFO_TXOFE_MASK << 24),  /*!< TX FIFO overflow interrupt. */
    kUART_TxFifoOverflowInterruptEnable = (UART_CFIFO_RXUFE_MASK << 24),  /*!< RX FIFO underflow interrupt. */
    kUART_RxFifoUnderflowInterruptEnable = (UART_CFIFO_RXUFE_MASK << 24), /*!< RX FIFO underflow interrupt. */
#endif
};

/*!
 * @brief UART status flags.
 *
 * This provides constants for the UART status flags for use in the UART functions.
 */
enum _uart_flags
{
    kUART_TxDataRegEmptyFlag = (UART_S1_TDRE_MASK),     /*!< TX data register empty flag. */
    kUART_TransmissionCompleteFlag = (UART_S1_TC_MASK), /*!< Transmission complete flag. */
    kUART_RxDataRegFullFlag = (UART_S1_RDRF_MASK),      /*!< RX data register full flag. */
    kUART_IdleLineFlag = (UART_S1_IDLE_MASK),           /*!< Idle line detect flag. */
    kUART_RxOverrunFlag = (UART_S1_OR_MASK),            /*!< RX overrun flag. */
    kUART_NoiseErrorFlag = (UART_S1_NF_MASK),           /*!< RX takes 3 samples of each received bit.
                                                             If any of these samples differ, noise flag sets */
    kUART_FramingErrorFlag = (UART_S1_FE_MASK),         /*!< Frame error flag, sets if logic 0 was detected
                                                             where stop bit expected */
    kUART_ParityErrorFlag = (UART_S1_PF_MASK),          /*!< If parity enabled, sets upon parity error detection */
#if defined(FSL_FEATURE_UART_HAS_LIN_BREAK_DETECT) && FSL_FEATURE_UART_HAS_LIN_BREAK_DETECT
    kUART_LinBreakFlag = (UART_S2_LBKDIF_MASK << 8), /*!< LIN break detect interrupt flag, sets when
                                                                   LIN break char detected and LIN circuit enabled */
#endif
    kUART_RxActiveEdgeFlag = (UART_S2_RXEDGIF_MASK << 8), /*!< RX pin active edge interrupt flag,
                                                                        sets when active edge detected */
    kUART_RxActiveFlag = (UART_S2_RAF_MASK << 8),         /*!< Receiver Active Flag (RAF),
                                                                        sets at beginning of valid start bit */
#if defined(FSL_FEATURE_UART_HAS_EXTENDED_DATA_REGISTER_FLAGS) && FSL_FEATURE_UART_HAS_EXTENDED_DATA_REGISTER_FLAGS
    kUART_NoiseErrorInRxDataRegFlag = (UART_ED_NOISY_MASK << 16),    /*!< Noisy bit, sets if noise detected. */
    kUART_ParityErrorInRxDataRegFlag = (UART_ED_PARITYE_MASK << 16), /*!< Paritye bit, sets if parity error detected. */
#endif
#if defined(FSL_FEATURE_UART_HAS_FIFO) && FSL_FEATURE_UART_HAS_FIFO
    kUART_TxFifoEmptyFlag = (UART_SFIFO_TXEMPT_MASK << 24),   /*!< TXEMPT bit, sets if TX buffer is empty */
    kUART_RxFifoEmptyFlag = (UART_SFIFO_RXEMPT_MASK << 24),   /*!< RXEMPT bit, sets if RX buffer is empty */
    kUART_TxFifoOverflowFlag = (UART_SFIFO_TXOF_MASK << 24),  /*!< TXOF bit, sets if TX buffer overflow occurred */
    kUART_RxFifoOverflowFlag = (UART_SFIFO_RXOF_MASK << 24),  /*!< RXOF bit, sets if receive buffer overflow */
    kUART_RxFifoUnderflowFlag = (UART_SFIFO_RXUF_MASK << 24), /*!< RXUF bit, sets if receive buffer underflow */
#endif
};

/*! @brief UART configure structure. */
typedef struct _uart_config
{
    uint32_t baudRate_Bps;         /*!< UART baud rate  */
    uart_parity_mode_t parityMode; /*!< Parity mode, disabled (default), even, odd */
#if defined(FSL_FEATURE_UART_HAS_STOP_BIT_CONFIG_SUPPORT) && FSL_FEATURE_UART_HAS_STOP_BIT_CONFIG_SUPPORT
    uart_stop_bit_count_t stopBitCount; /*!< Number of stop bits, 1 stop bit (default) or 2 stop bits  */
#endif
#if defined(FSL_FEATURE_UART_HAS_FIFO) && FSL_FEATURE_UART_HAS_FIFO
    uint8_t txFifoWatermark; /*!< TX FIFO watermark */
    uint8_t rxFifoWatermark; /*!< RX FIFO watermark */
#endif
    bool enableTx; /*!< Enable TX */
    bool enableRx; /*!< Enable RX */
} uart_config_t;

/*! @brief UART transfer structure. */
typedef struct _uart_transfer
{
    uint8_t *data;   /*!< The buffer of data to be transfer.*/
    size_t dataSize; /*!< The byte count to be transfer. */
} uart_transfer_t;

/* Forward declaration of the handle typedef. */
typedef struct _uart_handle uart_handle_t;

/*! @brief UART transfer callback function. */
typedef void (*uart_transfer_callback_t)(UART_Type *base, uart_handle_t *handle, status_t status, void *userData);

/*! @brief UART handle structure. */
struct _uart_handle
{
    uint8_t *volatile txData;   /*!< Address of remaining data to send. */
    volatile size_t txDataSize; /*!< Size of the remaining data to send. */
    uint8_t *volatile rxData;   /*!< Address of remaining data to receive. */
    volatile size_t rxDataSize; /*!< Size of the remaining data to receive. */

    uint8_t *rxRingBuffer;              /*!< Start address of the receiver ring buffer. */
    size_t rxRingBufferSize;            /*!< Size of the ring buffer. */
    volatile uint16_t rxRingBufferHead; /*!< Index for the driver to store received data into ring buffer. */
    volatile uint16_t rxRingBufferTail; /*!< Index for the user to get data from the ring buffer. */

    uart_transfer_callback_t callback; /*!< Callback function. */
    void *userData;                    /*!< Uart callback function parameter.*/

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
 * @brief Initialize an UART instance with user configuration structure and peripheral clock.
 *
 * This function configures the UART module with user-defined settings. User can configure the configuration
 * structure themselves and user can also get the defaul configuration by UART_GetDefaultConfig() function.
 * Example below show how to use this API to configure the UART.
 * @code
 *  uart_config_t uartConfig;
 *  uartConfig.baudRate_Bps = 115200U;
 *  uartConfig.parityMode = kUART_ParityDisabled;
 *  uartConfig.stopBitCount = kUART_OneStopBit;
 *  uartConfig.txFifoWatermark = 0;
 *  uartConfig.rxFifoWatermark = 1;
 *  UART_Init(UART1, &uartConfig, 20000000U);
 * @endcode
 *
 * @param base UART peripheral base address.
 * @param config Pointer to user-defined configuration structure.
 * @param srcClock_Hz UART clock source freqency in HZ.
 */
void UART_Init(UART_Type *base, const uart_config_t *config, uint32_t srcClock_Hz);

/*!
 * @brief Deinitializes an UART instance.
 *
 * This function waits for TX complete, disables TX and RX, then disables the UART clock.
 *
 * @param base UART peripheral base address.
 */
void UART_Deinit(UART_Type *base);

/*!
 * @brief Get the default configuration structure.
 *
 * This function initializes the UART configure structure to default value. The default
 * value are:
 *   uartConfig->baudRate_Bps = 115200U;
 *   uartConfig->bitCountPerChar = kUART_8BitsPerChar;
 *   uartConfig->parityMode = kUART_ParityDisabled;
 *   uartConfig->stopBitCount = kUART_OneStopBit;
 *   uartConfig->txFifoWatermark = 0;
 *   uartConfig->rxFifoWatermark = 1;
 *   uartConfig->enableTx = false;
 *   uartConfig->enableRx = false;
 *
 * @param config Pointer to configuration structure.
 */
void UART_GetDefaultConfig(uart_config_t *config);

/*!
 * @brief Set UART instance baudrate.
 *
 * This function configures the UART module baudrate. This function is used to update
 * the UART module baudrate after the UART module is initialized by the UART_Init.
 * @code
 *  UART_SetBaudRate(UART1, 115200U, 20000000U);
 * @endcode
 *
 * @param base UART peripheral base address.
 * @param baudRate_Bps UART baudrate to be set.
 * @param srcClock_Hz UART clock source freqency in HZ.
 */
void UART_SetBaudRate(UART_Type *base, uint32_t baudRate_Bps, uint32_t srcClock_Hz);

/* @} */

/*!
 * @name Status
 * @{
 */

/*!
 * @brief Get UART status flags.
 *
 * This function get all UART status flags, the flags are returned as the logical
 * OR value of the enumerators @ref _uart_flags. To check specific status,
 * compare the return value with enumerators in @ref _uart_flags.
 * For example, to check whether the TX is empty:
 * @code
 *     if (kUART_TxDataRegEmptyFlag & UART_GetStatusFlags(UART1))
 *     {
 *         ...
 *     }
 * @endcode
 *
 * @param base UART peripheral base address.
 * @return UART status flags which are ORed by the enumerators in the _uart_flags.
 */
uint32_t UART_GetStatusFlags(UART_Type *base);

/*!
 * @brief Clear status flags with provide mask.
 *
 * This function clears UART status flags with provided mask. Automatically cleared flag
 * can't be cleared by this function.
 * Some flags can only be cleared or set by the hardware itself, these flags are:
 *    kUART_TxDataRegEmptyFlag, kUART_TransmissionCompleteFlag, kUART_RxDataRegFullFlag,
 *    kUART_RxActiveFlag, kUART_NoiseErrorInRxDataRegFlag, kUART_ParityErrorInRxDataRegFlag,
 *    kUART_TxFifoEmptyFlag,kUART_RxFifoEmptyFlag
 *
 * @param base UART peripheral base address.
 * @param mask The status flags to be cleared, it is logical OR value of @ref _uart_flags.
 * @retval kStatus_UART_FlagCannotClearManually The flag can't be cleared by this function but
 *         it is cleared automatically by hardware.
 * @retval kStatus_Success Status in the mask are cleared.
 */
status_t UART_ClearStatusFlags(UART_Type *base, uint32_t mask);

/* @} */

/*!
 * @name Interrupts
 * @{
 */

/*!
 * @brief Enable UART interrupts according to provided mask.
 *
 * This function enables the UART interrupts according to provided mask. The mask
 * is a logical OR of enumeration members, see @ref _uart_interrupt_enable.
 * For example, to enable TX empty interrupt and RX full interrupt, do like this.
 * @code
 *     UART_EnableInterrupts(UART1,kUART_TxDataRegEmptyInterruptEnable | kUART_RxDataRegFullInterruptEnable);
 * @endcode
 *
 * @param base UART peripheral base address.
 * @param mask The interrupts to enable. Logical OR of @ref _uart_interrupt_enable.
 */
void UART_EnableInterrupts(UART_Type *base, uint32_t mask);

/*!
 * @brief Disable UART interrupts according to provided mask.
 *
 * This function disables the UART interrupts according to provided mask. The mask
 * is a logical OR of enumeration members, see @ref _uart_interrupt_enable.
 * For example, to disable TX empty interrupt and RX full interrupt, do like this.
 * @code
 *     UART_DisableInterrupts(UART1,kUART_TxDataRegEmptyInterruptEnable | kUART_RxDataRegFullInterruptEnable);
 * @endcode
 *
 * @param base UART peripheral base address.
 * @param mask The interrupts to disable. Logical OR of @ref _uart_interrupt_enable.
 */
void UART_DisableInterrupts(UART_Type *base, uint32_t mask);

/*!
 * @brief Get enabled UART interrupts.
 *
 * This function get enabled UART interrupts, the enabled interrupts are returned
 * as the logical OR value of the enumerators @ref _uart_interrupt_enable. To check
 * specific interrupts enable status, compare the return value with enumerators
 * in @ref _uart_interrupt_enable.
 * For example, to check whether TX empty interrupt is enabled:
 * @code
 *     uint32_t enabledInterrupts = UART_GetEnabledInterrupts(UART1);
 *
 *     if (kUART_TxDataRegEmptyInterruptEnable & enabledInterrupts)
 *     {
 *         ...
 *     }
 * @endcode
 *
 * @param base UART peripheral base address.
 * @return UART interrupt flags which are logical OR of the enumerators in @ref _uart_interrupt_enable.
 */
uint32_t UART_GetEnabledInterrupts(UART_Type *base);

/* @} */

#if defined(FSL_FEATURE_UART_HAS_DMA_SELECT) && FSL_FEATURE_UART_HAS_DMA_SELECT
/*!
 * @name DMA Control
 * @{
 */

/*!
 * @brief Get UART data register address.
 *
 * This function return the UART data register address, which is mainly used by DMA/eDMA case.
 *
 * @param base UART peripheral base address.
 * @return UART data register address which are used both by transmitter and receiver.
 */
static inline uint32_t UART_GetDataRegisterAddress(UART_Type *base)
{
    return (uint32_t) & (base->D);
}

/*!
 * @brief Enable or disable UART transmiter DMA request.
 *
 * This function enables or disables the transmit data register empty flag, S1[TDRE], to generate DMA requests.
 *
 * @param base UART peripheral base address.
 * @param enable True to enable, false to disable.
 */
static inline void UART_EnableTxDMA(UART_Type *base, bool enable)
{
    if (enable)
    {
#if (defined(FSL_FEATURE_UART_IS_SCI) && FSL_FEATURE_UART_IS_SCI)
        base->C4 |= UART_C4_TDMAS_MASK;
#else
        base->C5 |= UART_C5_TDMAS_MASK;
#endif
        base->C2 |= UART_C2_TIE_MASK;
    }
    else
    {
#if (defined(FSL_FEATURE_UART_IS_SCI) && FSL_FEATURE_UART_IS_SCI)
        base->C4 &= ~UART_C4_TDMAS_MASK;
#else
        base->C5 &= ~UART_C5_TDMAS_MASK;
#endif
        base->C2 &= ~UART_C2_TIE_MASK;
    }
}

/*!
 * @brief Enable or disable UART receiver DMA
 *
 * This function enables or disables the receiver data register full flag, S1[RDRF], to generate DMA requests.
 *
 * @param base UART peripheral base address.
 * @param enable True to enable, false to disable.
 */
static inline void UART_EnableRxDMA(UART_Type *base, bool enable)
{
    if (enable)
    {
#if (defined(FSL_FEATURE_UART_IS_SCI) && FSL_FEATURE_UART_IS_SCI)
        base->C4 |= UART_C4_RDMAS_MASK;
#else
        base->C5 |= UART_C5_RDMAS_MASK;
#endif
        base->C2 |= UART_C2_RIE_MASK;
    }
    else
    {
#if (defined(FSL_FEATURE_UART_IS_SCI) && FSL_FEATURE_UART_IS_SCI)
        base->C4 &= ~UART_C4_RDMAS_MASK;
#else
        base->C5 &= ~UART_C5_RDMAS_MASK;
#endif
        base->C2 &= ~UART_C2_RIE_MASK;
    }
}

/* @} */
#endif /* FSL_FEATURE_UART_HAS_DMA_SELECT */

/*!
 * @name Bus Operations
 * @{
 */

/*!
 * @brief Enable or disable UART TX.
 *
 * This function enables or disables the UART transmitter.
 *
 * @param base UART peripheral base address.
 * @param enable True to enable, false to disable.
 */
static inline void UART_EnableTx(UART_Type *base, bool enable)
{
    if (enable)
    {
        base->C2 |= UART_C2_TE_MASK;
    }
    else
    {
        base->C2 &= ~UART_C2_TE_MASK;
    }
}

/*!
 * @brief Enable or disable UART RX.
 *
 * This function enables or disables the UART receiver.
 *
 * @param base UART peripheral base address.
 * @param enable True to enable, false to disable.
 */
static inline void UART_EnableRx(UART_Type *base, bool enable)
{
    if (enable)
    {
        base->C2 |= UART_C2_RE_MASK;
    }
    else
    {
        base->C2 &= ~UART_C2_RE_MASK;
    }
}

/*!
 * @brief Write to TX register using blocking method.
 *
 * This function polls the TX register, waits for the TX register empty or TX FIFO
 * has empty room then writes data to the TX buffer.
 *
 * @note This function does not check whether all the data has been sent out to bus,
 * so before disable TX, check kUART_TransmissionCompleteFlag to ensure the TX is
 * finished.
 *
 * @param base UART peripheral base address.
 * @param data Start addresss of the data to write.
 * @param length Size of the data to write.
 */
void UART_WriteBlocking(UART_Type *base, const uint8_t *data, size_t length);

/*!
 * @brief Write to TX register.
 *
 * This function writes data to the TX register directly, upper layer must make
 * sure the TX register is empty or TX FIFO has empty room before calling this function.
 *
 * @param base UART peripheral base address.
 * @param data The byte to write.
 */
static inline void UART_WriteByte(UART_Type *base, uint8_t data)
{
    base->D = data;
}

/*!
 * @brief Read RX data register using blocking method.
 *
 * This function polls the RX register, waits for the RX register full or RX FIFO
 * has data then read data from TX register.
 *
 * @param base UART peripheral base address.
 * @param data Start addresss of the buffer to store the received data.
 * @param length Size of the buffer.
 */
void UART_ReadBlocking(UART_Type *base, uint8_t *data, size_t length);

/*!
 * @brief Read RX register directly.
 *
 * This function reads data from the TX register directly, upper layer must make
 * sure the RX register is full or TX FIFO has data before calling this function.
 *
 * @param base UART peripheral base address.
 * @return The byte read from UART data register.
 */
static inline uint8_t UART_ReadByte(UART_Type *base)
{
    return base->D;
}

/* @} */

/*!
 * @name Transactional
 * @{
 */

/*!
 * @brief Initialize the UART Handle.
 *
 * This function initializes the UART handle which can be used for other UART
 * transactional APIs. Usually, for a specified UART instance, user only need
 * to call this API once to get the initialized handle.
 *
 * @param base UART peripheral base address.
 * @param handle UART handle pointer.
 * @param callback The callback function.
 * @param userData The parameter of the callback function.
 */
void UART_CreateHandle(UART_Type *base, uart_handle_t *handle, uart_transfer_callback_t callback, void *userData);

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
 * @param base UART peripheral base address.
 * @param handle UART handle pointer.
 * @param ringBuffer Start address of ring buffer for backaround receiveing. Pass NULL to disable the ring buffer.
 * @param ringBufferSize size of the ringbuffer.
 */
void UART_StartRingBuffer(UART_Type *base, uart_handle_t *handle, uint8_t *ringBuffer, size_t ringBufferSize);

/*!
 * @brief Abort the background transfer and uninstall ring buffer.
 *
 * This function abort the background transfer and uninstall the ringbuffer.
 *
 * @param base UART peripheral base address.
 * @param handle UART handle pointer.
 */
void UART_StopRingBuffer(UART_Type *base, uart_handle_t *handle);

/*!
 * @brief Transmit a buffer of data using the interrupt method
 *
 * This function send data using interrupt method. This is non-blocking function,
 * returns directly without waiting for all data written to TX register. When
 * all data are written to TX register in ISR, UART driver calls the callback
 * function and pass @ref kStatus_UART_TxIdle as status parameter.
 *
 * @note The kStatus_UART_TxIdle is passed to upper layer when all data written
 * to TX register, but not ensure all the data sent out. So before disable TX,
 * check kUART_TransmissionCompleteFlag to ensure the TX is finished.
 *
 * @param base UART peripheral base address.
 * @param handle UART handle pointer.
 * @param xfer UART transfer sturcture, refer to #uart_transfer_t.
 * @retval kStatus_Success Sucessully start the data transmission.
 * @retval kStatus_UART_TxBusy Previous transmission still not finished, data not all written to TX register yet.
 */
status_t UART_SendNonBlocking(UART_Type *base, uart_handle_t *handle, uart_transfer_t *xfer);

/*!
 * @brief Abort interrupt driven data transmit.
 *
 * This function aborts interrupt driven data sending. User can get the remainBytes to know
 * how many bytes still not sent out.
 *
 * @param base UART peripheral base address.
 * @param handle UART handle pointer.
 */
void UART_AbortSend(UART_Type *base, uart_handle_t *handle);

/*!
 * @brief Get the number of remaining bytes not send.
 *
 * This function gets the number of remaining bytes not send driven by interrupt.
 *
 * @param base UART peripheral base address.
 * @param handle UART handle pointer.
 * @return The number of bytes not send.
 */
static inline size_t UART_GetSendRemainingBytes(UART_Type *base, uart_handle_t *handle)
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
 * request is saved by UART driver, when new data arrived, the receive request
 * is serviced first. When all data received, UART driver notifies upper layer
 * through callback function, pass status parameter @ref kStatus_UART_RxIdle.
 * For example, upper layer needs 10 bytes but there are only 5 bytes in ring buffer,
 * then the 5 bytes are copied to xfer->data, this function returns with the
 * parameter @p receivedBytes set to 5. For the left 5 bytes, new arrived data is
 * saved from xfer->data[5], when 5 bytes received, UART driver notifies upper layer.
 * If RX ring buffer is not enabled, this function enable RX and RX interrupt
 * to receive data to xfer->data. When all data received, upper layer is notified.
 *
 * @param base UART peripheral base address.
 * @param handle UART handle pointer.
 * @param xfer UART transfer sturcture, refer to #uart_transfer_t.
 * @param receivedBytes Bytes received from the ring buffer directly.
 * @retval kStatus_Success Sucessully queue the transfer into transmit queue.
 * @retval kStatus_UART_RxBusy Previous receive requst is not finished.
 */
status_t UART_ReceiveNonBlocking(UART_Type *base, uart_handle_t *handle, uart_transfer_t *xfer, size_t *receivedBytes);

/*!
 * @brief Abort interrupt driven data receiving.
 *
 * This function aborts interrupt driven data receiving. User can get the remainBytes to know
 * how many bytes not received yet.
 *
 * @param base UART peripheral base address.
 * @param handle UART handle pointer.
 */
void UART_AbortReceive(UART_Type *base, uart_handle_t *handle);

/*!
 * @brief Get the number of remaining bytes not received.
 *
 * This function gets the number of remaining bytes not received driven by interrupt.
 *
 * @param base UART peripheral base address.
 * @param handle UART handle pointer.
 * @return The number of bytes not received.
 */
static inline size_t UART_GetReceiveRemainingBytes(UART_Type *base, uart_handle_t *handle)
{
    return handle->rxDataSize;
}

/*!
 * @brief UART IRQ handle function
 *
 * This function handles the UART transmit and receive IRQ request.
 *
 * @param base UART peripheral base address.
 * @param handle UART handle pointer.
 */
void UART_HandleIRQ(UART_Type *base, uart_handle_t *handle);

/*!
 * @brief UART Error IRQ handle function
 *
 * This function handle the UART error IRQ request.
 *
 * @param base UART peripheral base address.
 * @param handle UART handle pointer.
 */
void UART_HandleErrorIRQ(UART_Type *base, uart_handle_t *handle);

/* @} */

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* _FSL_UART_H_ */
