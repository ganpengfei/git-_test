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

#ifndef _FSL_FLEXIO_UART_H_
#define _FSL_FLEXIO_UART_H_

#include "fsl_common.h"
#include "fsl_flexio.h"

/*!
 * @addtogroup flexio_uart
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief FLEXIO UART driver version */
#define FSL_FLEXIO_UART_DRIVER_VERSION (MAKE_VERSION(2, 0, 0)) /*!< Version 2.0.0. */

/*! @brief Error codes for the UART driver. */
enum _flexio_uart_status
{
    kStatus_FLEXIO_UART_TxBusy = MAKE_STATUS(kStatusGroup_FLEXIO_UART, 0), /*!< Transmitter is busy. */
    kStatus_FLEXIO_UART_RxBusy = MAKE_STATUS(kStatusGroup_FLEXIO_UART, 1), /*!< Receiver is busy. */
    kStatus_FLEXIO_UART_TxIdle = MAKE_STATUS(kStatusGroup_FLEXIO_UART, 2), /*!< UART transmitter is idle. */
    kStatus_FLEXIO_UART_RxIdle = MAKE_STATUS(kStatusGroup_FLEXIO_UART, 3), /*!< UART receiver is idle. */
    kStatus_FLEXIO_UART_ERROR = MAKE_STATUS(kStatusGroup_FLEXIO_UART, 4),  /*!< ERROR happens on UART. */
    kStatus_FLEXIO_UART_RxRingBufferOverrun =
        MAKE_STATUS(kStatusGroup_FLEXIO_UART, 5), /*!< UART RX software ring buffer overrun. */
    kStatus_FLEXIO_UART_RxHardwareOverrun = MAKE_STATUS(kStatusGroup_FLEXIO_UART, 6) /*!< UART RX receiver overrun. */
};

/*! @brief FLEXIO UART bit count per char. */
typedef enum _flexio_uart_bit_count_per_char
{
    kFLEXIO_UART_7BitsPerChar = 7U, /*!< 7-bit data characters */
    kFLEXIO_UART_8BitsPerChar = 8U, /*!< 8-bit data characters */
    kFLEXIO_UART_9BitsPerChar = 9U, /*!< 9-bit data characters */
} flexio_uart_bit_count_per_char_t;

/*! @brief Define FLEXIO UART interrupt mask. */
enum _flexio_uart_interrupt_enable
{
    kFLEXIO_UART_TxDataRegEmptyInterruptEnable = 0x1U, /*!< Transmit buffer empty interrupt enable. */
    kFLEXIO_UART_RxDataRegFullInterruptEnable = 0x2U,  /*!< Receive buffer full interrupt enable. */
};

/*! @brief Define FLEXIO UART status mask. */
enum _flexio_uart_status_flags
{
    kFLEXIO_UART_TxDataRegEmptyFlag = 0x1U, /*!< Transmit buffer empty flag. */
    kFLEXIO_UART_RxDataRegFullFlag = 0x2U,  /*!< Receive buffer full flag. */
    kFLEXIO_UART_RxOverRunFlag = 0x4U,      /*!< Receive buffer over run flag. */
};

/*! @brief Define FLEXIO UART access structure typedef. */
typedef struct _flexio_uart_type
{
    FLEXIO_Type *flexioBase; /*!< FLEXIO base pointer. */
    uint8_t TxPinIndex;      /*!< Pin select for UART_Tx. */
    uint8_t RxPinIndex;      /*!< Pin select for UART_Rx. */
    uint8_t shifterIndex[2]; /*!< Shifter index used in FLEXIO UART. */
    uint8_t timerIndex[2];   /*!< Timer index used in FLEXIO UART. */
} FLEXIO_UART_Type;

/*! @brief Define FLEXIO UART user configuration structure. */
typedef struct _flexio_uart_config
{
    bool enableUart;                                  /*!< Enable/disable FLEXIO UART TX & RX. */
    bool enableInDoze;                                /*!< Enable/disable FLEXIO operation in doze mode*/
    bool enableInDebug;                               /*!< Enable/disable FLEXIO operation in debug mode*/
    bool enableFastAccess;                            /*!< Enable/disable fast access to FLEXIO registers,
                                                       fast access requires the FLEXIO clock to be at least
                                                       twice the frequency of the bus clock. */
    uint32_t baudRate_Bps;                            /*!< Baud rate in Bps. */
    flexio_uart_bit_count_per_char_t bitCountPerChar; /*!< number of bits, 7/8/9 -bit */
} flexio_uart_config_t;

/*! @brief Define FLEXIO UART transfer structure. */
typedef struct _flexio_uart_transfer
{
    uint8_t *data;   /*!< Transfer buffer*/
    size_t dataSize; /*!< Transfer size*/
} flexio_uart_transfer_t;

/* Forward declaration of the handle typedef. */
typedef struct _flexio_uart_handle flexio_uart_handle_t;

/*! @brief FLEXIO UART transfer callback function. */
typedef void (*flexio_uart_transfer_callback_t)(FLEXIO_UART_Type *base,
                                                flexio_uart_handle_t *handle,
                                                status_t status,
                                                void *userData);

/*! @brief Define FLEXIO UART handle structure*/
struct _flexio_uart_handle
{
    uint8_t *volatile txData;   /*!< Address of remaining data to send. */
    volatile size_t txDataSize; /*!< Size of the remaining data to send. */
    uint8_t *volatile rxData;   /*!< Address of remaining data to receive. */
    volatile size_t rxDataSize; /*!< Size of the remaining data to receive. */

    uint8_t *rxRingBuffer;              /*!< Start address of the receiver ring buffer. */
    size_t rxRingBufferSize;            /*!< Size of the ring buffer. */
    volatile uint16_t rxRingBufferHead; /*!< Index for the driver to store received data into ring buffer. */
    volatile uint16_t rxRingBufferTail; /*!< Index for the user to get data from the ring buffer. */

    flexio_uart_transfer_callback_t callback; /*!< Callback function. */
    void *userData;                           /*!< Uart callback function parameter.*/

    volatile uint8_t txState; /*!< TX transfer state. */
    volatile uint8_t rxState; /*!< RX transfer state */
};

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /*_cplusplus*/

/*!
 * @name Initialization and deinitialization
 * @{
 */

/*!
 * @brief Ungate the FLEXIO clock, reset the FLEXIO module and do FLEXIO UART
 * hardware configuration.Configure the FLEXIO UART with flexio uart configuration.
 * The configuration structure can be filled by user from scratch, or be set with
 * default values by FLEXIO_UART_GetDefaultConfig().
 *
 * Example
   @code
   FLEXIO_UART_Type base = {
   .flexioBase = FLEXIO,
   .TxPinIndex = 0,
   .RxPinIndex = 1,
   .shifterIndex = {0,1},
   .timerIndex = {0,1}
   };
   flexio_uart_config_t config = {
   .enableInDoze = false,
   .enableInDebug = true,
   .enableFastAccess = false,
   .baudRate_Bps = 115200U,
   .bitCountPerChar = 8
   };
   FLEXIO_UART_Init(base, &config, srcClock_Hz);
   @endcode
 *
 * @param base pointer to FLEXIO_UART_Type structure
 * @param userConfig pointer to flexio_uart_config_t structure
 * @param srcClock_Hz flexio source clock in HZ
*/
void FLEXIO_UART_Init(FLEXIO_UART_Type *base, const flexio_uart_config_t *userConfig, uint32_t srcClock_Hz);

/*!
 * @brief Disable FLEXIO UART and gate the FLEXIO clock.
 *
 * @note After calling this API, user need to call FLEXO_UART_Init to use the FLEXIO UART module.
 *
 * @param base pointer to FLEXIO_UART_Type structure
*/
void FLEXIO_UART_Deinit(FLEXIO_UART_Type *base);

/*!
 * @brief Get the default configuration to configure FLEXIO UART. The configuration
 * could be used directly for calling FLEXIO_UART_Init().
 * Example:
   @code
   flexio_uart_config_t config;
   FLEXIO_UART_GetDefaultConfig(&userConfig);
   @endcode
 * @param userConfig pointer to flexio_uart_config_t structure
*/
void FLEXIO_UART_GetDefaultConfig(flexio_uart_config_t *userConfig);

/* @} */

/*!
 * @name Status
 * @{
 */

/*!
 * @brief Get FLEXIO UART status flags.
 *
 * @param base pointer to FLEXIO_UART_Type structure
 * @param statusFlag status flag
*/

uint32_t FLEXIO_UART_GetStatusFlags(FLEXIO_UART_Type *base);

/*!
 * @brief Get FLEXIO UART status flags.
 *
 * @param base pointer to FLEXIO_UART_Type structure
 * @param mask status flag
 *      The parameter could be any combination of the following values:
 *          @arg kFLEXIO_UART_TxDataRegEmptyFlag
 *          @arg kFLEXIO_UART_RxEmptyFlag
 *          @arg kFLEXIO_UART_RxOverRunFlag
*/

void FLEXIO_UART_ClearStatusFlags(FLEXIO_UART_Type *base, uint32_t mask);

/* @} */

/*!
 * @name Interrupts
 * @{
 */

/*!
 * @brief Enable flexio uart interrupt.
 *
 * This function enable the flexio uart interrupt
 *
 * @param base pointer to FLEXIO_UART_Type structure
 * @param mask interrupt source
 */
void FLEXIO_UART_EnableInterrupts(FLEXIO_UART_Type *base, uint32_t mask);

/*!
 * @brief Disable flexio uart interrupt.
 *
 * This function disable the flexio uart interrupt
 *
 * @param base pointer to FLEXIO_UART_Type structure
 * @param mask interrupt source
 */
void FLEXIO_UART_DisableInterrupts(FLEXIO_UART_Type *base, uint32_t mask);

/* @} */

/*!
 * @name DMA Control
 * @{
 */

/*!
 * @brief get flexio uart transmit data register address
 *
 * This function return the uart data register address, mainly used by DMA/eDMA
 *
 * @param base pointer to FLEXIO_UART_Type structure
 * @return flexio uart transmit data register address.
 */
static inline uint32_t FLEXIO_UART_GetTxDataRegisterAddress(FLEXIO_UART_Type *base)
{
    return FLEXIO_GetShifterBufferAddress(base->flexioBase, kFLEXIO_ShifterBuffer, base->shifterIndex[0]);
}

/*!
 * @brief get flexio uart receive data register address
 *
 * This function return the uart data register address, mainly used by DMA/eDMA
 *
 * @param base pointer to FLEXIO_UART_Type structure
 * @return flexio uart receive data register address.
 */
static inline uint32_t FLEXIO_UART_GetRxDataRegisterAddress(FLEXIO_UART_Type *base)
{
    return FLEXIO_GetShifterBufferAddress(base->flexioBase, kFLEXIO_ShifterBufferByteSwapped, base->shifterIndex[1]);
}

/*!
 * @brief Enable/Disable flexio uart transmit DMA. This function enables/disables the flexio uart Tx DMA,
 * which means assert kFLEXIO_UART_TxDataRegEmptyFlag will/won't trigger DMA request.
 *
 * @param base pointer to FLEXIO_UART_Type structure
 * @param enable True to enable, false to disable.
 */
static inline void FLEXIO_UART_EnableTxDMA(FLEXIO_UART_Type *base, bool enable)
{
    FLEXIO_EnableShifterStatusDMA(base->flexioBase, 1 << base->shifterIndex[0], enable);
}

/*!
 * @brief Enable/Dsiable flexio uart receive DMA. This function enables/disables the flexio uart Rx DMA,
 * which means assert kFLEXIO_UART_RxDataRegFullFlag will/won't trigger DMA request.
 *
 * @param base pointer to FLEXIO_UART_Type structure
 * @param enable True to enable, false to disable.
 */
static inline void FLEXIO_UART_EnableRxDMA(FLEXIO_UART_Type *base, bool enable)
{
    FLEXIO_EnableShifterStatusDMA(base->flexioBase, 1 << base->shifterIndex[1], enable);
}

/* @} */

/*!
 * @name Bus Operations
 * @{
 */

/*!
 * @brief Enable/Disable the FLEXIO UART module operation.
 *
 * @param base pointer to FLEXIO_UART_Type
 * @param enable True to enable, false to disable.
*/
static inline void FLEXIO_UART_Enable(FLEXIO_UART_Type *base, bool enable)
{
    if (enable)
    {
        base->flexioBase->CTRL |= FLEXIO_CTRL_FLEXEN_MASK;
    }
    else
    {
        base->flexioBase->CTRL &= ~FLEXIO_CTRL_FLEXEN_MASK;
    }
}

/*!
 * @brief Writes one byte of data.
 *
 * @note This is a non-blocking API and will return directly after the data is put into the
 * data register. User needs to make sure that the TxEmptyFlag is asserted before calling
 * this API.
 *
 * @param base pointer to FLEXIO_UART_Type structure
 * @param buffer The data bytes to send
 */
static inline void FLEXIO_UART_WriteByte(FLEXIO_UART_Type *base, const uint8_t *buffer)
{
    base->flexioBase->SHIFTBUF[base->shifterIndex[0]] = *buffer;
}

/*!
 * @brief Reads one byte of data.
 *
 * @note This is a non-blocking API and will return directly after the data is read from the
 * data register. User needs to make sure the RxFullFlag is asserted before calling this API.
 *
 * @param base pointer to FLEXIO_UART_Type structure
 * @param buffer The buffer to store the received bytes
 */
static inline void FLEXIO_UART_ReadByte(FLEXIO_UART_Type *base, uint8_t *buffer)
{
    *buffer = base->flexioBase->SHIFTBUFBYS[base->shifterIndex[1]];
}

/*!
 * @brief sends a buffer of data bytes.
 *
 * @note This function blocks via polling until all bytes have been sent.
 *
 * @param base pointer to FLEXIO_UART_Type structure
 * @param buffer The data bytes to send
 * @param size The number of data bytes to send
 * @return transfer status, succeed return kStatus_Success
 */
void FLEXIO_UART_WriteBlocking(FLEXIO_UART_Type *base, const uint8_t *txData, size_t txSize);

/*!
 * @brief Receives a buffer of bytes.
 *
 * @note This function blocks via polling until all bytes have been received.
 *
 * @param base pointer to FLEXIO_UART_Type structure
 * @param buffer The buffer to store the received bytes
 * @param size The number of data bytes to be received
 */
void FLEXIO_UART_ReadBlocking(FLEXIO_UART_Type *base, uint8_t *rxData, size_t rxSize);

/* @} */

/*!
 * @name Transactional
 * @{
 */

/*!
 * @brief Initialize the UART handle.
 *
 * This function initializes the FLEXIO UART handle which can be used for other FLEXIO
 * UART transactional APIs. Usually, user only need to call this API once to get the
 * initialized handle.
 *
 * UART driver supports the "backaround" receiveing, which means that user could setup
 * a RX ring buffer optionally. Data received are stored into the ring buffer even that
 * user don't call the FLEXIO_UART_ReceiveNonBlocking() API. If there are already data
 * received in the ring buffer, user can get the received data from the ring buffer
 * directly. The ring buffer is disabled if pass NULL as @p ringBuffer.
 *
 * @param pointer to FLEXIO_UART_Type structure
 * @param handle pointer to flexio_uart_handle_t structure to store the transfer state.
 * @param callback The callback function.
 * @param userData The parameter of the callback function.
 * @retval kStatus_Success Successfully create the handle.
 * @retval kStatus_OutOfRange The flexio type/handle/isr table out of range.
 */
status_t FLEXIO_UART_CreateHandle(FLEXIO_UART_Type *base,
                                  flexio_uart_handle_t *handle,
                                  flexio_uart_transfer_callback_t callback,
                                  void *userData);

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
 * @param base pointer to FLEXIO_UART_Type structure.
 * @param handle pointer to flexio_uart_handle_t structure to store the transfer state.
 * @param ringBuffer Start address of ring buffer for backaround receiveing. Pass NULL to disable the ring buffer.
 * @param ringBufferSize size of the ringbuffer.
 */
void FLEXIO_UART_StartRingBuffer(FLEXIO_UART_Type *base,
                                 flexio_uart_handle_t *handle,
                                 uint8_t *ringBuffer,
                                 size_t ringBufferSize);

/*!
 * @brief Abort the background transfer and uninstall ring buffer.
 *
 * This function abort the background transfer and uninstall the ringbuffer.
 *
 * @param base pointer to FLEXIO_UART_Type structure.
 * @param handle pointer to flexio_uart_handle_t structure to store the transfer state.
 */
void FLEXIO_UART_StopRingBuffer(FLEXIO_UART_Type *base, flexio_uart_handle_t *handle);

/*!
 * @brief Transmit a buffer of data using the interrupt method
 *
 * This function send data using interrupt method. This is non-blocking function,
 * returns directly without waiting for all data written to TX register. When
 * all data are written to TX register in ISR, FLEXIO UART driver calls the callback
 * function and pass @ref kStatus_FLEXIO_UART_TxIdle as status parameter.
 *
 * @note The kStatus_FLEXIO_UART_TxIdle is passed to upper layer when all data written
 * to TX register, but not ensure all the data sent out.
 *
 * @param base pointer to FLEXIO_UART_Type structure.
 * @param handle pointer to flexio_uart_handle_t structure to store the transfer state.
 * @param xfer flexio uart transfer sturcture, refer to #flexio_uart_transfer_t.
 * @retval kStatus_Success Sucessully start the data transmission.
 * @retval kStatus_UART_TxBusy Previous transmission still not finished, data not all written to TX register yet.
 */
status_t FLEXIO_UART_SendNonBlocking(FLEXIO_UART_Type *base,
                                     flexio_uart_handle_t *handle,
                                     flexio_uart_transfer_t *xfer);

/*!
 * @brief Abort interrupt driven data transmit.
 *
 * This function aborts interrupt driven data sending. User can get the remainBytes to know
 * how many bytes still not sent out.
 *
 * @param base pointer to FLEXIO_UART_Type structure.
 * @param handle pointer to flexio_uart_handle_t structure to store the transfer state.
 */
void FLEXIO_UART_AbortSend(FLEXIO_UART_Type *base, flexio_uart_handle_t *handle);

/*!
 * @brief Get the number of remaining bytes not send.
 *
 * This function gets the number of remaining bytes not send driven by interrupt.
 *
 * @param base pointer to FLEXIO_UART_Type structure.
 * @param handle pointer to flexio_uart_handle_t structure to store the transfer state.
 * @return The number of bytes not send.
 */
static inline size_t FLEXIO_UART_GetSendRemainingBytes(FLEXIO_UART_Type *base, flexio_uart_handle_t *handle)
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
 * @param base pointer to FLEXIO_UART_Type structure.
 * @param handle pointer to flexio_uart_handle_t structure to store the transfer state.
 * @param xfer UART transfer sturcture, refer to #flexio_uart_transfer_t.
 * @param receivedBytes Bytes received from the ring buffer directly.
 * @retval kStatus_Success Sucessully queue the transfer into transmit queue.
 * @retval kStatus_FLEXIO_UART_RxBusy Previous receive requst is not finished.
 */
status_t FLEXIO_UART_ReceiveNonBlocking(FLEXIO_UART_Type *base,
                                        flexio_uart_handle_t *handle,
                                        flexio_uart_transfer_t *xfer,
                                        size_t *receivedBytes);

/*!
 * @brief Abort the receive data which using IRQ
 *
 * This function abort receive data which using IRQ.
 *
 * @param base pointer to FLEXIO_UART_Type structure.
 * @param handle pointer to flexio_uart_handle_t structure to store the transfer state.
 */
void FLEXIO_UART_AbortReceive(FLEXIO_UART_Type *base, flexio_uart_handle_t *handle);

/*!
 * @brief Get the number of remaining bytes not received.
 *
 * This function gets the number of remaining bytes not received driven by interrupt.
 *
 * @param base pointer to FLEXIO_UART_Type structure.
 * @param handle pointer to flexio_uart_handle_t structure to store the transfer state.
 * @return The number of bytes not receive.
 */
static inline size_t FLEXIO_UART_GetReceiveRemainingBytes(FLEXIO_UART_Type *base, flexio_uart_handle_t *handle)
{
    return handle->rxDataSize;
}

/*!
 * @brief FLEXIO UART IRQ handler function
 *
 * This function process the FLEXIO UART transmit and receive IRQ requestion
 *
 * @param uartType pointer to FLEXIO_UART_Type structure.
 * @param handle pointer to flexio_uart_handle_t structure to store the transfer state.
 */
void FLEXIO_UART_HandleIRQ(void *uartType, void *uartHandle);

/*@}*/

#if defined(__cplusplus)
}
#endif /*_cplusplus*/
/*@}*/

#endif /*_FSL_FLEXIO_UART_H_*/
