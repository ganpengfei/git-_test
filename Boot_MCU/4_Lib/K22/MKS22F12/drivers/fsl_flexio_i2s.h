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
#ifndef _FSL_FLEXIO_I2S_H_
#define _FSL_FLEXIO_I2S_H_

#include "fsl_common.h"
#include "fsl_flexio.h"

/*!
 * @addtogroup flexio_i2s
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief FLEXIO I2S tranfer status */
enum _flexio_i2s_status
{
    kStatus_FLEXIO_I2S_Idle = MAKE_STATUS(kStatusGroup_FLEXIO_I2S, 0),      /*!< Flexio I2S is in idle state */
    kStatus_FLEXIO_I2S_TxBusy = MAKE_STATUS(kStatusGroup_FLEXIO_I2S, 1),    /*!< Flexio I2S Tx is busy */
    kStatus_FLEXIO_I2S_RxBusy = MAKE_STATUS(kStatusGroup_FLEXIO_I2S, 2),    /*!< Flexio I2S Tx is busy */
    kStatus_FLEXIO_I2S_Error = MAKE_STATUS(kStatusGroup_FLEXIO_I2S, 3),     /*!< Flexio I2S error occured */
    kStatus_FLEXIO_I2S_QueueFull = MAKE_STATUS(kStatusGroup_FLEXIO_I2S, 4), /*!< Flexio I2S transfer queue is full. */
};

/*! @brief Define FLEXIO I2S access structure typedef */
typedef struct _flexio_i2s_type
{
    FLEXIO_Type *flexioBase; /*!< Flexio base pointer */
    uint8_t txPinIndex;      /*!< Tx data pin index in flexio pins */
    uint8_t rxPinIndex;      /*!< Rx data pin index */
    uint8_t bclkPinIndex;    /*!< Bit clock pin index */
    uint8_t fsPinIndex;      /*!< Frame sync pin index */
    uint8_t txShifterIndex;  /*!< Tx data shifiter index */
    uint8_t rxShifterIndex;  /*!< Rx data shifter index */
    uint8_t bclkTimerIndex;  /*!< Bit clock timer index */
    uint8_t fsTimerIndex;    /*!< Frame sync timer index */
} FLEXIO_I2S_Type;

/*! @brief Master or slave mode */
typedef enum _flexio_i2s_master_slave
{
    kFLEXIO_I2S_Master = 0x0U, /*!< Master mode */
    kFLEXIO_I2S_Slave = 0x1U   /*!< Slave mode */
} flexio_i2s_master_slave_t;

/*! @brief Define FLEXIO Flexio I2S interrupt mask. */
enum _flexio_i2s_interrupt_enable
{
    kFLEXIO_I2S_TxDataRegEmptyInterruptEnable = 0x1U, /*!< Transmit buffer empty interrupt enable. */
    kFLEXIO_I2S_RxDataRegFullInterruptEnable = 0x2U,  /*!< Receive buffer full interrupt enable. */
};

/*! @brief Define FLEXIO Flexio I2S status mask. */
enum _flexio_i2s_status_flags
{
    kFLEXIO_I2S_TxDataRegEmptyFlag = 0x1U, /*!< Transmit buffer empty flag. */
    kFLEXIO_I2S_RxDataRegFullFlag = 0x2U,  /*!< Receive buffer full flag. */
};

/*! @brief Flexio I2S configure structure */
typedef struct _flexio_i2s_config
{
    bool enableI2S;                        /*!< Enable flexio I2S */
    flexio_i2s_master_slave_t masterSlave; /*!< Master or slave */
} flexio_i2s_config_t;

/*! @brief Flexio I2S audio format, flexio I2S only support the same format in Tx and Rx */
typedef struct _flexio_i2s_format
{
    uint8_t bitWidth;
    uint32_t sampleRate_Hz;
} flexio_i2s_format_t;

/*!@brief Flexio I2S transfer queue size, user can refine it according to use case. */
#define FLEXIO_I2S_XFER_QUEUE_SIZE (4)

/*! @brief Audio sample rate */
typedef enum _flexio_i2s_sample_rate
{
    kFLEXIO_I2S_SampleRate8KHz = 8000U,     /*!< Sample rate 8000Hz */
    kFLEXIO_I2S_SampleRate11025Hz = 11025U, /*!< Sample rate 11025Hz */
    kFLEXIO_I2S_SampleRate12KHz = 12000U,   /*!< Sample rate 12000Hz */
    kFLEXIO_I2S_SampleRate16KHz = 16000U,   /*!< Sample rate 16000Hz */
    kFLEXIO_I2S_SampleRate22050Hz = 22050U, /*!< Sample rate 22050Hz */
    kFLEXIO_I2S_SampleRate24KHz = 24000U,   /*!< Sample rate 24000Hz */
    kFLEXIO_I2S_SampleRate32KHz = 32000U,   /*!< Sample rate 32000Hz */
    kFLEXIO_I2S_SampleRate44100Hz = 44100U, /*!< Sample rate 44100Hz */
    kFLEXIO_I2S_SampleRate48KHz = 48000U,   /*!< Sample rate 48000Hz */
    kFLEXIO_I2S_SampleRate96KHz = 96000U    /*!< Sample rate 96000Hz */
} flexio_i2s_sample_rate_t;

/*! @brief Audio word width */
typedef enum _flexio_i2s_word_width
{
    kFLEXIO_I2S_WordWidth8bits = 8U,   /*!< Audio data width 8 bits */
    kFLEXIO_I2S_WordWidth16bits = 16U, /*!< Audio data width 16 bits */
    kFLEXIO_I2S_WordWidth24bits = 24U, /*!< Audio data width 24 bits */
    kFLEXIO_I2S_WordWidth32bits = 32U  /*!< Audio data width 32 bits */
} flexio_i2s_word_width_t;

typedef struct _flexio_i2s_transfer
{
    uint8_t *data;   /*!< Data buffer start pointer */
    size_t dataSize; /*!< Bytes to be transferred. */
} flexio_i2s_transfer_t;

typedef struct _flexio_i2s_handle flexio_i2s_handle_t;

/*! @brief FLEXIO I2S xfer callback prototype */
typedef void (*flexio_i2s_callback_t)(FLEXIO_I2S_Type *base,
                                      flexio_i2s_handle_t *handle,
                                      status_t status,
                                      void *userData);

struct _flexio_i2s_handle
{
    uint32_t state;                                          /*!< Internal state */
    flexio_i2s_callback_t callback;                          /*!< Callback function called at transfer event*/
    void *userData;                                          /*!< Callback parameter passed to callback function*/
    uint8_t bitWidth;                                        /*!< Bit width for transfer, 8/16/24/32bits */
    flexio_i2s_transfer_t queue[FLEXIO_I2S_XFER_QUEUE_SIZE]; /*!< Transfer queue storing queued transfer */
    volatile uint8_t queueUser;                              /*!< Index for user to queue transfer */
    volatile uint8_t queueDriver;                            /*!< Index for driver to get the transfer data and size */
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
 * @brief Initialize the FLEXIO I2S
 *
 * This API configures flexio pins and shifter to I2S and configure FLEXIO I2S with configuration structure.
 * The configuration structure can be filled by user from scratch, or be set with default values by
 * FLEXIO_I2S_GetDefaultConfig().
 *
 * @note  This API should be called at the beginning of the application to use
 * the FLEXIO I2S driver, or any access to the FLEXIO I2S module could cause hard fault
 * because clock is not enabled.
 *
 * @param base FLEXIO I2S base pointer
 * @param config FLEXIO I2S configure structure.
*/
void FLEXIO_I2S_Init(FLEXIO_I2S_Type *base, const flexio_i2s_config_t *config);

/*!
 * @brief  Set the Flexio I2S configuration structure to default values.
 *
 * The purpose of this API is to get the config structure initialized for use in FLEXIO_I2S_Init().
 * User may use the initialized structure unchanged in FLEXIO_I2S_Init(), or modify
 * some fields of the structure before calling FLEXIO_I2S_Init().
 *
 * @param config pointer to master config structure
 */
void FLEXIO_I2S_GetDefaultConfig(flexio_i2s_config_t *config);

/*!
 * @brief De-initialize the FLEXIO I2S.
 *
 * Call thi API will gate the flexio i2s clock, after calling this API, user need to call FLEXO_I2S_Init to use the
 * FLEXIO I2S module
 *
 * @param base FLEXIO I2S base pointer
*/
void FLEXIO_I2S_Deinit(FLEXIO_I2S_Type *base);

/*!
 * @brief Enable/Disable the FLEXIO I2S module operation.
 *
 * @param base pointer to FLEXIO_I2S_Type
 * @param enable True to enable, false to disable.
*/
static inline void FLEXIO_I2S_Enable(FLEXIO_I2S_Type *base, bool enable)
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

/*! @} */

/*!
 * @name Status
 * @{
 */

/*!
 * @brief Get FLEXIO I2S status flags.
 *
 * @param base pointer to FLEXIO_I2S_Type structure
 * @param statusFlag status flag, users need to use Status Mask to get the status value needed.
*/
uint32_t FLEXIO_I2S_GetStatusFlags(FLEXIO_I2S_Type *base);

/*! @} */

/*!
 * @name Interrupts
 * @{
 */

/*!
 * @brief Enable flexio i2s interrupt.
 *
 * This function enable the flexio uart interrupt
 *
 * @param base pointer to FLEXIO_I2S_Type structure
 * @param mask interrupt source
 */
void FLEXIO_I2S_EnableInterrupts(FLEXIO_I2S_Type *base, uint32_t mask);

/*!
 * @brief Disable flexio i2s interrupt.
 *
 * This function enable the flexio uart interrupt
 *
 * @param base pointer to FLEXIO_I2S_Type structure
 * @param mask interrupt source
 */
void FLEXIO_I2S_DisableInterrupts(FLEXIO_I2S_Type *base, uint32_t mask);

/*! @} */

/*!
 * @name DMA Control
 * @{
 */

/*!
 * @brief Enables/Disable FLEXIO I2S Tx DMA requests.
 *
 * @param base FLEXIO I2S base pointer
 */
static inline void FLEXIO_I2S_TxEnableDMA(FLEXIO_I2S_Type *base, bool enable)
{
    FLEXIO_EnableShifterStatusDMA(base->flexioBase, 1 << base->txShifterIndex, enable);
}

/*!
 * @brief Enables/Disable FLEXIO I2S Rx DMA requests.
 *
 * @param base FLEXIO I2S base pointer
 */
static inline void FLEXIO_I2S_RxEnableDMA(FLEXIO_I2S_Type *base, bool enable)
{
    FLEXIO_EnableShifterStatusDMA(base->flexioBase, 1 << base->rxShifterIndex, enable);
}

/*!
 * @brief get flexio i2s send data register address
 *
 * This function return the i2s data register address, mainly used by DMA/eDMA
 *
 * @param base pointer to FLEXIO_I2S_Type structure
 * @return flexio i2s send data register address.
 */
static inline uint32_t FLEXIO_I2S_TxGetDataRegisterAddress(FLEXIO_I2S_Type *base)
{
    return FLEXIO_GetShifterBufferAddress(base->flexioBase, kFLEXIO_ShifterBufferBitSwapped, base->txShifterIndex);
}

/*!
 * @brief get flexio i2s receive data register address
 *
 * This function return the i2s data register address, mainly used by DMA/eDMA
 *
 * @param base pointer to FLEXIO_I2S_Type structure
 * @return flexio i2s receive data register address.
 */
static inline uint32_t FLEXIO_I2S_RxGetDataRegisterAddress(FLEXIO_I2S_Type *base)
{
    return FLEXIO_GetShifterBufferAddress(base->flexioBase, kFLEXIO_ShifterBufferBitSwapped, base->rxShifterIndex);
}

/*! @} */

/*!
 * @name Bus Operations
 * @{
 */

/*!
 * @brief Configure Flexio I2S audio format.
 *
 * Audio format can be changed in run-time of Flexio I2S. This function configures the sample rate and audio data
 * format to be transferred.
 *
 * @param handle FLEXIO I2S handle pointer
 * @param format Pointer to FLEXIO I2S audio data format structure.
 * @param mclkSourceClockHz I2S master clock source frequency in Hz.
*/
void FLEXIO_I2S_SetFormat(FLEXIO_I2S_Type *base, flexio_i2s_format_t *format, uint32_t sourceClock_Hz);

/*!
 * @brief sends a piece of data in blocking way.
 *
 * @note This function blocks via polling until data is ready to be sent.
 *
 * @param base FLEXIO I2S base pointer
 * @pa-ram bitWidth How many bits in a audio word, usually 8/16/24/32 bits.
 * @param buffer Pointer to the data to be written.
 * @param size Bytes to be written.
 */
void FLEXIO_I2S_WriteBlocking(FLEXIO_I2S_Type *base, uint8_t bitWidth, uint8_t *txData, size_t size);

/*!
 * @brief Write a data into data register.
 *
 * @param base FLEXIO I2S base pointer
 * @param bitWidth How many bits in a audio word, usually 8/16/24/32 bits.
 * @param data Data to be written.
 */
static inline void FLEXIO_I2S_WriteData(FLEXIO_I2S_Type * base, uint8_t bitWidth, uint32_t data)
{
    base->flexioBase->SHIFTBUFBIS[base->txShifterIndex] = (data << (32U - bitWidth));
}

/*!
 * @brief Receive a piece of data in blocking way.
 *
 * @note This function blocks via polling until data is ready to be sent.
 *
 * @param base FLEXIO I2S base pointer
 * @param bitWidth How many bits in a audio word, usually 8/16/24/32 bits.
 * @param buffer Pointer to the data to be read.
 * @param size Bytes to be read.
 */
void FLEXIO_I2S_ReadBlocking(FLEXIO_I2S_Type *base, uint8_t bitWidth, uint8_t *rxData, size_t size);

/*!
 * @brief Read a data from data register.
 *
 * @param base FLEXIO I2S base pointer
 * @return Data read from data register.
 */
static inline uint32_t FLEXIO_I2S_ReadData(FLEXIO_I2S_Type *base)
{
    return base->flexioBase->SHIFTBUFBIS[base->rxShifterIndex];
}

/*! @} */

/*!
 * @name Transactional
 * @{
 */

/*!
 * @brief Initialize the Flexio I2S handle.
 *
 * This function initializes the FLEXIO I2S handle which can be used for other
 * Flexio I2S transactional APIs. Usually, user only need to call this API once to get the
 * initialized handle.
 *
 * @param handle pointer to flexio_i2s_handle_t structure to store the transfer state.
 * @param base pointer to FLEXIO_I2S_Type structure
 */
void FLEXIO_I2S_TxCreateHandle(FLEXIO_I2S_Type *base,
                               flexio_i2s_handle_t *handle,
                               flexio_i2s_callback_t callback,
                               void *userData);

/*!
 * @brief Configure flexio i2s audio format.
 *
 * Audio format can be changed in run-time of flexio i2s. This function configures the sample rate and audio data
 * format to be transferred.
 *
 * @param handle SAI handle pointer
 * @param format Pointer to SAI audio data format structure.
 * @param sourceClockHz SAI bit clock source frequency in Hz. This parameter can be 0 while in slave mode.
*/
void FLEXIO_I2S_SetTransferFormat(FLEXIO_I2S_Type *base,
                                  flexio_i2s_handle_t *handle,
                                  flexio_i2s_format_t *format,
                                  uint32_t sourceClock_Hz);

/*!
 * @brief Initialize the Flexio I2S handle.
 *
 * This function initializes the FLEXIO I2S handle which can be used for other
 * Flexio I2S transactional APIs. Usually, user only need to call this API once to get the
 * initialized handle.
 *
 * @param handle pointer to flexio_i2s_handle_t structure to store the transfer state.
 * @param base pointer to FLEXIO_I2S_Type structure
 */
void FLEXIO_I2S_RxCreateHandle(FLEXIO_I2S_Type *base,
                               flexio_i2s_handle_t *handle,
                               flexio_i2s_callback_t callback,
                               void *userData);

/*!
 * @brief Performs an interrupt non-blocking send transfer on FLEXIO I2S
 *
 * @note Calling the API will return immediately after transfer initiates, user needs
 * to call FLEXIO_I2S_GetRemainingBytes to poll the transfer status to check whether
 * the transfer is finished, if the return status is 0, the transfer is finished.
 *
 * @param handle pointer to flexio_i2s_handle_t structure which stores the transfer state
 * @param xfer pointer to flexio_i2s_transfer_t structure
 * @return status of status_t
 */
status_t FLEXIO_I2S_SendNonBlocking(FLEXIO_I2S_Type *base, flexio_i2s_handle_t *handle, flexio_i2s_transfer_t *xfer);

/*!
 * @brief Performs an interrupt non-blocking receive transfer on FLEXIO I2S
 *
 * @note Calling the API will return immediately after transfer initiates, user needs
 * to call FLEXIO_I2S_GetRemainingBytes to poll the transfer status to check whether
 * the transfer is finished, if the return status is 0, the transfer is finished.
 *
 * @param handle pointer to flexio_i2s_handle_t structure which stores the transfer state
 * @param xfer pointer to flexio_i2s_transfer_t structure
 * @return status of status_t
 */
status_t FLEXIO_I2S_ReceiveNonBlocking(FLEXIO_I2S_Type *base, flexio_i2s_handle_t *handle, flexio_i2s_transfer_t *xfer);

/*!
 * @brief Abort the current send.
 *
 * @note This API could be called at any time when interrupt non-blocking transfer initiates
 * to abort the transfer in a early time.
 *
 * @param handle pointer to flexio_i2s_handle_t structure which stores the transfer state
 */
void FLEXIO_I2S_AbortSend(FLEXIO_I2S_Type *base, flexio_i2s_handle_t *handle);

/*!
 * @brief Abort the current receive.
 *
 * @note This API could be called at any time when interrupt non-blocking transfer initiates
 * to abort the transfer in a early time.
 *
 * @param handle pointer to flexio_i2s_handle_t structure which stores the transfer state
 */
void FLEXIO_I2S_AbortReceive(FLEXIO_I2S_Type *base, flexio_i2s_handle_t *handle);

/*!
 * @brief Get the remaining bytes to be sent.
 *
 * @param handle pointer to flexio_i2s_handle_t structure which stores the transfer state
 * @return Remaining bytes to be sent.
 */
size_t FLEXIO_I2S_GetSendRemainingBytes(FLEXIO_I2S_Type *base, flexio_i2s_handle_t *handle);

/*!
 * @brief Get the remaining bytes to be received.
 *
 * @param handle pointer to flexio_i2s_handle_t structure which stores the transfer state
 * @return Remaining bytes to be received.
 */
size_t FLEXIO_I2S_GetReceiveRemainingBytes(FLEXIO_I2S_Type *base, flexio_i2s_handle_t *handle);

/*!
 * @brief Tx interrupt handler
 *
 * @param handle pointer to flexio_i2s_handle_t structure
 */
void FLEXIO_I2S_TxHandleIRQ(void *i2sBase, void *i2sHandle);

/*!
 * @brief Rx interrupt handler
 *
 * @param handle pointer to flexio_i2s_handle_t structure
 */
void FLEXIO_I2S_RxHandleIRQ(void *i2sBase, void *i2sHandle);

/*! @} */

#if defined(__cplusplus)
}
#endif /*_cplusplus*/

/*! @} */

#endif /* _FSL_FLEXIO_I2S_H_ */
