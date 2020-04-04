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
#ifndef _FSL_FLEXIO_I2S_EDMA_H_
#define _FSL_FLEXIO_I2S_EDMA_H_

#include "fsl_flexio_i2s.h"
#include "fsl_edma.h"

/*!
 * @addtogroup flexio_i2s
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
typedef struct _flexio_i2s_edma_handle flexio_i2s_edma_handle_t;

/*! @brief FLEXIO I2S EDMA transfer callback function for finish and error */
typedef void (*flexio_i2s_edma_callback_t)(FLEXIO_I2S_Type *base,
                                           flexio_i2s_edma_handle_t *handle,
                                           status_t status,
                                           void *userData);

/*! @brief FLEXIO I2S DMA transfer handle, users should not touch the content of the handle.*/
struct _flexio_i2s_edma_handle
{
    edma_handle_t *dmaHandle;                        /*!< DMA handler for FLEXIO I2S send */
    uint8_t bytesPerFrame;                           /*!< Bytes in a frame */
    uint32_t state;                                  /*!< Internal state for FLEXIO I2S EDMA transfer */
    flexio_i2s_edma_callback_t callback;             /*!< Callback for users while transfer finish or error occured */
    void *userData;                                  /*!< User callback parameter */
    edma_tcd_t tcd[FLEXIO_I2S_XFER_QUEUE_SIZE + 1U]; /*!< TCD pool for EDMA transfer. */
    flexio_i2s_transfer_t queue[FLEXIO_I2S_XFER_QUEUE_SIZE]; /*!< Transfer queue storing queued transfer. */
    volatile uint8_t queueUser;                              /*!< Index for user to queue transfer. */
    volatile uint8_t queueDriver;                            /*!< Index for driver to get the transfer data and size */
};

/*******************************************************************************
 * APIs
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name EDMA Transactional
 * @{
 */

/*!
 * @brief Initialize the FLEXIO I2S EDMA handle.
 *
 * This function initializes the FLEXIO I2S master DMA handle which can be used for other FLEXIO I2S master
 * transactional APIs.
 * Usually, for a specified FLEXIO I2S instance, user need only call this API once to get the initialized handle.
 *
 * @param handle FLEXIO I2S handle pointer.
 * @param base FLEXIO I2S peripheral base address.
 */
void FLEXIO_I2S_TxCreateHandleEDMA(FLEXIO_I2S_Type *base,
                                   flexio_i2s_edma_handle_t *handle,
                                   flexio_i2s_edma_callback_t callback,
                                   void *userData,
                                   edma_handle_t *dmaHandle);

/*!
 * @brief Initialize the FLEXIO I2S Rx EDMA handle.
 *
 * This function initializes the FLEXIO I2S slave DMA handle which can be used for other FLEXIO I2S master transactional
 * APIs.
 * Usually, for a specified FLEXIO I2S instance, user need only call this API once to get the initialized handle.
 *
 * @param handle FLEXIO I2S handle pointer.
 * @param base FLEXIO I2S peripheral base address.
 */
void FLEXIO_I2S_RxCreateHandleEDMA(FLEXIO_I2S_Type *base,
                                   flexio_i2s_edma_handle_t *handle,
                                   flexio_i2s_edma_callback_t callback,
                                   void *userData,
                                   edma_handle_t *dmaHandle);

/*!
 * @brief Configure FLEXIO I2S Tx audio format.
 *
 * Audio format can be changed in run-time of FLEXIO I2S. This function configures the sample rate and audio data
 * format to be transferred. This function also sets EDMA parameter according to format.
 *
 * @param handle FLEXIO I2S handle pointer
 * @param format Pointer to FLEXIO I2S audio data format structure.
 * @param sourceClock_Hz FLEXIO I2S clock source frequency in Hz.
 * @retval kStatus_Success Audio format set successfully.
 * @retval kStatus_InvalidArgument The input arguments is invalid.
*/
void FLEXIO_I2S_SetTransferFormatEDMA(FLEXIO_I2S_Type *base,
                                      flexio_i2s_edma_handle_t *handle,
                                      flexio_i2s_format_t *format,
                                      uint32_t sourceClock_Hz);

/*!
 * @brief Perform a non-blocking FLEXIO I2S transfer using DMA.
 *
 * @note This interface returned immediately after transfer initiates, users should call
 * FLEXIO_I2S_GetTransferStatus to poll the transfer status to check whether FLEXIO I2S transfer finished.
 *
 * @param handle FLEXIO I2S DMA handle pointer.
 * @param xfer Pointer to dma transfer structure.
 * @retval kStatus_Success Start a FLEXIO I2S EDMA send successfully.
 * @retval kStatus_InvalidArgument The input arguments is invalid.
 * @retval kStatus_TxBusy FLEXIO I2S is busy sending data.
 */
status_t FLEXIO_I2S_SendEDMA(FLEXIO_I2S_Type *base, flexio_i2s_edma_handle_t *handle, flexio_i2s_transfer_t *xfer);

/*!
 * @brief Perform a non-blocking FLEXIO I2S receive using EDMA.
 *
 * @note This interface returned immediately after transfer initiates, users should call
 * FLEXIO_I2S_GetReceiveRemainingBytes to poll the transfer status to check whether FLEXIO I2S transfer finished.
 *
 * @param handle FLEXIO I2S DMA handle pointer.
 * @param xfer Pointer to dma transfer structure.
 * @retval kStatus_Success Start a FLEXIO I2S EDMA receive successfully.
 * @retval kStatus_InvalidArgument The input arguments is invalid.
 * @retval kStatus_RxBusy FLEXIO I2S is busy receiving data.
 */
status_t FLEXIO_I2S_ReceiveEDMA(FLEXIO_I2S_Type *base, flexio_i2s_edma_handle_t *handle, flexio_i2s_transfer_t *xfer);

/*!
 * @brief Abort a FLEXIO I2S transfer using EDMA.
 *
 * @param handle FLEXIO I2S DMA handle pointer.
 */
void FLEXIO_I2S_AbortSendEDMA(FLEXIO_I2S_Type *base, flexio_i2s_edma_handle_t *handle);

/*!
 * @brief Abort a FLEXIO I2S receive using EDMA.
 *
 * @param handle FLEXIO I2S DMA handle pointer.
 */
void FLEXIO_I2S_AbortReceiveEDMA(FLEXIO_I2S_Type *base, flexio_i2s_edma_handle_t *handle);

/*!
 * @brief Get the remaining bytes to be sent.
 *
 * @param handle FLEXIO I2S DMA handle pointer.
 * @return Remaining bytes to be sent.
 */
size_t FLEXIO_I2S_GetSendRemainingBytesEDMA(FLEXIO_I2S_Type *base, flexio_i2s_edma_handle_t *handle);

/*!
 * @brief Get the remaining bytes to be received.
 *
 * @param handle FLEXIO I2S DMA handle pointer.
 * @return Remaining bytes to be received.
 */
size_t FLEXIO_I2S_GetReceiveRemainingBytesEDMA(FLEXIO_I2S_Type *base, flexio_i2s_edma_handle_t *handle);

/*! @} */

#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */
#endif
