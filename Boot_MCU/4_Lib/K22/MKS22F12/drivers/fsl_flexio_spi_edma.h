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
#ifndef _FSL_FLEXIO_SPI_EDMA_H_
#define _FSL_FLEXIO_SPI_EDMA_H_

#include "fsl_flexio_spi.h"
#include "fsl_edma.h"

/*!
 * @addtogroup flexio_spi
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief  typedef for flexio_spi_master_edma_handle_t in advance. */
typedef struct _flexio_spi_master_edma_handle flexio_spi_master_edma_handle_t;

/*! @brief  Slave handle is the same with master handle. */
typedef flexio_spi_master_edma_handle_t flexio_spi_slave_edma_handle_t;

/*! @brief FLEXIO SPI master callback for finished transmit */
typedef void (*flexio_spi_master_edma_transfer_callback_t)(FLEXIO_SPI_Type *base,
                                                           flexio_spi_master_edma_handle_t *handle,
                                                           status_t status,
                                                           void *userData);

/*! @brief FLEXIO SPI slave callback for finished transmit */
typedef void (*flexio_spi_slave_edma_transfer_callback_t)(FLEXIO_SPI_Type *base,
                                                          flexio_spi_slave_edma_handle_t *handle,
                                                          status_t status,
                                                          void *userData);

/*! @brief FLEXIO SPI EDMA transfer handle, users should not touch the content of the handle.*/
struct _flexio_spi_master_edma_handle
{
    bool txInProgress;                                   /*!< Send transfer in progress */
    bool rxInProgress;                                   /*!< Receive transfer in progress */
    edma_handle_t *txHandle;                             /*!< DMA handler for SPI send */
    edma_handle_t *rxHandle;                             /*!< DMA handler for SPI receive */
    flexio_spi_master_edma_transfer_callback_t callback; /*!< Callback for SPI DMA transfer */
    void *userData;                                      /*!< User Data for SPI DMA callback */
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
 * @brief Initialize the FLEXO SPI master EDMA handle.
 *
 * This function initializes the FLEXO SPI master EDMA handle which can be used for other FLEXO SPI master transactional
 * APIs.
 * Usually, for a specified FLEXO SPI instance, user need only call this API once to get the initialized handle.
 *
 * @param base pointer to FLEXIO_SPI_Type structure.
 * @param handle pointer to flexio_spi_master_edma_handle_t structure to store the transfer state.
 * @param callback spi callback, NULL means no callback.
 * @param userData callback function parameter.
 * @param txHandle User requested EDMA handle for FLEXIO SPI RX EDMA transfer.
 * @param rxHandle User requested EDMA handle for FLEXIO SPI TX EDMA transfer.
 * @retval kStatus_Success Successfully create the handle.
 * @retval kStatus_OutOfRange The flexio spi edma type/handle table out of range.
 */
status_t FLEXIO_SPI_MasterCreateHandleEDMA(FLEXIO_SPI_Type *base,
                                       flexio_spi_master_edma_handle_t *handle,
                                       flexio_spi_master_edma_transfer_callback_t callback,
                                       void *userData,
                                       edma_handle_t *txHandle,
                                       edma_handle_t *rxHandle);

/*!
 * @brief Perform a non-blocking FLEXIO SPI transfer using EDMA.
 *
 * @note This interface returned immediately after transfer initiates, users could call
 * FLEXIO_SPI_MasterGetTransferRemainingBytesEDMA to poll the transfer status to check
 * whether FLEXIO SPI transfer finished.
 *
 * @param base pointer to FLEXIO_SPI_Type structure.
 * @param handle pointer to flexio_spi_master_edma_handle_t structure to store the transfer state.
 * @param xfer Pointer to flexio spi transfer structure.
 * @retval kStatus_Success Successfully start a transfer.
 * @retval kStatus_InvalidArgument Input argument is invalid.
 * @retval kStatus_FLEXIO_SPI_Busy FLEXIO SPI is not idle, is running another transfer.
 */
status_t FLEXIO_SPI_MasterTransferEDMA(FLEXIO_SPI_Type *base,
                                       flexio_spi_master_edma_handle_t *handle,
                                       flexio_spi_transfer_t *xfer);

/*!
 * @brief Abort a FLEXIO SPI transfer using EDMA.
 *
 * @param base pointer to FLEXIO_SPI_Type structure.
 * @param handle FLEXIO SPI EDMA handle pointer.
 */
void FLEXIO_SPI_MasterAbortTransferEDMA(FLEXIO_SPI_Type *base, flexio_spi_master_edma_handle_t *handle);

/*!
 * @brief Get the remaining bytes for FLEXIO SPI EDMA transfer.
 *
 * @param base pointer to FLEXIO_SPI_Type structure.
 * @param handle FLEXIO SPI EDMA handle pointer.
 * @return Remaining bytes to be transferred.
 */
size_t FLEXIO_SPI_MasterGetTransferRemainingBytesEDMA(FLEXIO_SPI_Type *base, flexio_spi_master_edma_handle_t *handle);

/*!
 * @brief Initialize the FLEXIO SPI slave EDMA handle.
 *
 * This function initializes the FLEXIO SPI slave EDMA handle.
 *
 * @param base pointer to FLEXIO_SPI_Type structure.
 * @param handle pointer to flexio_spi_slave_edma_handle_t structure to store the transfer state.
 * @param callback spi callback, NULL means no callback.
 * @param userData callback function parameter.
 * @param txHandle User requested EDMA handle for FLEXIO SPI TX EDMA transfer.
 * @param rxHandle User requested EDMA handle for FLEXIO SPI RX EDMA transfer.
 */
static inline void FLEXIO_SPI_SlaveCreateHandleEDMA(FLEXIO_SPI_Type *base,
                                                    flexio_spi_slave_edma_handle_t *handle,
                                                    flexio_spi_slave_edma_transfer_callback_t callback,
                                                    void *userData,
                                                    edma_handle_t *txHandle,
                                                    edma_handle_t *rxHandle)
{
    FLEXIO_SPI_MasterCreateHandleEDMA(base, handle, callback, userData, txHandle, rxHandle);
}

/*!
 * @brief Perform a non-blocking FLEXIO SPI transfer using EDMA.
 *
 * @note This interface returned immediately after transfer initiates, users could call
 * FLEXIO_SPI_SlaveGetTransferRemainingBytesEDMA to poll the transfer status to
 * check whether FLEXIO SPI transfer finished.
 *
 * @param base pointer to FLEXIO_SPI_Type structure.
 * @param handle pointer to flexio_spi_slave_edma_handle_t structure to store the transfer state.
 * @param xfer Pointer to flexio spi transfer structure.
 * @retval kStatus_Success Successfully start a transfer.
 * @retval kStatus_InvalidArgument Input argument is invalid.
 * @retval kStatus_FLEXIO_SPI_Busy FLEXIO SPI is not idle, is running another transfer.
 */
static inline status_t FLEXIO_SPI_SlaveTransferEDMA(FLEXIO_SPI_Type *base,
                                            flexio_spi_slave_edma_handle_t *handle,
                                            flexio_spi_transfer_t *xfer)
{

    return FLEXIO_SPI_MasterTransferEDMA(base, handle, xfer);
}

/*!
 * @brief Abort a FLEXIO SPI transfer using EDMA.
 *
 * @param base pointer to FLEXIO_SPI_Type structure.
 * @param handle pointer to flexio_spi_slave_edma_handle_t structure to store the transfer state.
 */
static inline void FLEXIO_SPI_SlaveAbortTransferEDMA(FLEXIO_SPI_Type *base, flexio_spi_slave_edma_handle_t *handle)
{
    FLEXIO_SPI_MasterAbortTransferEDMA(base, handle);
}

/*!
 * @brief Get the remaining bytes to be transferred for FLEXIO SPI EDMA.
 *
 * @param base pointer to FLEXIO_SPI_Type structure.
 * @param handle FLEXIO SPI EDMA handle pointer.
 * @return Remaining bytes to be transferred.
 */
static inline size_t FLEXIO_SPI_SlaveGetTransferRemainingBytesEDMA(FLEXIO_SPI_Type *base,
                                                                   flexio_spi_slave_edma_handle_t *handle)
{
    return FLEXIO_SPI_MasterGetTransferRemainingBytesEDMA(base, handle);
}

/*! @} */

#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */
#endif
