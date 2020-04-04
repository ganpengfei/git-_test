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
#ifndef _FSL_DSPI_EDMA_H_
#define _FSL_DSPI_EDMA_H_

#include "fsl_dspi.h"
#include "fsl_edma.h"
/*!
 * @addtogroup dspi_edma_driver
 * @{
 */
/***********************************************************************************************************************
 * Definitions
 **********************************************************************************************************************/

/*!
* @brief Forward declaration of the dspi edma master handle typedefs.
*/
typedef struct _dspi_master_edma_handle dspi_master_edma_handle_t;

/*!
* @brief Forward declaration of the dspi edma slave handle typedefs.
*/
typedef struct _dspi_slave_edma_handle dspi_slave_edma_handle_t;

/*!
 * @brief Completion callback function pointer type.
 *
 * @param base DSPI peripheral base address.
 * @param handle Pointer to the handle for the DSPI master.
 * @param status Success or error code describing whether the transfer completed.
 * @param userData Arbitrary pointer-dataSized value passed from the application.
 */
typedef void (*dspi_master_edma_transfer_callback_t)(SPI_Type *base,
                                                     dspi_master_edma_handle_t *handle,
                                                     status_t status,
                                                     void *userData);
/*!
 * @brief Completion callback function pointer type.
 *
 * @param base DSPI peripheral base address.
 * @param handle Pointer to the handle for the DSPI slave.
 * @param status Success or error code describing whether the transfer completed.
 * @param userData Arbitrary pointer-dataSized value passed from the application.
 */
typedef void (*dspi_slave_edma_transfer_callback_t)(SPI_Type *base,
                                                    dspi_slave_edma_handle_t *handle,
                                                    status_t status,
                                                    void *userData);

/*! @brief DSPI master edma transfer handle structure used for transactional API. */
struct _dspi_master_edma_handle
{
    uint32_t bitsPerFrame;         /*!< Desired number of bits per frame. */
    volatile uint32_t command;     /*!< Desired data command. */
    volatile uint32_t lastCommand; /*!< Desired last data command. */

    uint8_t fifoSize; /*!< Fifo dataSize. */

    volatile bool isPcsActiveAfterTransfer; /*!< Is PCS signal keep active after the last frame transfer.*/
    volatile bool isThereExtraByte;         /*!< Is there extra byte.*/

    uint8_t *volatile txData;                  /*!< Send buffer. */
    uint8_t *volatile rxData;                  /*!< Receive buffer. */
    volatile size_t remainingSendByteCount;    /*!< Number of bytes remaining to send.*/
    volatile size_t remainingReceiveByteCount; /*!< Number of bytes remaining to receive.*/
    size_t totalByteCount;                     /*!< Number of transfer bytes*/

    uint32_t rxBuffIfNull; /*!< Used if there is not rxData for DMA purpose.*/
    uint32_t txBuffIfNull; /*!< Used if there is not txData for DMA purpose.*/

    volatile uint8_t state; /*!< DSPI transfer state , _dspi_transfer_state.*/

    dspi_master_edma_transfer_callback_t callback; /*!< Completion callback. */
    void *userData;                                /*!< Callback user data. */

    edma_handle_t *edmaRxRegToRxDataHandle;        /*!<edma_handle_t handle point used for RxReg to RxData buff*/
    edma_handle_t *edmaTxDataToIntermediaryHandle; /*!<edma_handle_t handle point used for TxData to Intermediary*/
    edma_handle_t *edmaIntermediaryToTxRegHandle;  /*!<edma_handle_t handle point used for Intermediary to TxReg*/

    edma_tcd_t dspiSoftwareTCD[2]; /*!<SoftwareTCD , internal used*/
};

/*! @brief DSPI slave edma transfer handle structure used for transactional API.*/
struct _dspi_slave_edma_handle
{
    uint32_t bitsPerFrame;          /*!< Desired number of bits per frame. */
    volatile bool isThereExtraByte; /*!< Is there extra byte.*/

    uint8_t *volatile txData;                  /*!< Send buffer. */
    uint8_t *volatile rxData;                  /*!< Receive buffer. */
    volatile size_t remainingSendByteCount;    /*!< Number of bytes remaining to send.*/
    volatile size_t remainingReceiveByteCount; /*!< Number of bytes remaining to receive.*/
    size_t totalByteCount;                     /*!< Number of transfer bytes*/

    uint32_t rxBuffIfNull; /*!< Used if there is not rxData for DMA purpose.*/
    uint32_t txBuffIfNull; /*!< Used if there is not txData for DMA purpose.*/
    uint32_t txLastData;   /*!< Used if there is an extra byte when 16bits per frame for DMA purpose.*/

    volatile uint8_t state; /*!< DSPI transfer state.*/

    uint32_t errorCount; /*!< Error count for slave transfer.*/

    dspi_slave_edma_transfer_callback_t callback; /*!< Completion callback. */
    void *userData;                               /*!< Callback user data. */

    edma_handle_t *edmaRxRegToRxDataHandle; /*!<edma_handle_t handle point used for RxReg to RxData buff*/
    edma_handle_t *edmaTxDataToTxRegHandle; /*!<edma_handle_t handle point used for TxData to TxReg*/

    edma_tcd_t dspiSoftwareTCD[2]; /*!<SoftwareTCD , internal used*/
};

/***********************************************************************************************************************
 * API
 **********************************************************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif /*_cplusplus*/

/*Transactional APIs*/

/*!
 * @brief Initialize the DSPI master edma handle.
 *
 * This function initializes the DSPI edma handle which can be used for other DSPI transactional APIs.  Usually, for a
 * specified DSPI instance, user need only call this API once to get the initialized handle.
 *
 * Note that DSPI edma has separated(rx and tx , 2 source) or shared(rx  and tx is the same source) dma request source .
 * (1)For separated dma request source , you should enable and set the Rx DMAMUX source for edmaRxRegToRxDataHandle and
 * Tx DMAMUX source for edmaIntermediaryToTxRegHandle.
 * (2)For shared dma request source , you should enable and set the Rx/Rx DMAMUX source for edmaRxRegToRxDataHandle .
 *
 * @param base DSPI peripheral base address.
 * @param handle DSPI handle pointer to dspi_master_edma_handle_t.
 * @param callback dspi callback.
 * @param userData callback function parameter.
 * @param edmaRxRegToRxDataHandle edmaRxRegToRxDataHandle pointer to edma_handle_t.
 * @param edmaTxDataToIntermediaryHandle edmaTxDataToIntermediaryHandle pointer to edma_handle_t.
 * @param edmaIntermediaryToTxRegHandle edmaIntermediaryToTxRegHandle pointer to edma_handle_t.
 */
void DSPI_MasterCreateEdmaHandle(SPI_Type *base,
                                 dspi_master_edma_handle_t *handle,
                                 dspi_master_edma_transfer_callback_t callback,
                                 void *userData,
                                 edma_handle_t *edmaRxRegToRxDataHandle,
                                 edma_handle_t *edmaTxDataToIntermediaryHandle,
                                 edma_handle_t *edmaIntermediaryToTxRegHandle);

/*!
 * @brief DSPI master transfer data using EDMA.
 *
 * This function transfer data using EDMA , this is non-blocking function, will return right away, when all data
 * have been transfer, the callback function will be called.
 *
 * @param base DSPI peripheral base address.
 * @param handle pointer to dspi_master_edma_handle_t structure which stores the transfer state.
 * @param transfer pointer to dspi_transfer_t structure.
 * @return status of status_t.
 */
status_t DSPI_MasterTransferEDMA(SPI_Type *base, dspi_master_edma_handle_t *handle, dspi_transfer_t *transfer);

/*!
 * @brief DSPI master abort transfer which using edma.
 *
 * This function abort transfer which using edma.
 *
 * @param base DSPI peripheral base address.
 * @param handle pointer to dspi_master_edma_handle_t structure which stores the transfer state.
 */
void DSPI_MasterAbortEDMA(SPI_Type *base, dspi_master_edma_handle_t *handle);

/*!
 * @brief Get the master edma transfer remaining bytes.
 *
 * This function get the master edma transfer remaining bytes.
 *
 * @param base DSPI peripheral base address.
 * @param handle pointer to dspi_master_edma_handle_t structure which stores the transfer state.
 * @return remaining bytes of size_t.
 */
size_t DSPI_MasterGetTransferRemainingBytesEDMA(SPI_Type *base, dspi_master_edma_handle_t *handle);

/*!
 * @brief Initialize the dspi slave edma handle.
 *
 * This function initializes the DSPI edma handle which can be used for other DSPI transactional APIs.  Usually, for a
 * specified DSPI instance, user need only call this API once to get the initialized handle.
 *
 * Note that DSPI edma has separated(rx and tx , 2 source) or shared(rx  and tx is the same source) dma request source .
 * (1)For separated dma request source , you should enable and set the Rx DMAMUX source for edmaRxRegToRxDataHandle and
 * Tx DMAMUX source for edmaTxDataToTxRegHandle.
 * (2)For shared dma request source , you should enable and set the Rx/Rx DMAMUX source for edmaRxRegToRxDataHandle .
 *
 * @param base DSPI peripheral base address.
 * @param handle DSPI handle pointer to dspi_slave_edma_handle_t.
 * @param callback dspi callback.
 * @param userData callback function parameter.
 * @param edmaRxRegToRxDataHandle edmaRxRegToRxDataHandle pointer to edma_handle_t.
 * @param edmaTxDataToTxRegHandle edmaTxDataToTxRegHandle pointer to edma_handle_t.
 */
void DSPI_SlaveCreateEdmaHandle(SPI_Type *base,
                                dspi_slave_edma_handle_t *handle,
                                dspi_slave_edma_transfer_callback_t callback,
                                void *userData,
                                edma_handle_t *edmaRxRegToRxDataHandle,
                                edma_handle_t *edmaTxDataToTxRegHandle);

/*!
 * @brief DSPI slave transfer data using EDMA.
 *
 * This function transfer data using EDMA , this is non-blocking function, will return right away, when all data
 * have been transfer, the callback function will be called.
 *
 * Note that slave EDMA transfer cannot support the situation that transfer_size is 1 when the bitsPerFrame is greater
 * than 8 .

 * @param base DSPI peripheral base address.
 * @param handle pointer to dspi_slave_edma_handle_t structure which stores the transfer state.
 * @param transfer pointer to dspi_transfer_t structure.
 * @return status of status_t.
 */
status_t DSPI_SlaveTransferEDMA(SPI_Type *base, dspi_slave_edma_handle_t *handle, dspi_transfer_t *transfer);

/*!
 * @brief DSPI slave abort transfer which using edma.
 *
 * This function abort transfer which using edma.
 *
 * @param base DSPI peripheral base address.
 * @param handle pointer to dspi_slave_edma_handle_t structure which stores the transfer state.
 */
void DSPI_SlaveAbortEDMA(SPI_Type *base, dspi_slave_edma_handle_t *handle);

/*!
 * @brief Get the slave edma transfer remaining bytes.
 *
 * This function get the slave edma transfer remaining bytes.
 *
 * @param base DSPI peripheral base address.
 * @param handle pointer to dspi_slave_edma_handle_t structure which stores the transfer state.
 * @return remaining bytes of size_t.
 */
size_t DSPI_SlaveGetTransferRemainingBytesEDMA(SPI_Type *base, dspi_slave_edma_handle_t *handle);

#if defined(__cplusplus)
}
#endif /*_cplusplus*/
       /*!
        *@}
        */

#endif /*_FSL_DSPI_EDMA_H_*/
