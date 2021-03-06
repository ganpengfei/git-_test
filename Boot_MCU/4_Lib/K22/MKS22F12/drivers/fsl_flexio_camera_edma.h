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
#ifndef _FSL_FLEXIO_CAMERA_EDMA_H_
#define _FSL_FLEXIO_CAMERA_EDMA_H_

#include "fsl_flexio_camera.h"
#include "fsl_dmamux.h"
#include "fsl_edma.h"

/*!
 * @addtogroup flexio_camera
 * @{
 */

/*! @file*/

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Forward declaration of the handle typedef. */
typedef struct _flexio_camera_edma_handle flexio_camera_edma_handle_t;

typedef void (*flexio_camera_edma_transfer_callback_t)(FLEXIO_CAMERA_Type *base,
                                                       flexio_camera_edma_handle_t *handle,
                                                       status_t status,
                                                       void *userData);

/*!
* @brief CAMERA EDMA handle
*/
struct _flexio_camera_edma_handle
{
    flexio_camera_edma_transfer_callback_t callback; /*!< Callback function. */
    void *userData;                                  /*!< CAMERA callback function parameter.*/
    edma_handle_t *rxEdmaHandle;                     /*!< The EDMA RX channel used. */
    volatile uint8_t rxState;                        /*!< RX transfer state */
};

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name EDMA transactional
 * @{
 */

/*!
 * @brief Init the CAMERA handle which is used in transcational functions.
 *
 * @param base pointer to FLEXIO_CAMERA_Type.
 * @param handle Pointer to flexio_camera_edma_handle_t structure.
 * @param callback The callback function.
 * @param userData The parameter of the callback function.
 * @param rxEdmaHandle User requested DMA handle for RX DMA transfer.
 * @retval kStatus_Success Successfully create the handle.
 * @retval kStatus_OutOfRange The flexio camera edma type/handle table out of range.
 */
status_t FLEXIO_CAMERA_CreateHandleEDMA(FLEXIO_CAMERA_Type *base,
                                    flexio_camera_edma_handle_t *handle,
                                    flexio_camera_edma_transfer_callback_t callback,
                                    void *userData,
                                    edma_handle_t *rxEdmaHandle);

/*!
 * @brief Receive data using EDMA
 *
 * This function receive data using EDMA, this is non-blocking function, will return
 * right away, when all data have been received, the receive callback function will be called.
 *
 * @param base pointer to FLEXIO_CAMERA_Type.
 * @param handle Pointer to flexio_camera_edma_handle_t structure.
 * @param xfer CAMERA EDMA transfer sturcture, refer to #flexio_camera_transfer_t.
 * @retval kStatus_Success if succeed, others failed.
 * @retval kStatus_CAMERA_RxBusy Previous transfer on going.
 */
status_t FLEXIO_CAMERA_ReceiveEDMA(FLEXIO_CAMERA_Type *base,
                                   flexio_camera_edma_handle_t *handle,
                                   flexio_camera_transfer_t *xfer);

/*!
 * @brief Abort the receive data which using EDMA.
 *
 * This function abort receive data which using EDMA.
 *
 * @param base pointer to FLEXIO_CAMERA_Type.
 * @param handle Pointer to flexio_camera_edma_handle_t structure.
 */
void FLEXIO_CAMERA_AbortReceiveEDMA(FLEXIO_CAMERA_Type *base, flexio_camera_edma_handle_t *handle);

/*!
 * @brief Get the number of bytes still not received.
 *
 * This function gets the number of bytes still not received.
 *
 * @param base pointer to FLEXIO_CAMERA_Type.
 * @param handle Pointer to flexio_camera_edma_handle_t structure.
 * @return The number of bytes still not received.
 */
size_t FLEXIO_CAMERA_GetReceiveRemainingBytesEDMA(FLEXIO_CAMERA_Type *base, flexio_camera_edma_handle_t *handle);

/*@}*/

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* _FSL_CAMERA_EDMA_H_ */
