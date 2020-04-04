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

#include "fsl_uart_edma.h"
#include "fsl_dmamux.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*<! Structure definition for uart_edma_private_handle_t. The structure is private. */
typedef struct _uart_edma_private_handle
{
    UART_Type *base;
    uart_edma_handle_t *handle;
} uart_edma_private_handle_t;

/* UART EDMA transfer handle. */
enum _uart_edma_tansfer_states
{
    kUART_TxIdle, /* TX idle. */
    kUART_TxBusy, /* TX busy. */
    kUART_RxIdle, /* RX idle. */
    kUART_RxBusy  /* RX busy. */
};

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*<! Private handle only used for internally. */
uart_edma_private_handle_t s_edmaPrivateHandle[FSL_FEATURE_SOC_UART_COUNT];

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*!
 * @brief UART EDMA send finished callback function.
 *
 * This function is called when UART EDMA send finished. It disables the UART
 * TX EDMA request and sends @ref kStatus_UART_TxIdle to UART callback.
 *
 * @param handle The EDMA handle.
 * @param param Callback function parameter.
 */
static void UART_SendEDMACallback(edma_handle_t *handle, void *param, bool transferDone, uint32_t tcds);

/*!
 * @brief UART EDMA receive finished callback function.
 *
 * This function is called when UART EDMA receive finished. It disables the UART
 * RX EDMA request and sends @ref kStatus_UART_RxIdle to UART callback.
 *
 * @param handle The EDMA handle.
 * @param param Callback function parameter.
 */
static void UART_ReceiveEDMACallback(edma_handle_t *handle, void *param, bool transferDone, uint32_t tcds);

/*!
 * @brief Get the UART instance from peripheral base address.
 *
 * @param base UART peripheral base address.
 * @return UART instance.
 */
extern uint32_t UART_GetInstance(UART_Type *base);

/*******************************************************************************
 * Code
 ******************************************************************************/

static void UART_SendEDMACallback(edma_handle_t *handle, void *param, bool transferDone, uint32_t tcds)
{
    uart_edma_private_handle_t *uartPrivateHandle = (uart_edma_private_handle_t *)param;

    /* Avoid the warning for unused variables. */
    handle = handle;
    tcds = tcds;

    if (transferDone)
    {
        UART_AbortSendEDMA(uartPrivateHandle->base, uartPrivateHandle->handle);

        if (uartPrivateHandle->handle->callback)
        {
            uartPrivateHandle->handle->callback(uartPrivateHandle->base, uartPrivateHandle->handle, kStatus_UART_TxIdle,
                                                uartPrivateHandle->handle->userData);
        }
    }
}

static void UART_ReceiveEDMACallback(edma_handle_t *handle, void *param, bool transferDone, uint32_t tcds)
{
    uart_edma_private_handle_t *uartPrivateHandle = (uart_edma_private_handle_t *)param;

    /* Avoid warning for unused parameters. */
    handle = handle;
    tcds = tcds;

    if (transferDone)
    {
        /* Disable transfer. */
        UART_AbortReceiveEDMA(uartPrivateHandle->base, uartPrivateHandle->handle);

        if (uartPrivateHandle->handle->callback)
        {
            uartPrivateHandle->handle->callback(uartPrivateHandle->base, uartPrivateHandle->handle, kStatus_UART_RxIdle,
                                                uartPrivateHandle->handle->userData);
        }
    }
}

void UART_CreateHandleEDMA(UART_Type *base,
                           uart_edma_handle_t *handle,
                           uart_edma_transfer_callback_t callback,
                           void *userData,
                           edma_handle_t *txEdmaHandle,
                           edma_handle_t *rxEdmaHandle)
{
    assert(handle);

    uint32_t instance = UART_GetInstance(base);

    s_edmaPrivateHandle[instance].base = base;
    s_edmaPrivateHandle[instance].handle = handle;

    memset(handle, 0, sizeof(*handle));

    handle->rxState = kUART_RxIdle;
    handle->txState = kUART_TxIdle;

    handle->rxEdmaHandle = rxEdmaHandle;
    handle->txEdmaHandle = txEdmaHandle;

    handle->callback = callback;
    handle->userData = userData;

#if defined(FSL_FEATURE_UART_HAS_FIFO) && FSL_FEATURE_UART_HAS_FIFO
    /* Note:
       Take care of the RX FIFO, EDMA request only assert when received bytes
       equal or more than RX water mark, there is potential issue if RX water
       mark larger than 1.
       For example, if RX FIFO water mark is 2, upper layer needs 5 bytes and
       5 bytes are received. the last byte will be saved in FIFO but not trigger
       EDMA transfer because the water mark is 2.
     */
    if (rxEdmaHandle)
    {
        base->RWFIFO = 1U;
    }
#endif

    /* Configure TX. */
    if (txEdmaHandle)
    {
        EDMA_SetCallback(handle->txEdmaHandle, UART_SendEDMACallback, &s_edmaPrivateHandle[instance]);
    }

    /* Configure RX. */
    if (rxEdmaHandle)
    {
        EDMA_SetCallback(handle->rxEdmaHandle, UART_ReceiveEDMACallback, &s_edmaPrivateHandle[instance]);
    }
}

status_t UART_SendEDMA(UART_Type *base, uart_edma_handle_t *handle, uart_transfer_t *xfer)
{
    assert(handle->txEdmaHandle);

    edma_transfer_config_t xferConfig;
    status_t status;

    /* If previous TX not finished. */
    if (kUART_TxBusy == handle->txState)
    {
        status = kStatus_UART_TxBusy;
    }
    else
    {
        handle->txState = kUART_TxBusy;

        /* Prepare transfer. */
        EDMA_PrepareTransfer(&xferConfig, xfer->data, sizeof(uint8_t), (void *)UART_GetDataRegisterAddress(base),
                             sizeof(uint8_t), sizeof(uint8_t), xfer->dataSize, kEDMA_MemoryToPeripheral);

        /* Submit transfer. */
        EDMA_SubmitTransfer(handle->txEdmaHandle, &xferConfig);
        EDMA_StartTransfer(handle->txEdmaHandle);

        /* Enable UART TX EDMA. */
        UART_EnableTxDMA(base, true);

        status = kStatus_Success;
    }

    return status;
}

status_t UART_ReceiveEDMA(UART_Type *base, uart_edma_handle_t *handle, uart_transfer_t *xfer)
{
    assert(handle->rxEdmaHandle);

    edma_transfer_config_t xferConfig;
    status_t status;

    /* If previous RX not finished. */
    if (kUART_RxBusy == handle->rxState)
    {
        status = kStatus_UART_RxBusy;
    }
    else
    {
        handle->rxState = kUART_RxBusy;

        /* Prepare transfer. */
        EDMA_PrepareTransfer(&xferConfig, (void *)UART_GetDataRegisterAddress(base), sizeof(uint8_t), xfer->data,
                             sizeof(uint8_t), sizeof(uint8_t), xfer->dataSize, kEDMA_PeripheralToMemory);

        /* Submit transfer. */
        EDMA_SubmitTransfer(handle->rxEdmaHandle, &xferConfig);
        EDMA_StartTransfer(handle->rxEdmaHandle);

        /* Enable UART RX EDMA. */
        UART_EnableRxDMA(base, true);

        status = kStatus_Success;
    }

    return status;
}

void UART_AbortSendEDMA(UART_Type *base, uart_edma_handle_t *handle)
{
    assert(handle->txEdmaHandle);

    /* Disable UART TX EDMA. */
    UART_EnableTxDMA(base, false);

    /* Stop transfer. */
    EDMA_AbortTransfer(handle->txEdmaHandle);

    handle->txState = kUART_TxIdle;
}

void UART_AbortReceiveEDMA(UART_Type *base, uart_edma_handle_t *handle)
{
    assert(handle->rxEdmaHandle);

    /* Disable UART RX EDMA. */
    UART_EnableRxDMA(base, false);

    /* Stop transfer. */
    EDMA_AbortTransfer(handle->rxEdmaHandle);

    handle->rxState = kUART_RxIdle;
}

size_t UART_GetReceiveRemainingBytesEDMA(UART_Type *base, uart_edma_handle_t *handle)
{
    assert(handle->rxEdmaHandle);

    size_t bytes;

    if (kUART_RxBusy == handle->rxState)
    {
        bytes = EDMA_GetRemainingBytes(handle->rxEdmaHandle->base, handle->rxEdmaHandle->channel);
    }
    else
    {
        bytes = 0;
    }

    return bytes;
}

size_t UART_GetSendRemainingBytesEDMA(UART_Type *base, uart_edma_handle_t *handle)
{
    assert(handle->txEdmaHandle);

    size_t bytes;

    if (kUART_TxBusy == handle->txState)
    {
        bytes = EDMA_GetRemainingBytes(handle->txEdmaHandle->base, handle->txEdmaHandle->channel);
    }
    else
    {
        bytes = 0;
    }

    return bytes;
}
