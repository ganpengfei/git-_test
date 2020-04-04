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
#ifndef __FSL_LPUART_RTOS_H__
#define __FSL_LPUART_RTOS_H__

#include "fsl_lpuart.h"
#include <FreeRTOS.h>
#include <event_groups.h>
#include <semphr.h>

/*!
 * @addtogroup uart
 * @{
 */

/*! @file*/

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief LPUART FreeRTOS driver version */
#define FSL_LPUART_FREERTOS_DRIVER_VERSION (MAKE_VERSION(2, 0, 0)) /*!< Version 2.0.0. */

struct rtos_lpuart_config
{
    LPUART_Type *base;
    uint32_t srcclk;
    uint32_t baudrate;
    lpuart_parity_mode_t parity;
    lpuart_stop_bit_count_t stopbits;
    uint8_t *buffer;
    uint32_t buffer_size;
};

typedef struct _lpuart_rtos_handle
{
    LPUART_Type *base;
    struct _lpuart_transfer tx_xfer;
    struct _lpuart_transfer rx_xfer;
    SemaphoreHandle_t rx_sem;
    SemaphoreHandle_t tx_sem;
#define RTOS_UART_COMPLETE 0x1
    EventGroupHandle_t rx_event;
    EventGroupHandle_t tx_event;
    void *t_state; /* transactional layer state */
} lpuart_rtos_handle_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name LPUART RTOS Operation
 * @{
 */

/*!
 * @brief Initializes an LPUART instance for operation in RTOS. It operates on the
 * background ring buffer that is allocated by caller, together with transactional handle
 * and the RTOS handle.
 * Example below shows how to fill the configuration structure for the LPUART.
 * @code
 *  rtos_lpuart_config_t lpuartConfig;
 *  uint8_t buffer[16];
 *  lpuartConfig.base = LPUART1;
 *  lpuartConfig.srcclk = 2000000u;
 *  lpuartConfig.baudrate = 115200u;
 *  lpuartConfig.parity = kLPUART_ParityDisabled;
 *  lpuartConfig.stopbits = kLPUART_OneStopBit;
 *  lpuartConfig.buffer = buffer;
 *  lpuartConfig.buffer_size = sizeof(buffer);
 * @endcode
 *
 * @param handle The RTOS LPUART handle, the pointer to allocated space for RTOS context.
 * @param t_handle The pointer to allocated space where to store transactional layer internal state.
 * @param cfg The pointer to the parameters required to configure the LPUART after initialization.
 * @return 0 succeed, others failed
 */
int LPUART_RTOS_Init(lpuart_rtos_handle_t *handle, lpuart_handle_t *t_handle, const struct rtos_lpuart_config *cfg);

/*!
 * @brief Deinitializes an LPUART instance for operation.
 *
 * This function deinitializes the LPUART module, set all register value to reset value
 * and releases the resources.
 *
 * @param handle The RTOS LPUART handle.
 */
int LPUART_RTOS_DeInit(lpuart_rtos_handle_t *handle);

/*!
 * @brief Send data to the initialized instance of LPUART.
 *
 * This function sends data. It is synchronous API. The function blocks until the length number of data is sent
 * to the IP.
 *
 * @param handle The RTOS LPUART handle.
 * @param buffer The pointer to buffer to send.
 * @param length The number of bytes to send.
 */
int LPUART_RTOS_Send(lpuart_rtos_handle_t *handle, const uint8_t *buffer, uint32_t length);

/*!
 * @brief Receives data. It is synchronous API.
 *
 * This function receives data from LPUART. The function blocks until the length number of chars are 
 * received into buffer.
 *
 * @param handle The RTOS LPUART handle.
 * @param buffer The pointer to buffer where to write received data.
 * @param length The number of bytes to receive.
 * @param received The pointer to variable of size_t where the number of received data will be filled.
 */
int LPUART_RTOS_Receive(lpuart_rtos_handle_t *handle, uint8_t *buffer, uint32_t length, size_t *received);

/* @} */

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* __FSL_LPUART_RTOS_H__ */
