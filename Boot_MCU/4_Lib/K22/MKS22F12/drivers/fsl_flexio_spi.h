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

#ifndef _FSL_FLEXIO_SPI_H_
#define _FSL_FLEXIO_SPI_H_

#include "fsl_common.h"
#include "fsl_flexio.h"

/*!
 * @addtogroup flexio_spi
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief FLEXIO SPI driver version */
#define FSL_FLEXIO_SPI_DRIVER_VERSION (MAKE_VERSION(2, 0, 0)) /*!< Version 2.0.0. */

/*! @brief FLEXIO SPI dummy transfer data, the data is sent while txData is NULL. */
#define FLEXIO_SPI_DUMMYDATA (0xFFFFU)

/*! @brief Error codes for the FLEXIO SPI driver. */
enum _flexio_spi_status
{
    kStatus_FLEXIO_SPI_Busy = MAKE_STATUS(kStatusGroup_FLEXIO_SPI, 1),  /*!< FLEXIO SPI is busy. */
    kStatus_FLEXIO_SPI_Idle = MAKE_STATUS(kStatusGroup_FLEXIO_SPI, 2),  /*!< SPI is idle */
    kStatus_FLEXIO_SPI_Error = MAKE_STATUS(kStatusGroup_FLEXIO_SPI, 3), /*!< FLEXIO SPI error. */
};

/*! @brief FLEXIO SPI clock phase configuration. */
typedef enum _flexio_spi_clock_phase
{
    kFLEXIO_SPI_ClockPhaseFirstEdge = 0x0U,  /*!< First edge on SPSCK occurs at the middle of the first
                                   *   cycle of a data transfer. */
    kFLEXIO_SPI_ClockPhaseSecondEdge = 0x1U, /*!< First edge on SPSCK occurs at the start of the
                                   *   first cycle of a data transfer. */
} flexio_spi_clock_phase_t;

/*! @brief FLEXIO SPI data shifter direction options. */
typedef enum _flexio_spi_shift_direction
{
    kFLEXIO_SPI_MsbFirst = 0, /*!< Data transfers start with most significant bit. */
    kFLEXIO_SPI_LsbFirst = 1, /*!< Data transfers start with least significant bit. */
} flexio_spi_shift_direction_t;

/*! @brief FLEXIO SPI data length mode options. */
typedef enum _flexio_spi_data_bitcount_mode
{
    kFLEXIO_SPI_8BitMode = 0x08U,  /*!< 8-bit data transmission mode. */
    kFLEXIO_SPI_16BitMode = 0x10U, /*!< 16-bit data transmission mode. */
} flexio_spi_data_bitcount_mode_t;

/*! @brief Define FLEXIO SPI interrupt mask. */
enum _flexio_spi_interrupt_enable
{
    kFLEXIO_SPI_TxEmptyInterruptEnable = 0x1U, /*!< Transmit buffer empty interrupt enable. */
    kFLEXIO_SPI_RxFullInterruptEnable = 0x2U,  /*!< Receive buffer full interrupt enable. */
};

/*! @brief Define FLEXIO SPI status mask. */
enum _flexio_spi_status_flags
{
    kFLEXIO_SPI_TxBufferEmptyFlag = 0x1U, /*!< Transmit buffer empty flag. */
    kFLEXIO_SPI_RxBufferFullFlag = 0x2U,  /*!< Receive buffer full flag. */
};

enum _flexio_spi_dma_enable
{
    kFLEXIO_SPI_TxDmaEnable = 0x1U,  /*!< Tx dma request source */
    kFLEXIO_SPI_RxDmaEnable = 0x2U,  /*!< Rx dma request source */
    kFLEXIO_SPI_DmaAllEnable = 0x3U, /*!< All dma request source*/
};

enum _flexio_spi_transfer_flags
{
    kFLEXIO_SPI_8bitMsb = 0x1U,  /*!< FLEXIO SPI 8-bit MSB first */
    kFLEXIO_SPI_8bitLsb = 0x2U,  /*!< FLEXIO SPI 8-bit LSB first */
    kFLEXIO_SPI_16bitMsb = 0x9U, /*!< FLEXIO SPI 16-bit MSB first */
    kFLEXIO_SPI_16bitLsb = 0xaU, /*!< FLEXIO SPI 16-bit LSB first */
};

/*! @brief Define FLEXIO SPI access structure typedef. */
typedef struct _flexio_spi_type
{
    FLEXIO_Type *flexioBase; /*!< FLEXIO base pointer. */
    uint8_t SDOPinIndex;     /*!< Pin select for data output. */
    uint8_t SDIPinIndex;     /*!< Pin select for data input. */
    uint8_t SCKPinIndex;     /*!< Pin select for clock. */
    uint8_t CSnPinIndex;     /*!< Pin select for enable. */
    uint8_t shifterIndex[2]; /*!< Shifter index used in FLEXIO SPI. */
    uint8_t timerIndex[2];   /*!< Timer index used in FLEXIO SPI. */
} FLEXIO_SPI_Type;

/*! @brief Define FLEXIO SPI master configuration structure. */
typedef struct _flexio_spi_master_config
{
    bool enableMaster;                        /*!< Enable/disable FLEXIO SPI master after configuration. */
    bool enableInDoze;                        /*!< Enable/disable FLEXIO operation in doze mode. */
    bool enableInDebug;                       /*!< Enable/disable FLEXIO operation in debug mode. */
    bool enableFastAccess;                    /*!< Enable/disable fast access to FLEXIO registers,
                                              fast access requires the FLEXIO clock to be at least
                                              twice the frequency of the bus clock. */
    uint32_t baudRate_Bps;                    /*!< Baud rate in Bps. */
    flexio_spi_clock_phase_t phase;           /*!< Clock phase. */
    flexio_spi_data_bitcount_mode_t dataMode; /*!< 8bit or 16bit mode. */
} flexio_spi_master_config_t;

/*! @brief Define FLEXIO SPI slave configuration structure. */
typedef struct _flexio_spi_slave_config
{
    bool enableSlave;                         /*!< Enable/disable FLEXIO SPI slave after configuration. */
    bool enableInDoze;                        /*!< Enable/disable FLEXIO operation in doze mode. */
    bool enableInDebug;                       /*!< Enable/disable FLEXIO operation in debug mode. */
    bool enableFastAccess;                    /*!< Enable/disable fast access to FLEXIO registers,
                                              fast access requires the FLEXIO clock to be at least
                                              twice the frequency of the bus clock. */
    flexio_spi_clock_phase_t phase;           /*!< Clock phase. */
    flexio_spi_data_bitcount_mode_t dataMode; /*!< 8bit or 16bit mode. */
} flexio_spi_slave_config_t;

/*! @brief Define FLEXIO SPI transfer structure. */
typedef struct _flexio_spi_transfer
{
    uint8_t *txData; /*!< Send buffer. */
    uint8_t *rxData; /*!< Receive buffer. */
    size_t dataSize; /*!< Transfer bytes. */
    uint8_t flags;   /*!< FLEXIO SPI control flag, Msb first  or Lsb first. */
} flexio_spi_transfer_t;

/*! @brief  typedef for flexio_spi_master_handle_t in advance. */
typedef struct _flexio_spi_master_handle flexio_spi_master_handle_t;

/*! @brief  Slave handle is the same with master handle. */
typedef flexio_spi_master_handle_t flexio_spi_slave_handle_t;

/*! @brief FLEXIO SPI master callback for finished transmit */
typedef void (*flexio_spi_master_transfer_callback_t)(FLEXIO_SPI_Type *base,
                                                      flexio_spi_master_handle_t *handle,
                                                      status_t status,
                                                      void *userData);

/*! @brief FLEXIO SPI slave callback for finished transmit */
typedef void (*flexio_spi_slave_transfer_callback_t)(FLEXIO_SPI_Type *base,
                                                     flexio_spi_slave_handle_t *handle,
                                                     status_t status,
                                                     void *userData);

/*! @brief Define FLEXIO SPI handle structure. */
struct _flexio_spi_master_handle
{
    uint8_t *txData;                                /*!< Transfer buffer. */
    uint8_t *rxData;                                /*!< Receive buffer. */
    volatile size_t txRemainingBytes;               /*!< Send data remaining in bytes. */
    volatile size_t rxRemainingBytes;               /*!< Receive data remaining in bytes. */
    volatile uint32_t state;                        /*!< FLEXIO SPI interanl state. */
    uint8_t bytePerFrame;                           /*!< SPI mode, 2bytes or 1byte in a frame */
    flexio_spi_shift_direction_t direction;         /*!< Shift direction. */
    flexio_spi_master_transfer_callback_t callback; /*!< FLEXIO SPI callback. */
    void *userData;                                 /*!< Callback parameter. */
};

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /*_cplusplus*/

/*!
 * @name FLEXIO SPI Configuration
 * @{
 */

/*!
 * @brief Ungate the FLEXIO clock, reset the FLEXIO module and do FLEXIO SPI master hardware
 * configuration, then configure the FLEXIO SPI with flexio spi master configuration. The
 * configuration structure can be filled by user from scratch, or be set with default values
 * by FLEXIO_SPI_MasterGetDefaultConfig().
 *
 * @note FLEXIO SPI master only support CPOL = 0, which means clock inactive low.
 *
 * Example
   @code
   FLEXIO_SPI_Type spiDev = {
   .flexioBase = FLEXIO,
   .SDOPinIndex = 0,
   .SDIPinIndex = 1,
   .SCKPinIndex = 2,
   .CSnPinIndex = 3,
   .shifterIndex = {0,1},
   .timerIndex = {0,1}
   };
   flexio_spi_master_config_t config = {
   .enableMaster = true,
   .enableInDoze = false,
   .enableInDebug = true,
   .enableFastAccess = false,
   .baudRate_Bps = 500000,
   .phase = kFLEXIO_SPI_ClockPhaseFirstEdge,
   .direction = kFLEXIO_SPI_MsbFirst,
   .dataMode = kFLEXIO_SPI_8BitMode
   };
   FLEXIO_SPI_MasterInit(&spiDev, &config, srcClock_Hz);
   @endcode
 *
 * @param base pointer to FLEXIO_SPI_Type structure
 * @param masterConfig pointer to flexio_spi_master_config_t structure
 * @param srcClock_Hz flexio source clock in HZ
*/
void FLEXIO_SPI_MasterInit(FLEXIO_SPI_Type *base, flexio_spi_master_config_t *masterConfig, uint32_t srcClock_Hz);

/*!
 * @brief Gate the FLEXIO clock.
 *
 * @param base pointer to FLEXIO_SPI_Type
*/
void FLEXIO_SPI_MasterDeInit(FLEXIO_SPI_Type *base);

/*!
 * @brief Get the default configuration to configure FLEXIO SPI master. The configuration
 * could be used directly for calling FLEXIO_SPI_MasterConfigure().
 * Example:
   @code
   flexio_spi_master_config_t masterConfig;
   FLEXIO_SPI_MasterGetDefaultConfig(&masterConfig);
   @endcode
 * @param masterConfig pointer to flexio_spi_master_config_t structure
*/
void FLEXIO_SPI_MasterGetDefaultConfig(flexio_spi_master_config_t *masterConfig);

/*!
 * @brief Ungate the FLEXIO clock, reset the FLEXIO module and do FLEXIO SPI slave hardware
 * configuration. Then Configure the FLEXIO SPI with flexio spi slave configuration. The
 * configuration structure can be filled by user from scratch, or be set with default values
 * by FLEXIO_SPI_SlaveGetDefaultConfig().
 *
 * @note Only one timer is needed in FLEXIO SPI slave, so the second timer Index is ignored.
 * FLEXIO SPI slave only support CPOL = 0, which means clock inactive low.
 * Example
   @code
   FLEXIO_SPI_Type spiDev = {
   .flexioBase = FLEXIO,
   .SDOPinIndex = 0,
   .SDIPinIndex = 1,
   .SCKPinIndex = 2,
   .CSnPinIndex = 3,
   .shifterIndex = {0,1},
   .timerIndex = {0}
   };
   flexio_spi_slave_config_t config = {
   .enableSlave = true,
   .enableInDoze = false,
   .enableInDebug = true,
   .enableFastAccess = false,
   .phase = kFLEXIO_SPI_ClockPhaseFirstEdge,
   .direction = kFLEXIO_SPI_MsbFirst,
   .dataMode = kFLEXIO_SPI_8BitMode
   };
   FLEXIO_SPI_SlaveInit(&spiDev, &config);
   @endcode
 * @param base pointer to FLEXIO_SPI_Type structure
 * @param slaveConfig pointer to flexio_spi_slave_config_t structure
*/
void FLEXIO_SPI_SlaveInit(FLEXIO_SPI_Type *base, flexio_spi_slave_config_t *slaveConfig);

/*!
 * @brief Gate the FLEXIO clock.
 *
 * @param base pointer to FLEXIO_SPI_Type
*/
void FLEXIO_SPI_SlaveDeInit(FLEXIO_SPI_Type *base);

/*!
 * @brief Get the default configuration to configure FLEXIO SPI slave. The configuration
 * could be used directly for calling FLEXIO_SPI_SlaveConfigure().
 * Example:
   @code
   flexio_spi_slave_config_t slaveConfig;
   FLEXIO_SPI_SlaveGetDefaultConfig(&slaveConfig);
   @endcode
 * @param slaveConfig pointer to flexio_spi_slave_config_t structure
*/
void FLEXIO_SPI_SlaveGetDefaultConfig(flexio_spi_slave_config_t *slaveConfig);

/*@}*/

/*!
 * @name Status
 * @{
 */

/*!
 * @brief Get FLEXIO SPI status flags.
 *
 * @param base pointer to FLEXIO_SPI_Type structure
 * @return status flag, use status flag to AND the following flag mask to get the according status.
 *          @arg kFLEXIO_SPI_TxEmptyFlag
 *          @arg kFLEXIO_SPI_RxEmptyFlag
*/

uint32_t FLEXIO_SPI_GetStatusFlags(FLEXIO_SPI_Type *base);

/*!
 * @brief Get FLEXIO SPI status flags.
 *
 * @param base pointer to FLEXIO_SPI_Type structure
 * @param mask status flag
 *      The parameter could be any combination of the following values:
 *          @arg kFLEXIO_SPI_TxEmptyFlag
 *          @arg kFLEXIO_SPI_RxEmptyFlag
*/

void FLEXIO_SPI_ClearStatusFlags(FLEXIO_SPI_Type *base, uint32_t mask);

/*@}*/

/*!
 * @name Interrupts
 * @{
 */

/*!
 * @brief Enable flexio spi interrupt.
 *
 * This function enable the flexio spi interrupt
 *
 * @param base pointer to FLEXIO_SPI_Type structure
 * @param mask interrupt source. The parameter could be any combination of the following values:
 *        @arg kFLEXIO_SPI_RxFullInterruptEnable
 *        @arg kFLEXIO_SPI_TxEmptyInterruptEnable
 */
void FLEXIO_SPI_EnableInterrupts(FLEXIO_SPI_Type *base, uint32_t mask);

/*!
 * @brief Disable flexio spi interrupt.
 *
 * This function disable the flexio spi interrupt
 *
 * @param base pointer to FLEXIO_SPI_Type structure
 * @param mask interrupt source The parameter could be any combination of the following values:
 *        @arg kFLEXIO_SPI_RxFullInterruptEnable
 *        @arg kFLEXIO_SPI_TxEmptyInterruptEnable
 */
void FLEXIO_SPI_DisableInterrupts(FLEXIO_SPI_Type *base, uint32_t mask);

/*@}*/

/*!
 * @name DMA Control
 * @{
 */

/*!
 * @brief Enable/Disable flexio spi transmit DMA. This function enables/disables the flexio spi Tx DMA,
 * which means assert kFLEXIO_SPI_TxEmptyFlag will/won't trigger DMA request.
 *
 * @param base pointer to FLEXIO_SPI_Type structure
 * @param mask SPI dma source.
 * @param enable True means enable DMA, false means disable DMA
 */
void FLEXIO_SPI_EnableDMA(FLEXIO_SPI_Type *base, uint32_t mask, bool enable);

/*!
 * @brief get flexio spi transmit data register address for MSB first transfer
 *
 * This function return the spi data register address, mainly used by DMA/eDMA
 *
 * @param base pointer to FLEXIO_SPI_Type structure
 * @param direction shift direction of MSB first or LSB first
 * @return flexio spi transmit data register address.
 */
static inline uint32_t FLEXIO_SPI_GetTxDataRegisterAddress(FLEXIO_SPI_Type *base,
                                                           flexio_spi_shift_direction_t direction)
{
    if (direction == kFLEXIO_SPI_MsbFirst)
    {
        return FLEXIO_GetShifterBufferAddress(base->flexioBase, kFLEXIO_ShifterBufferBitByteSwapped,
                                              base->shifterIndex[0]);
    }
    else
    {
        return FLEXIO_GetShifterBufferAddress(base->flexioBase, kFLEXIO_ShifterBuffer, base->shifterIndex[0]);
    }
}

/*!
 * @brief get flexio spi receive data register address for MSB first transfer
 *
 * This function return the spi data register address, mainly used by DMA/eDMA
 *
 * @param base pointer to FLEXIO_SPI_Type structure
 * @param direction shift direction of MSB first or LSB first
 * @return flexio spi receive data register address.
 */
static inline uint32_t FLEXIO_SPI_GetRxDataRegisterAddress(FLEXIO_SPI_Type *base,
                                                           flexio_spi_shift_direction_t direction)
{
    if (direction == kFLEXIO_SPI_MsbFirst)
    {
        return FLEXIO_GetShifterBufferAddress(base->flexioBase, kFLEXIO_ShifterBufferBitSwapped, base->shifterIndex[1]);
    }
    else
    {
        return FLEXIO_GetShifterBufferAddress(base->flexioBase, kFLEXIO_ShifterBufferByteSwapped,
                                              base->shifterIndex[1]);
    }
}

/*@}*/

/*!
 * @name Bus Operations
 * @{
 */

/*!
 * @brief Enable/Disable the FLEXIO SPI module operation.
 *
 * @param base pointer to FLEXIO_SPI_Type
 * @param enable True to enable, false to disable.
*/
static inline void FLEXIO_SPI_Enable(FLEXIO_SPI_Type *base, bool enable)
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
 * @brief Set Baud rate for FLEXIO SPI transfer, this only used in master.
 *
 * @param base pointer to FLEXIO_SPI_Type structure
 * @param baudRate_Bps Buad Rate needed in Hz.
 * @param srcClockHz SPI source clock frequency in Hz.
 */
void FLEXIO_SPI_MasterSetBaudRate(FLEXIO_SPI_Type *base, uint32_t baudRate_Bps, uint32_t srcClockHz);

/*!
 * @brief Writes one byte of data and the data is sent in Msb way.
 *
 * @note This is a non-blocking API and will return directly after the data is put into the
 * data register but not data transfer finished on the bus. User needs to make sure that
 * the TxEmptyFlag is asserted before calling this API.
 *
 * @param base pointer to FLEXIO_SPI_Type structure
 * @param direction shift direction of MSB first or LSB first
 * @param data 8bit/16bit data
 */
static inline void FLEXIO_SPI_WriteData(FLEXIO_SPI_Type *base,
                                               flexio_spi_shift_direction_t direction,
                                               uint16_t data)
{
    if (direction == kFLEXIO_SPI_MsbFirst)
    {
        base->flexioBase->SHIFTBUFBBS[base->shifterIndex[0]] = data;
    }
    else
    {
        base->flexioBase->SHIFTBUF[base->shifterIndex[0]] = data;
    }
}

/*!
 * @brief Reads 8bit/16bit data.
 *
 * @note This is a non-blocking API and will return directly after the data is read from the
 * data register. User needs to make sure the RxFullFlag is asserted before calling this API.
 *
 * @param base pointer to FLEXIO_SPI_Type structure
 * @param direction shift direction of MSB first or LSB first
 * @return 8bit/16bit data received
 */
static inline uint16_t FLEXIO_SPI_ReadData(FLEXIO_SPI_Type *base, flexio_spi_shift_direction_t direction)
{
    if (direction == kFLEXIO_SPI_MsbFirst)
    {
        return base->flexioBase->SHIFTBUFBIS[base->shifterIndex[1]];
    }
    else
    {
        return base->flexioBase->SHIFTBUFBYS[base->shifterIndex[1]];
    }
}

/*!
 * @brief sends a buffer of data bytes.
 *
 * @note This function blocks via polling until all bytes have been sent.
 *
 * @param base pointer to FLEXIO_SPI_Type structure
 * @param direction shift direction of MSB first or LSB first
 * @param buffer The data bytes to send
 * @param size The number of data bytes to send
 * @return transfer status, succeed return kStatus_Success
 */
void FLEXIO_SPI_WriteBlocking(FLEXIO_SPI_Type *base,
                              flexio_spi_shift_direction_t direction,
                              const uint8_t *buffer,
                              size_t size);

/*!
 * @brief Receives a buffer of bytes.
 *
 * @note This function blocks via polling until all bytes have been received.
 *
 * @param base pointer to FLEXIO_SPI_Type structure
 * @param direction shift direction of MSB first or LSB first
 * @param buffer The buffer to store the received bytes
 * @param size The number of data bytes to be received
 * @param direction shift direction of MSB first or LSB first
 */
void FLEXIO_SPI_ReadBlocking(FLEXIO_SPI_Type *base,
                             flexio_spi_shift_direction_t direction,
                             uint8_t *buffer,
                             size_t size);

/*Transactional APIs*/

/*!
 * @name Transactional
 * @{
 */

/*!
 * @brief Init the FLEXIO SPI Master handle which is used in transcational functions
 *
 * @param base pointer to FLEXIO_SPI_Type structure
 * @param handle pointer to flexio_spi_master_handle_t structure to store the transfer state.
 * @param callback The callback function.
 * @param userData The parameter of the callback function.
 * @retval kStatus_Success Successfully create the handle.
 * @retval kStatus_OutOfRange The flexio type/handle/isr table out of range.
 */
status_t FLEXIO_SPI_MasterCreateHandle(FLEXIO_SPI_Type *base,
                                   flexio_spi_master_handle_t *handle,
                                   flexio_spi_master_transfer_callback_t callback,
                                   void *userData);

/*!
 * @brief Master transfer data using IRQ
 *
 * This function send data using IRQ, this is non-blocking function, will return
 * right away, when all data have been sent out/received, the callback function will be called.
 *
 * @param base pointer to FLEXIO_SPI_Type structure
 * @param handle pointer to flexio_spi_master_handle_t structure to store the transfer state.
 * @param xfer flexio spi transfer sturcture, refer to #flexio_spi_transfer_t.
 * @retval kStatus_Success Successfully start a transfer.
 * @retval kStatus_InvalidArgument Input argument is invalid.
 * @retval kStatus_FLEXIO_SPI_Busy SPI is not idle, is running another transfer.
 */
status_t FLEXIO_SPI_MasterTransferNonBlocking(FLEXIO_SPI_Type *base,
                                              flexio_spi_master_handle_t *handle,
                                              flexio_spi_transfer_t *xfer);

/*!
 * @brief Abort the master data transfer which using IRQ
 *
 * @param base pointer to FLEXIO_SPI_Type structure
 * @param handle pointer to flexio_spi_master_handle_t structure to store the transfer state.
 * @param remainingBytes Remaining bytes need to transfer. This pointer can be NULL is user do not need it.
 */
void FLEXIO_SPI_MasterAbortTransfer(FLEXIO_SPI_Type *base, flexio_spi_master_handle_t *handle);

/*!
 * @brief Get the data transfer status which using IRQ
 *
 * @param base pointer to FLEXIO_SPI_Type structure
 * @param handle pointer to flexio_spi_master_handle_t structure to store the transfer state.
 * @return Remaining bytes need to be transferred.
 */
size_t FLEXIO_SPI_MasterGetTransferRemainingBytes(FLEXIO_SPI_Type *base, flexio_spi_master_handle_t *handle);

/*!
 * @brief FLEXIO SPI master IRQ handler function
 *
 * @param spiType pointer to FLEXIO_SPI_Type structure
 * @param spiHandle pointer to flexio_spi_master_handle_t structure to store the transfer state.
 */
void FLEXIO_SPI_MasterHandleIRQ(void *spiType, void *spiHandle);

/*!
 * @brief Init the FLEXIO SPI Slave handle which is used in transcational functions
 *
 * @param base pointer to FLEXIO_SPI_Type structure
 * @param handle pointer to flexio_spi_slave_handle_t structure to store the transfer state.
 * @param callback The callback function.
 * @param userData The parameter of the callback function.
 */
static inline void FLEXIO_SPI_SlaveCreateHandle(FLEXIO_SPI_Type *base,
                                                flexio_spi_slave_handle_t *handle,
                                                flexio_spi_slave_transfer_callback_t callback,
                                                void *userData)
{
    FLEXIO_SPI_MasterCreateHandle(base, handle, callback, userData);
}

/*!
 * @brief Slave transfer data using IRQ
 *
 * This function send data using IRQ, this is non-blocking function, will return
 * right away, when all data have been sent out/received, the callback function will be called.
 * @param handle pointer to flexio_spi_slave_handle_t structure to store the transfer state.
 *
 * @param base pointer to FLEXIO_SPI_Type structure
 * @param xfer flexio spi transfer sturcture, refer to #flexio_spi_transfer_t.
 * @retval kStatus_Success Successfully start a transfer.
 * @retval kStatus_InvalidArgument Input argument is invalid.
 * @retval kStatus_FLEXIO_SPI_Busy SPI is not idle, is running another transfer.
 */
static inline status_t FLEXIO_SPI_SlaveTransferNonBlocking(FLEXIO_SPI_Type *base,
                                                           flexio_spi_slave_handle_t *handle,
                                                           flexio_spi_transfer_t *xfer)
{
    return FLEXIO_SPI_MasterTransferNonBlocking(base, handle, xfer);
}
/*!
 * @brief Abort the slave data transfer which using IRQ
 *
 * @param base pointer to FLEXIO_SPI_Type structure
 * @param handle pointer to flexio_spi_slave_handle_t structure to store the transfer state.
 */
static inline void FLEXIO_SPI_SlaveAbortTransfer(FLEXIO_SPI_Type *base, flexio_spi_slave_handle_t *handle)
{
    FLEXIO_SPI_MasterAbortTransfer(base, handle);
}
/*!
 * @brief Get the data transfer status which using IRQ
 *
 * @param base pointer to FLEXIO_SPI_Type structure
 * @param handle pointer to flexio_spi_slave_handle_t structure to store the transfer state.
 * @return Remaining bytes need to be transferred.
 */
static inline size_t FLEXIO_SPI_SlaveGetTransferRemainingBytes(FLEXIO_SPI_Type *base, flexio_spi_slave_handle_t *handle)
{
    return FLEXIO_SPI_MasterGetTransferRemainingBytes(base, handle);
}

/*!
 * @brief FLEXIO SPI slave IRQ handler function
 *
 * @param spiType pointer to FLEXIO_SPI_Type structure
 * @param spiHandle pointer to flexio_spi_slave_handle_t structure to store the transfer state.
 */
static inline void FLEXIO_SPI_SlaveHandleIRQ(void *spiType, void *spiHandle)
{
    FLEXIO_SPI_MasterHandleIRQ(spiType, spiHandle);
}
/*@}*/

#if defined(__cplusplus)
}
#endif /*_cplusplus*/
/*@}*/

#endif /*_FSL_FLEXIO_SPI_H_*/
