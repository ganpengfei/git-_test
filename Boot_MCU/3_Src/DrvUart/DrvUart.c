#include "include.h"


static INT8U UartBusyFlag[5] = {IDLE,IDLE,IDLE,IDLE,IDLE};

void UART0_UserCallback(UART_Type *base, uart_edma_handle_t *handle, status_t status, void *userData);
void UART1_UserCallback(UART_Type *base, uart_edma_handle_t *handle, status_t status, void *userData);
void UART2_UserCallback(UART_Type *base, uart_edma_handle_t *handle, status_t status, void *userData);
void UART3_UserCallback(UART_Type *base, uart_edma_handle_t *handle, status_t status, void *userData);
void UART4_UserCallback(UART_Type *base, uart_edma_handle_t *handle, status_t status, void *userData);


/***********************************************************************************
*Name      : DrvUart_Init
*Brief  	 : 串口外设初始化
*Parameter : UART_Type *Uartx : 串口号
																		UART0		
																		UART1
																		UART2
																		UART3
																		UART4
             INT32U Baud : 波特率(9600/115200/...)      
             uart_parity_mode_t Parity: 校验方式 UART_PARITY_EVEN ， UART_PARITY_NONE ，UART_PARITY_ODD
*return    : NA
***********************************************************************************/
void DrvUart_Init(UART_Type *Uartx,INT32U Baud,uart_parity_mode_t Parity)
{
	uart_config_t config;
	/*
	 * config.baudRate_Bps = 115200U;
	 * config.parityMode = kUART_ParityDisabled;
	 * config.stopBitCount = kUART_OneStopBit;
	 * config.txFifoWatermark = 0;
	 * config.rxFifoWatermark = 1;
	 * config.enableTx = false;
	 * config.enableRx = false;
	 */
	UART_GetDefaultConfig(&config);
	config.baudRate_Bps = Baud;		
	config.enableTx = TRUE;
	config.enableRx = TRUE;
	config.parityMode = Parity;
	switch((INT32U)Uartx)
	{
		case (INT32U)UART0:
			PORT_SetPinMux(UART0_PORT, UART0_PORT_TX_PIN, UART0_PORT_MUX_FUN);
			PORT_SetPinMux(UART0_PORT, UART0_PORT_RX_PIN, UART0_PORT_MUX_FUN);
		
			UART_Init(Uartx, &config, CLOCK_GetFreq(SYS_CLK));
			UART_EnableInterrupts(Uartx, kUART_RxDataRegFullInterruptEnable | kUART_RxOverrunInterruptEnable);
			UART0->C5 = 0x80;//发送通道打开 
			NVIC_SetPriority(UART0_RX_TX_IRQn,4);
			EnableIRQ(UART0_RX_TX_IRQn);
			break;
		case (INT32U)UART1:
			PORT_SetPinMux(UART1_PORT, UART1_PORT_TX_PIN, UART1_PORT_MUX_FUN);
			PORT_SetPinMux(UART1_PORT, UART1_PORT_RX_PIN, UART1_PORT_MUX_FUN);		
		
			UART_Init(Uartx, &config, CLOCK_GetFreq(SYS_CLK));
			UART_EnableInterrupts(Uartx, kUART_RxDataRegFullInterruptEnable | kUART_RxOverrunInterruptEnable);
			UART1->C5 = 0x80;//发送通道打开 
			NVIC_SetPriority(UART1_RX_TX_IRQn,4);
			EnableIRQ(UART1_RX_TX_IRQn);
			break;	
		case (INT32U)UART2:
			PORT_SetPinMux(UART2_PORT, UART2_PORT_TX_PIN, UART2_PORT_MUX_FUN);
			PORT_SetPinMux(UART2_PORT, UART2_PORT_RX_PIN, UART2_PORT_MUX_FUN);	
		
			UART_Init(Uartx, &config, CLOCK_GetFreq(BUS_CLK));
			UART_EnableInterrupts(Uartx, kUART_RxDataRegFullInterruptEnable | kUART_RxOverrunInterruptEnable);
			UART2->C5 = 0x80;//发送通道打开 
			NVIC_SetPriority(UART2_RX_TX_IRQn,4);
			EnableIRQ(UART2_RX_TX_IRQn);
			break;
#ifdef CPU_MK66FN2M0VMD18			
		case (INT32U)UART3:
			PORT_SetPinMux(UART3_PORT, UART3_PORT_TX_PIN, UART3_PORT_MUX_FUN);
			PORT_SetPinMux(UART3_PORT, UART3_PORT_RX_PIN, UART3_PORT_MUX_FUN);		
		
			UART_Init(Uartx, &config, CLOCK_GetFreq(BUS_CLK));
			UART_EnableInterrupts(Uartx, kUART_RxDataRegFullInterruptEnable | kUART_RxOverrunInterruptEnable);
			UART3->C5 = 0x80;//发送通道打开 
			NVIC_SetPriority(UART3_RX_TX_IRQn,4);
			EnableIRQ(UART3_RX_TX_IRQn);
			break;
		case (INT32U)UART4:
			
			PORT_SetPinMux(UART4_PORT, UART4_PORT_TX_PIN, UART4_PORT_MUX_FUN);
			PORT_SetPinMux(UART4_PORT, UART4_PORT_RX_PIN, UART4_PORT_MUX_FUN);	
				
			UART_Init(Uartx, &config, CLOCK_GetFreq(BUS_CLK));
			UART_EnableInterrupts(Uartx, kUART_RxDataRegFullInterruptEnable | kUART_RxOverrunInterruptEnable);
			UART4->C5 = 0x80;//发送通道打开 
			NVIC_SetPriority(UART4_RX_TX_IRQn,4);
			EnableIRQ(UART4_RX_TX_IRQn);
			break;
#endif		
		default:break;
	}
}
void DrvUart_Stop(UART_Type *Uartx)
{

	switch((INT32U)Uartx)
	{
		case (INT32U)UART0:
				DisableIRQ(UART0_RX_TX_IRQn);
			break;
		case (INT32U)UART1:
				DisableIRQ(UART1_RX_TX_IRQn);
			break;
		case (INT32U)UART2:
				DisableIRQ(UART2_RX_TX_IRQn);
			break;
#ifdef CPU_MK66FN2M0VMD18		
		case (INT32U)UART3:
				DisableIRQ(UART3_RX_TX_IRQn);
			break;
		case (INT32U)UART4:
				DisableIRQ(UART4_RX_TX_IRQn);
			break;
#endif		
	}	
}
/***********************************************************************************
*Name      : DrvUart_Com0RxCharIrq
*Brief  	 : 串口0中断处理函数
*Parameter : NA 
*return    : NA
***********************************************************************************/
void DrvUart_Com0RxCharIrq(void)
{

	if ((kUART_RxDataRegFullFlag | kUART_RxOverrunFlag) & UART_GetStatusFlags(UART0))
	{
		DrvDbg_RxChar(UART_ReadByte(UART0),DEV_UART0);
	}
}
/***********************************************************************************
*Name      : DrvUart_Com1RxCharIrq
*Brief  	 : 串口1中断处理函数
*Parameter : NA 
*return    : NA
***********************************************************************************/
void DrvUart_Com1RxCharIrq(void)
{
	if ((kUART_RxDataRegFullFlag | kUART_RxOverrunFlag) & UART_GetStatusFlags(UART1))
	{
		DrvDbg_RxChar(UART_ReadByte(UART1),DEV_UART1);
	}
}
/***********************************************************************************
*Name      : DrvUart_Com2RxCharIrq
*Brief  	 : 串口2中断处理函数
*Parameter : NA 
*return    : NA
***********************************************************************************/
void DrvUart_Com2RxCharIrq(void)
{
	if ((kUART_RxDataRegFullFlag | kUART_RxOverrunFlag) & UART_GetStatusFlags(UART2))
	{
		DrvDbg_RxChar(UART_ReadByte(UART2),DEV_UART2);
	}
}
#ifdef CPU_MK66FN2M0VMD18
/***********************************************************************************
*Name      : DrvUart_Com3RxCharIrq
*Brief  	 : 串口3中断处理函数
*Parameter : NA 
*return    : NA
***********************************************************************************/
void DrvUart_Com3RxCharIrq(void)
{
	if ((kUART_RxDataRegFullFlag | kUART_RxOverrunFlag) & UART_GetStatusFlags(UART3))
	{
		DrvDbg_RxChar(UART_ReadByte(UART3),DEV_UART3);
	}
}
/***********************************************************************************
*Name      : DrvUart_Com4RxCharIrq
*Brief  	 : 串口4中断处理函数
*Parameter : NA 
*return    : NA
***********************************************************************************/
void DrvUart_Com4RxCharIrq(void)
{
	if ((kUART_RxDataRegFullFlag | kUART_RxOverrunFlag) & UART_GetStatusFlags(UART4))
	{
		DrvDbg_RxChar(UART_ReadByte(UART4),DEV_UART4);
	}
}
#endif
/***********************************************************************************
*Name      : DrvUart_TxBuff
*Brief  	 : 串口发送一段长度的Buff(DMA模式)
*Parameter : TypeEnum_Uartx Uartx : 串口号
																		UART0		
																		UART1
																		UART2
																		UART3
																		UART4
             INT8U* Buff : 待发送数据的首地址  
						 INT8U Lenth：待发送数据长度
*return    : NA
***********************************************************************************/
static uart_edma_handle_t g_uartEdmaHandle[5];
static edma_handle_t g_uartTxEdmaHandle[5];
static uart_transfer_t xfer[5];
void DrvUart_TxBuff(UART_Type *Uartx,INT8U* Buff,INT8U Lenth)
{
	#ifdef CPU_MK66FN2M0VMD18
	
	/* Configure DMA. */
	DMAMGR_Init();
	
	switch((INT32U)Uartx)
	{
		case (INT32U)UART0:
			/* Request dma channels from DMA manager. */
			DMAMGR_RequestChannel(UART0_TX_DMA_SOURCE, UART0_DMA_CHANNAL, &g_uartTxEdmaHandle[0]);
			/* Create UART DMA handle. */
			UART_TransferCreateHandleEDMA(Uartx, &g_uartEdmaHandle[0], UART0_UserCallback, NULL, &g_uartTxEdmaHandle[0],
														NULL);
			/* Send g_tipString out. */
			xfer[0].data = Buff;
			xfer[0].dataSize = Lenth;
			UartBusyFlag[0] = BUSY;
			UART_SendEDMA(Uartx, &g_uartEdmaHandle[0], &xfer[0]);
			break;
		case (INT32U)UART1:
			/* Request dma channels from DMA manager. */
			DMAMGR_RequestChannel(UART1_TX_DMA_SOURCE, UART1_DMA_CHANNAL, &g_uartTxEdmaHandle[1]);
			/* Create UART DMA handle. */
			UART_TransferCreateHandleEDMA(Uartx, &g_uartEdmaHandle[1], UART1_UserCallback, NULL, &g_uartTxEdmaHandle[1],
														NULL);
			/* Send g_tipString out. */
			xfer[1].data = Buff;
			xfer[1].dataSize = Lenth;
			UartBusyFlag[1] = BUSY;
			UART_SendEDMA(Uartx, &g_uartEdmaHandle[1], &xfer[1]);
			break;
		case (INT32U)UART2:
			/* Request dma channels from DMA manager. */
			DMAMGR_RequestChannel(UART2_TX_DMA_SOURCE, UART2_DMA_CHANNAL, &g_uartTxEdmaHandle[2]);
			/* Create UART DMA handle. */
			UART_TransferCreateHandleEDMA(Uartx, &g_uartEdmaHandle[2], UART2_UserCallback, NULL, &g_uartTxEdmaHandle[2],
														NULL);
			/* Send g_tipString out. */
			xfer[2].data = Buff;
			xfer[2].dataSize = Lenth;
			UartBusyFlag[2] = BUSY;
			UART_SendEDMA(Uartx, &g_uartEdmaHandle[2], &xfer[2]);
			break;
	
		case (INT32U)UART3:
			/* Request dma channels from DMA manager. */
			DMAMGR_RequestChannel(UART3_TX_DMA_SOURCE, UART3_DMA_CHANNAL, &g_uartTxEdmaHandle[3]);
			/* Create UART DMA handle. */
			UART_TransferCreateHandleEDMA(Uartx, &g_uartEdmaHandle[3], UART3_UserCallback, NULL, &g_uartTxEdmaHandle[3],
														NULL);
			/* Send g_tipString out. */
			xfer[3].data = Buff;
			xfer[3].dataSize = Lenth;
			UartBusyFlag[3] = BUSY;
			UART_SendEDMA(Uartx, &g_uartEdmaHandle[3], &xfer[3]);
			break;
		case (INT32U)UART4:
			/* Request dma channels from DMA manager. */
			DMAMGR_RequestChannel(UART4_TX_DMA_SOURCE, UART4_DMA_CHANNAL, &g_uartTxEdmaHandle[4]);
			/* Create UART DMA handle. */
			UART_TransferCreateHandleEDMA(Uartx, &g_uartEdmaHandle[4], UART4_UserCallback, NULL, &g_uartTxEdmaHandle[4],
														NULL);
			/* Send g_tipString out. */
			xfer[4].data = Buff;
			xfer[4].dataSize = Lenth;
			UartBusyFlag[4] = BUSY;
			UART_SendEDMA(Uartx, &g_uartEdmaHandle[4], &xfer[4]);
			break;
		
		default : break;
	}
#endif
#ifdef CPU_MKS22FN256VLL12
	/* Configure DMA. */
	DMAMGR_Init();
	
	switch((INT32U)Uartx)
	{
		case (INT32U)UART0:
			/* Request dma channels from DMA manager. */
			DMAMGR_RequestChannel(UART0_TX_DMA_SOURCE, UART0_DMA_CHANNAL, &g_uartTxEdmaHandle[0]);
			/* Create UART DMA handle. */
			UART_CreateHandleEDMA(Uartx, &g_uartEdmaHandle[0], UART0_UserCallback, NULL, &g_uartTxEdmaHandle[0],
														NULL);
			/* Send g_tipString out. */
			xfer[0].data = Buff;
			xfer[0].dataSize = Lenth;
			UartBusyFlag[0] = BUSY;
			UART_SendEDMA(Uartx, &g_uartEdmaHandle[0], &xfer[0]);
			break;
		case (INT32U)UART1:
			/* Request dma channels from DMA manager. */
			DMAMGR_RequestChannel(UART1_TX_DMA_SOURCE, UART1_DMA_CHANNAL, &g_uartTxEdmaHandle[1]);
			/* Create UART DMA handle. */
			UART_CreateHandleEDMA(Uartx, &g_uartEdmaHandle[1], UART1_UserCallback, NULL, &g_uartTxEdmaHandle[1],
														NULL);
			/* Send g_tipString out. */
			xfer[1].data = Buff;
			xfer[1].dataSize = Lenth;
			UartBusyFlag[1] = BUSY;
			UART_SendEDMA(Uartx, &g_uartEdmaHandle[1], &xfer[1]);
			break;
		case (INT32U)UART2:
			/* Request dma channels from DMA manager. */
			DMAMGR_RequestChannel(UART2_TX_DMA_SOURCE, UART2_DMA_CHANNAL, &g_uartTxEdmaHandle[2]);
			/* Create UART DMA handle. */
			UART_CreateHandleEDMA(Uartx, &g_uartEdmaHandle[2], UART2_UserCallback, NULL, &g_uartTxEdmaHandle[2],
														NULL);
			/* Send g_tipString out. */
			xfer[2].data = Buff;
			xfer[2].dataSize = Lenth;
			UartBusyFlag[2] = BUSY;
			UART_SendEDMA(Uartx, &g_uartEdmaHandle[2], &xfer[2]);
			break;
		default : break;
	}
#endif
}
/***********************************************************************************
*Name      : DrvUart_InqUartStatus
*Brief  	 : 查询串口发送标志
*Parameter : TypeEnum_Uartx Uartx : 串口号
																		UART0		
																		UART1
																		UART2
																		UART3
																		UART4
*return    : 对应串口号的工作标志 BUSY or IDLE
***********************************************************************************/
INT8U DrvUart_InqUartStatus(UART_Type *Uartx)
{
	INT8U Data;
	
	switch((INT32U)Uartx)
	{
		case (INT32U)UART0:Data = UartBusyFlag[0];break;
		case (INT32U)UART1:Data = UartBusyFlag[1];break;
		case (INT32U)UART2:Data = UartBusyFlag[2];break;
#ifdef CPU_MK66FN2M0VMD18
		case (INT32U)UART3:Data = UartBusyFlag[3];break;
		case (INT32U)UART4:Data = UartBusyFlag[4];break;
#endif
		default:break;
	}
	return Data;
}
///***********************************************************************************
//*Name      : UART0_UserCallback
//*Brief  	 : UART0发送完成标志
//*Parameter : NA 
//*return    : NA
//***********************************************************************************/
/* UART user callback */
void UART0_UserCallback(UART_Type *base, uart_edma_handle_t *handle, status_t status, void *userData)
{
	userData = userData;

	if (kStatus_UART_TxIdle == status)
	{
		UartBusyFlag[0] = IDLE;
	}

}
///***********************************************************************************
//*Name      : UART1_UserCallback
//*Brief  	 : UART1发送完成标志
//*Parameter : NA 
//*return    : NA
//***********************************************************************************/
void UART1_UserCallback(UART_Type *base, uart_edma_handle_t *handle, status_t status, void *userData)
{
	userData = userData;

	if (kStatus_UART_TxIdle == status)
	{
		UartBusyFlag[1] = IDLE;
	}

}
///***********************************************************************************
//*Name      : UART2_UserCallback
//*Brief  	 : UART2发送完成标志
//*Parameter : NA 
//*return    : NA
//***********************************************************************************/
void UART2_UserCallback(UART_Type *base, uart_edma_handle_t *handle, status_t status, void *userData)
{
	userData = userData;

	if (kStatus_UART_TxIdle == status)
	{
		UartBusyFlag[2] = IDLE;
	}

}
///***********************************************************************************
//*Name      : UART3_UserCallback
//*Brief  	 : UART3发送完成标志
//*Parameter : NA 
//*return    : NA
//***********************************************************************************/
void UART3_UserCallback(UART_Type *base, uart_edma_handle_t *handle, status_t status, void *userData)
{
	userData = userData;

	if (kStatus_UART_TxIdle == status)
	{
		UartBusyFlag[3] = IDLE;
	}

}
///***********************************************************************************
//*Name      : UART4_UserCallback
//*Brief  	 : UART4发送完成标志
//*Parameter : NA 
//*return    : NA
//***********************************************************************************/
void UART4_UserCallback(UART_Type *base, uart_edma_handle_t *handle, status_t status, void *userData)
{
	userData = userData;

	if (kStatus_UART_TxIdle == status)
	{
		UartBusyFlag[4] = IDLE;
	}

}
