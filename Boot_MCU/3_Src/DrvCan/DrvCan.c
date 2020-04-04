#include "include.h"

//static flexcan_handle_t flexcanHandle;
static STypeCanPara CanPara[2];

void DrvCan_TxPack(CAN_Type *canx,TYPE_FORMAT Format,TYPE_FRAME Frame,
	INT16U Add,INT8U *Buff,INT8U Length);
/***********************************************************************************
*Name      : DrvCan_Init
*Brief  	 : CAN模块初始化
*Parameter : CAN_Type *canx     CAN通道
				INT32U bps      波特率
				INT8U id        ID
*return    : NA
***********************************************************************************/
void DrvCan_Init(CAN_Type *canx,INT32U bps,INT8U id)
{
	flexcan_config_t flexcanConfig;
	flexcan_rx_mb_config_t mbConfig;
	
	
	CanPara[0].Status = IDLE;
	CanPara[1].Status = IDLE;
	

	FLEXCAN_GetDefaultConfig(&flexcanConfig);
	flexcanConfig.clkSrc = kFLEXCAN_ClkSrcPeri;
	flexcanConfig.baudRate = bps;
	flexcanConfig.enableLoopBack = false;
	flexcanConfig.enableIndividMask = true;

	switch((INT32U)canx)
	{
		case (INT32U)CAN0:
			
			PORT_SetPinMux(CAN0_PORT, CAN0_PORT_TX_PIN, CAN0_PORT_MUX_SOURCE);
			PORT_SetPinMux(CAN0_PORT, CAN0_PORT_RX_PIN, CAN0_PORT_MUX_SOURCE);
		
			FLEXCAN_Init(CAN0, &flexcanConfig, CLOCK_GetFreq(kCLOCK_BusClk));
		
		
			/* Setup Rx Message Buffer. */
			mbConfig.format = CAN_STD;
			mbConfig.type = CAN_DATA;
			mbConfig.id = FLEXCAN_ID_STD(id);
			FLEXCAN_SetRxMbConfig(CAN0, CAN0_RX_MESSAGE_BUFFER_NUM, &mbConfig, true);
		
			FlEXCAN_SetRxIndividualMask(CAN0,CAN0_RX_MESSAGE_BUFFER_NUM,FLEXCAN_RX_MB_STD_MASK(CAN_MASK_BIT,1,1));
				/* Setup Tx Message Buffer. */
			FLEXCAN_SetTxMbConfig(CAN0, CAN0_TX_MESSAGE_BUFFER_NUM, true);
		
			/* Enable Rx Message Buffer interrupt. */
			FLEXCAN_EnableMbInterrupts(CAN0, 1 << CAN0_RX_MESSAGE_BUFFER_NUM);
			FLEXCAN_EnableMbInterrupts(CAN0, 1 << CAN0_TX_MESSAGE_BUFFER_NUM);
			EnableIRQ(CAN0_ORed_Message_buffer_IRQn);

			FLEXCAN_Enable(CAN0,true);

			NVIC_SetPriority(CAN0_ORed_Message_buffer_IRQn,3);
		
//			DrvCfg_GpioInit(GPIOE,26u,kGPIO_DigitalOutput,0);//拉低standbay
		
			/* Setup Tx Message Buffer. */
			FLEXCAN_SetTxMbConfig(CAN0, CAN0_TX_MESSAGE_BUFFER_NUM, true);
		
		break;
		case (INT32U)CAN1:
			PORT_SetPinMux(CAN1_PORT, CAN1_PORT_TX_PIN, CAN1_PORT_MUX_SOURCE);
			PORT_SetPinMux(CAN1_PORT, CAN1_PORT_RX_PIN, CAN1_PORT_MUX_SOURCE);
		
			FLEXCAN_Init(CAN1, &flexcanConfig, CLOCK_GetFreq(kCLOCK_BusClk));
		
		
			/* Setup Rx Message Buffer. */
			mbConfig.format = CAN_STD;
			mbConfig.type = CAN_DATA;
			mbConfig.id = FLEXCAN_ID_STD(id);
			FLEXCAN_SetRxMbConfig(CAN1, CAN1_RX_MESSAGE_BUFFER_NUM, &mbConfig, true);
		
			FlEXCAN_SetRxIndividualMask(CAN1,CAN1_RX_MESSAGE_BUFFER_NUM,FLEXCAN_RX_MB_STD_MASK(CAN_MASK_BIT,1,1));
			/* Setup Tx Message Buffer. */
			FLEXCAN_SetTxMbConfig(CAN1, CAN1_TX_MESSAGE_BUFFER_NUM, true);	
			/* Enable Rx Message Buffer interrupt. */
			FLEXCAN_EnableMbInterrupts(CAN1, 1 << CAN1_RX_MESSAGE_BUFFER_NUM);
			FLEXCAN_EnableMbInterrupts(CAN1, 1 << CAN1_TX_MESSAGE_BUFFER_NUM);
			EnableIRQ(CAN1_ORed_Message_buffer_IRQn);

			FLEXCAN_Enable(CAN1,true);

			NVIC_SetPriority(CAN1_ORed_Message_buffer_IRQn,3);
		
//			DrvCfg_GpioInit(GPIOE,26u,kGPIO_DigitalOutput,0);//拉低standbay
		
			/* Setup Tx Message Buffer. */
			FLEXCAN_SetTxMbConfig(CAN1, CAN1_TX_MESSAGE_BUFFER_NUM, true);	
		break;
		default:
			return;
		break;
	}
}
/***********************************************************************************
*Name      : DrvCan_Stop
*Brief  	 : CAN模块STOP
*Parameter : CAN_Type *canx     CAN通道
*return    : NA
***********************************************************************************/
void DrvCan_Stop(CAN_Type *canx)
{
	switch((INT32U)canx)
	{
		case (INT32U)CAN0:
			DisableIRQ(CAN0_ORed_Message_buffer_IRQn);

		break;
		case (INT32U)CAN1:
			DisableIRQ(CAN1_ORed_Message_buffer_IRQn);
		break;
		default : break;
	}
}
/***********************************************************************************
*Name      : DrvCan0_IRQHandler
*Brief  	 : CAN模块通道0中断处理
*Parameter : NA
*return    : NA
***********************************************************************************/
void DrvCan0_IRQHandler(void)
{
    /* If new data arrived. */
	static	INT8U i;
	static	flexcan_frame_t rxFrame;
	static	INT8U RxData[8];
	static	INT8U RxLength;
    if (FLEXCAN_GetMbStatusFlags(CAN0, 1 << CAN0_RX_MESSAGE_BUFFER_NUM))
    {
        FLEXCAN_ClearMbStatusFlags(CAN0, 1 << CAN0_RX_MESSAGE_BUFFER_NUM);
        FLEXCAN_ReadRxMb(CAN0, CAN0_RX_MESSAGE_BUFFER_NUM, &rxFrame);
		
				RxLength = rxFrame.length;
				RxData[0] = rxFrame.dataByte0;
				RxData[1] = rxFrame.dataByte1;
				RxData[2] = rxFrame.dataByte2;
				RxData[3] = rxFrame.dataByte3;
				RxData[4] = rxFrame.dataByte4;
				RxData[5] = rxFrame.dataByte5;
				RxData[6] = rxFrame.dataByte6;
				RxData[7] = rxFrame.dataByte7;
				for(i=0;(i<RxLength)&&(i<=8);i++)
				{
					DrvDbg_RxChar(RxData[i],DEV_CAN0);
				}
				return;
			
    }
		/* If Tx data completed. */
    if (FLEXCAN_GetMbStatusFlags(CAN0, 1 << CAN0_TX_MESSAGE_BUFFER_NUM))
    {
        FLEXCAN_ClearMbStatusFlags(CAN0, 1 << CAN0_TX_MESSAGE_BUFFER_NUM);
		if(CanPara[0].TxLength != 0)
		{
			DrvCan_TxBuff(CAN0,CanPara[0].Format,CanPara[0].Frame,CanPara[0].Add,CanPara[0].DataAdd,CanPara[0].TxLength);
		}
		else
		{
			CanPara[0].Status = IDLE;
		}	
    }
}
/***********************************************************************************
*Name      : DrvCan1_IRQHandler
*Brief  	 : CAN模块通道1中断处理
*Parameter : NA
*return    : NA
***********************************************************************************/
void DrvCan1_IRQHandler(void)
{
    /* If new data arrived. */
	static	INT8U i;
	static	flexcan_frame_t rxFrame;
	static	INT8U RxData[8];
	static	INT8U RxLength;
    if (FLEXCAN_GetMbStatusFlags(CAN1, 1 << CAN1_RX_MESSAGE_BUFFER_NUM))
    {
        FLEXCAN_ClearMbStatusFlags(CAN1, 1 << CAN1_RX_MESSAGE_BUFFER_NUM);
        FLEXCAN_ReadRxMb(CAN1, CAN1_RX_MESSAGE_BUFFER_NUM, &rxFrame);
		
				RxLength = rxFrame.length;
				RxData[0] = rxFrame.dataByte0;
				RxData[1] = rxFrame.dataByte1;
				RxData[2] = rxFrame.dataByte2;
				RxData[3] = rxFrame.dataByte3;
				RxData[4] = rxFrame.dataByte4;
				RxData[5] = rxFrame.dataByte5;
				RxData[6] = rxFrame.dataByte6;
				RxData[7] = rxFrame.dataByte7;
				for(i=0;(i<RxLength)&&(i<=8);i++)
				{
					DrvDbg_RxChar(RxData[i],DEV_CAN1);
				}
			
    }
		/* If Tx data completed. */
    if (FLEXCAN_GetMbStatusFlags(CAN1, 1 << CAN1_TX_MESSAGE_BUFFER_NUM))
    {
        FLEXCAN_ClearMbStatusFlags(CAN1, 1 << CAN1_TX_MESSAGE_BUFFER_NUM);
		if(CanPara[1].TxLength != 0)
		{
			DrvCan_TxBuff(CAN1,CanPara[1].Format,CanPara[1].Frame,CanPara[1].Add,CanPara[1].DataAdd,CanPara[1].TxLength);
		}
		else
		{
			CanPara[1].Status = IDLE;
		}	
    }
}
/***********************************************************************************
*Name      : DrvCan_TxPack
*Brief  	 : CAN发送一包数据 
*Parameter : TYPE_FORMAT Format：什么帧   标准帧 CAN_STD  or 扩展帧 CAN_STD
             TYPE_FRAME Frame  : 什么帧   数据帧 CAN_DATA or 远程帧 CAN_REMOT
             INT16U Add        : 地址    
             INT8U *Buff       : 待发送数据地址
             INT8U Length      : 发送数据长度 <= 8 byte
*return    : NA
***********************************************************************************/
void DrvCan_TxPack(CAN_Type *canx,TYPE_FORMAT Format,TYPE_FRAME Frame,
	INT16U Add,INT8U *Buff,INT8U Length)
{
	static flexcan_frame_t txFrame[2];
	
	
	switch((INT32U)canx)
	{
		case (INT32U)CAN0:
			FLEXCAN_ClearMbStatusFlags(CAN0, 1 << CAN0_TX_MESSAGE_BUFFER_NUM);	
			txFrame[0].format    = Format;
			txFrame[0].type      = Frame;
			txFrame[0].id        = FLEXCAN_ID_STD(Add);
			txFrame[0].length    = Length;
			txFrame[0].dataByte0 = Buff[0];
			txFrame[0].dataByte1 = Buff[1];
			txFrame[0].dataByte2 = Buff[2];
			txFrame[0].dataByte3 = Buff[3];
			txFrame[0].dataByte4 = Buff[4];
			txFrame[0].dataByte5 = Buff[5];
			txFrame[0].dataByte6 = Buff[6];
			txFrame[0].dataByte7 = Buff[7];
			//FLEXCAN_TransferSendNonBlocking(CAN0, &flexcanHandle, &txBuff);
			FLEXCAN_WriteTxMb(CAN0, CAN0_TX_MESSAGE_BUFFER_NUM, &txFrame[0]);
			break;
		case (INT32U)CAN1:
			FLEXCAN_ClearMbStatusFlags(CAN1, 1 << CAN1_TX_MESSAGE_BUFFER_NUM);	
			txFrame[1].format    = Format;
			txFrame[1].type      = Frame;
			txFrame[1].id        = FLEXCAN_ID_STD(Add);
			txFrame[1].length    = Length;
			txFrame[1].dataByte0 = Buff[0];
			txFrame[1].dataByte1 = Buff[1];
			txFrame[1].dataByte2 = Buff[2];
			txFrame[1].dataByte3 = Buff[3];
			txFrame[1].dataByte4 = Buff[4];
			txFrame[1].dataByte5 = Buff[5];
			txFrame[1].dataByte6 = Buff[6];
			txFrame[1].dataByte7 = Buff[7];
			//FLEXCAN_TransferSendNonBlocking(CAN0, &flexcanHandle, &txBuff);
			FLEXCAN_WriteTxMb(CAN1, CAN1_TX_MESSAGE_BUFFER_NUM, &txFrame[1]);
			break;
		default:break;
	}
}

/***********************************************************************************
*Name      : DrvCan_TxBuff
*Brief  	 : 数据打包，对外接口     发送一串数据
*Parameter : TYPE_FORMAT Format：什么帧   标准帧 CAN_STD  or 扩展帧 CAN_STD
             TYPE_FRAME Frame  : 什么帧   数据帧 CAN_DATA or 远程帧 CAN_REMOT
             INT16U Add        : 地址    
             INT8U *Buff       : 待发送数据地址
             INT8U Length      : 发送数据长度 
*return    : NA
***********************************************************************************/
void DrvCan_TxBuff(CAN_Type *canx,TYPE_FORMAT Format,TYPE_FRAME Frame,
	          INT16U Add,INT8U *Buff,INT8U Length)
{
		switch((INT32U)canx)
	{
		case (INT32U)CAN0:
			CanPara[0].Format = Format;
			CanPara[0].Frame = Frame;
			CanPara[0].Status = BUSY;
			CanPara[0].Add = Add;
			CanPara[0].DataAdd = Buff;
			CanPara[0].TxLength = Length;

			if(Frame == CAN_REMOT)
					CanPara[0].TxLength = 0;
			if(CanPara[0].TxLength > 8)
			{
				DrvCan_TxPack(CAN0,CanPara[0].Format,CanPara[0].Frame,CanPara[0].Add,CanPara[0].DataAdd,8);
				CanPara[0].TxLength = CanPara[0].TxLength - 8;
				CanPara[0].DataAdd = CanPara[0].DataAdd + 8;
				
			}
			else
			{
				DrvCan_TxPack(CAN0,CanPara[0].Format,CanPara[0].Frame,CanPara[0].Add,CanPara[0].DataAdd,CanPara[0].TxLength);
				CanPara[0].TxLength = 0;
			}
			break;
		case (INT32U)CAN1:
			CanPara[1].Format = Format;
			CanPara[1].Frame = Frame;
			CanPara[1].Status = BUSY;
			CanPara[1].Add = Add;
			CanPara[1].DataAdd = Buff;
			CanPara[1].TxLength = Length;

			if(Frame == CAN_REMOT)
					CanPara[1].TxLength = 0;
			if(CanPara[1].TxLength > 8)
			{
				DrvCan_TxPack(CAN1,CanPara[1].Format,CanPara[1].Frame,CanPara[1].Add,CanPara[1].DataAdd,8);
				CanPara[1].TxLength = CanPara[1].TxLength - 8;
				CanPara[1].DataAdd = CanPara[1].DataAdd + 8;
				
			}
			else
			{
				DrvCan_TxPack(CAN1,CanPara[1].Format,CanPara[1].Frame,CanPara[1].Add,CanPara[1].DataAdd,CanPara[1].TxLength);
				CanPara[1].TxLength = 0;
			}
			break;
		default:
			break;
	}
}	

/***********************************************************************************
*Name      : DrvCan_InqTxStatus
*Brief  	 : 查询CAN当前状态
*Parameter : NA
*return    : BUSY or IDLE
***********************************************************************************/
INT8U DrvCan_InqTxStatus(CAN_Type *canx)
{
	switch((INT32U)canx)
	{
		case (INT32U)CAN0:
			return CanPara[0].Status;break;
		case (INT32U)CAN1:
			return CanPara[1].Status;break;
		default: return BUSY;break;
	}
	return BUSY;
}	



