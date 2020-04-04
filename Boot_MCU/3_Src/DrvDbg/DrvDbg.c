#include "include.h"



static  sType_DbgPara RxPara;
static INT8U DrvDbgTxBuff[MAX_BUFF_LENGTH];
static INT8U TempBuff[MAX_BUFF_LENGTH];

static STypeDbgLines *DbgTxQueue;
static INT8U DbgTxQueBuff[DBG_TX_QUEUE_LENGTH][MAX_BUFF_LENGTH];
static INT8U DrvUartDmaBuff[MAX_BUFF_LENGTH];
/*
*Name      : DrvPb_CreatLine
*Brief  	 :建立n个节点的双向链表，存在周期直至掉电，不考虑删除
*Parameter : INT8U element n个节点
*return    : STypeLines * 其中一个节点的地址
*/
STypeDbgLines *DrvDbg_CreatLine(INT8U element)
{
	STypeDbgLines *first;
	STypeDbgLines *middle1;
	STypeDbgLines *middle2;
	
	first = (STypeDbgLines*)malloc(sizeof(STypeDbgLines));
	middle1 = first;
	
	for(INT8U i=0;i<(element-1);i++)
	{
		middle2 = middle1;
		middle1 = (STypeDbgLines*)malloc(sizeof(STypeDbgLines));
		middle1->Prev = middle2;
		middle2->Next =  middle1;
	}
	middle1->Next = first;
	first->Prev = middle1;
	return first;
}
/*
*Name      : DrvPb_TxBuffQueue
*Brief  	 :吧要发送的数据放入缓存
*Parameter : INT8U *txBuff,INT8U cnt    
*return    :SUCC or FAIL
*/
INT8U DrvDbg_TxBuffQueue(INT8U *txBuff,INT8U cnt)
{
	STypeDbgLines *element;
	INT8U i;
	element = DbgTxQueue;
	for(i=0;i<DBG_TX_QUEUE_LENGTH;i++)
	{
		if(element->EmptyFlag == IDLE)
		{
			break;
		}	
		element = element->Next;
	}
	if(i >= DBG_TX_QUEUE_LENGTH)
		return FAIL;
	
	element->EmptyFlag = BUSY;
	element->TxCnt = cnt;
	element->SendTime = DrvTimer_InqSysTime();
	for(i =0;i<cnt;i++)
	{
		element->Buff[i] = txBuff[i];
	}
	return SUCC;
}
/*
*Name      : DrvPb_InqTxQueueStatus
*Brief  	 : 查询发送队列是否满了
*Parameter : na
*return    : BUSY or IDLE
*/
INT8U DrvDbg_InqTxQueueStatus(void)
{
	STypeDbgLines *element;

	element = DbgTxQueue;
	for(INT8U i=0;i<DBG_TX_QUEUE_LENGTH;i++)
	{
		if(element->EmptyFlag == IDLE)
			return IDLE;
		element = element->Next;
	}
	return BUSY;
}
/*
*Name      : DrvPb_InqTxQueueCnt
*Brief  	 :查询发送队列有多少个节点有数据等待
*Parameter : NA
*return    : cnt
*/
INT8U DrvDbg_InqTxQueueCnt(void)
{
	INT8U cnt;
	STypeDbgLines *element;

	element = DbgTxQueue;
	cnt = 0;
	for(INT8U i=0;i<DBG_TX_QUEUE_LENGTH;i++)
	{
		if(element->EmptyFlag == BUSY)
			cnt++;
		element = element->Next;
	}
	return cnt;
}
/*
*Name      : DrvDbg_Init
*Brief  	 : 调试模块初始化
*Parameter : NA
*return    : NA
*/
void DrvDbg_Init(void)
{
	RxPara.FindHead = FALSE;
	RxPara.Length = 0;
	RxPara.Sum = 0;
	RxPara.TempFind = FALSE;
	RxPara.TempPack = 0;
	RxPara.TempLength = 0;
	RxPara.LastDecode = 0;  
	
	DbgTxQueue = DrvDbg_CreatLine(DBG_TX_QUEUE_LENGTH);
	
	for(INT8U i=0;i<DBG_TX_QUEUE_LENGTH;i++)
	{	
		DbgTxQueue->EmptyFlag = IDLE;
		DbgTxQueue->TxCnt = 0;
		DbgTxQueue->SendTime = 0;
		DbgTxQueue->Buff = &DbgTxQueBuff[i][0]; 
		DbgTxQueue = DbgTxQueue->Next;
	}
}
/*
*Name      : DrvDbg_FreeLine
*Brief  	 : 释放内存
*Parameter : NA
*return    : NA
*/
void DrvDbg_FreeLine(void)
{
	for(INT8U i=0;i<DBG_TX_QUEUE_LENGTH-1;i++)
	{	
		free(DbgTxQueue->Prev);
		DbgTxQueue=DbgTxQueue->Next;
	}
	free(DbgTxQueue);
}
/*
*Name      : DrvDbg_Handle
*Brief  	 : 钩子函数
*Parameter : NA
*return    : NA
*/
void DrvDbg_Handle(void)
{
	if(DrvDbg_InqDevStatus() == IDLE)
	{
		
		if(DbgTxQueue->EmptyFlag == BUSY)
		{
			for(INT8U i=0;i<DbgTxQueue->TxCnt;i++)
				DrvUartDmaBuff[i] = DbgTxQueue->Buff[i];
			DrvDbg_Buff(DrvUartDmaBuff,DbgTxQueue->TxCnt);
			DbgTxQueue->EmptyFlag = IDLE;
			DbgTxQueue->TxCnt = 0;
		}
		else
		{
			DbgTxQueue = DbgTxQueue->Next;
		}					
	}
}
/*
*Name      : DrvDbg_RxChar
*Brief  	 : 接受一个字节 解包函数
*Parameter : INT8U Data ：传入一个字节
*return    : NA
*/
void DrvDbg_RxChar(INT8U Data,ETypeDev device)
{
	static INT8U i;
	
//	static INT32U Cnt = 0;
//		{
//			static INT8U PrintData[50];
//			memset(PrintData,0,sizeof(PrintData));
//			sprintf((char*)PrintData,"Rx:%d",Cnt++);		
//			DevDbg_PrintStringHDLC(PrintData);		
//		}
	if(RxPara.FindHead == TRUE)
	{
		if((RxPara.Length >= (RxPara.RxBuff[0] + 1))
			  &&(RxPara.Sum == (INT8U)(RxPara.RxBuff[RxPara.Length - 1]<<1))
				&&(RxPara.RxBuff[1] == DRVDBG_ADD)
				&&(Data == 0x7e))
		{
			if(RxPara.TempFind == TRUE)
			{
				RxPara.Length = 0;
				RxPara.FindHead = FALSE;
				RxPara.Sum = 0;
			//	memset(RxPara.RxBuff,0,sizeof(RxPara.RxBuff));
				return;
			}
			for(i=0;i<RxPara.Length;i++)
			{
				RxPara.TempBuff[i] = RxPara.RxBuff[i];
			}
			
			RxPara.Length = 0;
			RxPara.FindHead = FALSE;
			RxPara.Sum = 0;
			RxPara.LastDecode = 0;
			//memset(RxPara.RxBuff,0,sizeof(RxPara.RxBuff));
			
			RxPara.TempFind = TRUE;
			RxPara.TempPack++;
			RxPara.TempLength = RxPara.RxBuff[0] + 1;
			DevAcu_SetDev(device);
			return;
		}
		else
		{
			RxPara.RxBuff[RxPara.Length] = Data;

			if(RxPara.Length >= MAX_BUFF_LENGTH)	
			{
				RxPara.Length = 0;
				RxPara.FindHead = FALSE;
				RxPara.Sum = 0;
				RxPara.LastDecode = 0;
				return;
				//memset(RxPara.RxBuff,0,sizeof(RxPara.RxBuff));
			}
				/*转义 7e -> 7d 5e and 7d -> 7d 5d*/
			if(RxPara.Length >= 1)
			{
				if((RxPara.RxBuff[RxPara.Length - 1] == 0x7d)  
						&&(RxPara.RxBuff[RxPara.Length] == 0x5e)
						&&(RxPara.LastDecode != RxPara.Length))
				{
					RxPara.LastDecode = RxPara.Length;
					RxPara.RxBuff[RxPara.Length - 1] = 0x7e;
					RxPara.Sum += 0x01;
					return;
				}
				else if((RxPara.RxBuff[RxPara.Length - 1] == 0x7d)
							&&(RxPara.RxBuff[RxPara.Length] == 0x5d)
							&&(RxPara.LastDecode != RxPara.Length))
				{
					RxPara.LastDecode = RxPara.Length;
					RxPara.RxBuff[RxPara.Length - 1] = 0x7d;
					return;
				}
				else
				{
					RxPara.Length = RxPara.Length + 1;
				}
			}
			else
			{
				RxPara.Length = RxPara.Length + 1;
			}
			RxPara.Sum += RxPara.RxBuff[RxPara.Length - 1];
		}
	}
	else
	{
		RxPara.Length = 0;
		RxPara.FindHead = FALSE;
		RxPara.Sum = 0;
		RxPara.LastDecode = 0;
		//memset(RxPara.RxBuff,0,sizeof(RxPara.RxBuff));
	}
	
	if(Data == 0x7e)
	{
		RxPara.Length = 0;
		RxPara.FindHead = TRUE;
		RxPara.Sum = 0;
		RxPara.LastDecode = 0;
		//memset(RxPara.RxBuff,0,sizeof(RxPara.RxBuff));
	}
}


/*
*Name      : DrvDbg_InqPackLength
*Brief  	 : 查询是否有包  和 包的大小
*Parameter : INT8U *Length 查询接受到的包长
*return    : 是否有包
*/
INT8U DrvDbg_InqPackStatus(INT8U *Length)
{
	INT8U FindFlag;
	FindFlag = RxPara.TempFind;
	*Length = RxPara.TempLength;
	return FindFlag;
}

/*
*Name      : DrvDbg_InqPackData
*Brief  	 : 读包
*Parameter : INT8U *RxBuff 读缓存
             INT8U Length  读多少个字节
*return    : NA
*/
void DrvDbg_InqPackData(INT8U *RxBuff,INT8U Length)
{
	INT8U i;
	for(i=0;i<Length;i++)
	{
		RxBuff[i] = RxPara.TempBuff[i];
	}
	RxPara.TempFind = FALSE;
	RxPara.TempLength = 0;
	memset(RxPara.TempBuff,0,sizeof(RxPara.TempBuff));
}
/*
*Name      : DrvDbg_TxPack
*Brief  	 : DBG发送打包函数
*Parameter : INT8U* Data 字符串首地址
             INT8U Length 长度
*return    : NA
*/
void DrvDbg_TxPack(INT8U *Buff,INT8U Length,INT8U queueEnable)
{
	INT8U i;
	INT8U j;
	INT8U Sum;

	Sum = 0;
	for(j=0;j<Length;j++)
	{
		Sum += Buff[j];
	}
	for(j=0;j<Length;j++)
	{
		TempBuff[j] = Buff[j];
	}
	TempBuff[Length] = Sum;
	
	i = 0;
	DrvDbgTxBuff[i++] = 0x7e;
	for(j=0;j<Length+1;j++)
	{
		if(TempBuff[j] == 0x7e)
		{
			DrvDbgTxBuff[i++] = 0x7d;
			DrvDbgTxBuff[i++] = 0x5e;
		}
		else if(TempBuff[j] == 0x7d)
		{
			DrvDbgTxBuff[i++] = 0x7d;
			DrvDbgTxBuff[i++] = 0x5d;
		}
		else
		{
			DrvDbgTxBuff[i++] = TempBuff[j];
		}
	}
	DrvDbgTxBuff[i++] = 0x7e;
	if(queueEnable != ENABLE)
		DrvDbg_Buff(DrvDbgTxBuff,i);
	else	
		DrvDbg_TxBuffQueue(DrvDbgTxBuff,i);
}

/*
*Name      : DrvDbg_Buff
*Brief  	 : DBG发送函数
*Parameter : INT8U* Data 字符串首地址
             INT8U Length 长度
*return    : NA
*/
void DrvDbg_Buff(INT8U *Buff,INT8U Length)
{
	INT8U i;
	for(i=0;i<Length;i++)
	{
		DrvDbgTxBuff[i] = Buff[i];
	}
	
	switch(DevAcu_InqDev())
	{
		case DEV_UART0:
			DrvUart_TxBuff(UART0,DrvDbgTxBuff,Length);
			break;
		case DEV_UART1:
			DrvUart_TxBuff(UART1,DrvDbgTxBuff,Length);
			break;
		case DEV_UART2:
			DrvUart_TxBuff(UART2,DrvDbgTxBuff,Length);
			break;
#ifdef CPU_MK66FN2M0VMD18			
		case DEV_UART3:
			DrvUart_TxBuff(UART3,DrvDbgTxBuff,Length);
			break;
		case DEV_UART4:
			DrvUart_TxBuff(UART4,DrvDbgTxBuff,Length);
			break;
#endif		
		case DEV_CAN0:
			DrvCan_TxBuff(CAN0,CAN_STD,CAN_DATA,CONFIG_CAN0_ID,DrvDbgTxBuff,Length);
			break;
		case DEV_CAN1:
			DrvCan_TxBuff(CAN0,CAN_STD,CAN_DATA,CONFIG_CAN1_ID,DrvDbgTxBuff,Length);
			break;
		default:break; 
	}
}
/*
*Name      : DrvDbg_Buff
*Brief  	 : DBG发送函数
*Parameter : INT8U* Data 字符串首地址
             INT8U Length 长度
*return    : NA
*/
INT8U DrvDbg_InqDevStatus(void)
{
	switch(DevAcu_InqDev())
	{
		case DEV_UART0:
			return DrvUart_InqUartStatus(UART0);
			break;
		case DEV_UART1:
			return DrvUart_InqUartStatus(UART1);
			break;
		case DEV_UART2:
			return DrvUart_InqUartStatus(UART2);
			break;
#ifdef CPU_MK66FN2M0VMD18			
		case DEV_UART3:
			return DrvUart_InqUartStatus(UART3);
			break;
		case DEV_UART4:
			return DrvUart_InqUartStatus(UART4);
			break;
#endif		
		case DEV_CAN0:
			return DrvCan_InqTxStatus(CAN0);
			break;
		case DEV_CAN1:
			return DrvCan_InqTxStatus(CAN1);
			break;
		default:break; 
	}
	return BUSY;
}
