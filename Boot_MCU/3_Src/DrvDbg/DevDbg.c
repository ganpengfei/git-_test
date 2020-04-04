
#include "include.h"

static INT8U TxBuff[255];
static INT8U AckBuff[255];
static STypedefDbg DbgPara;

/***********************************************************************************
*Name      : DevDbg_Init
*Brief  	 : 调试模块初始化
*Parameter : NA
*return    : NA
***********************************************************************************/
void DevDbg_Init(void)
{

		/*调试驱动初始化*/
	DrvDbg_Init();
	
	DbgPara.UpdateFlsg = FALSE;

}
/***********************************************************************************
*Name      : DevDbg_Handle
*Brief  	 : 调试模块处理
*Parameter : NA
*return    : NA
***********************************************************************************/

void DevDbg_Handle(void)
{
	DrvDbg_Handle();
	DevDbg_TxHandle();
	DevDbg_RxHandle();
	
	
}
/***********************************************************************************
*Name      : DevDbg_TxHandle
*Brief  	 : 定时发送数据
*Parameter : NA
*return    : NA
***********************************************************************************/
void DevDbg_TxHandle(void)
{

}

/***********************************************************************************
*Name      : DevDbg_TxHandle
*Brief  	 : 定时发送数据
*Parameter : NA
*return    : NA
***********************************************************************************/
void DevDbg_TxPara(void)
{
	union
	{
		INT8U Data8[4];
		INT32U Data32;
		FLOAT32S Dataf;
	}UData;
	AckBuff[0] = DBG_ACU_TIMER_TO_PC;
	
	UData.Dataf = DevDbg_InqPara(DbgPara.TxPara1);
	AckBuff[1] = UData.Data8[0];
	AckBuff[2] = UData.Data8[1];
	AckBuff[3] = UData.Data8[2];
	AckBuff[4] = UData.Data8[3];
	
	UData.Dataf = DevDbg_InqPara(DbgPara.TxPara2);
	AckBuff[5] = UData.Data8[0];
	AckBuff[6] = UData.Data8[1];
	AckBuff[7] = UData.Data8[2];
	AckBuff[8] = UData.Data8[3];
	if(DrvDbg_InqTxQueueStatus() != BUSY)
			DevDbg_TxPack(AckBuff,9,ENABLE);
}
/***********************************************************************************
*Name      : DevDbg_TxHandle
*Brief  	 : 定时发送数据
*Parameter : NA
*return    : NA
***********************************************************************************/
FLOAT32S DevDbg_InqPara(INT8U paraId)
{
	FLOAT32S ReturnData;
	switch(paraId)
	{
		case 0:
			//ReturnData = CtlCmd_InqTaskCnt();
			break;
		case 1:
		case 2:
		case 3:
		case 4:
		case 5:
		case 6:
		case 7:
		case 8:
			//ReturnData = DevMotor_InqEnc(paraId - 1);
			break;
		case 9:
		case 10:
		case 11:
		case 12:
		case 13:
		case 14:
		case 15:
		case 16:
			//ReturnData = DevMotor_InqSpeed(paraId - 9);
			break;
		case 17:
			//ReturnData = (FLOAT32S)DrvEncode_InqEnc();
			break;
		default:ReturnData = 0;break;
	}
	return ReturnData;
}
/***********************************************************************************
*Name      : DevDbg_RxHandle
*Brief  	 : 调试模块数据接收
*Parameter : NA
*return    : NA
***********************************************************************************/
void DevDbg_RxHandle(void)
{
	static INT8U TempData[255];

	INT8U DataLength;
	INT8U AckLength;
	




	if(DrvTimer_InqEventStatus(TIMER_EVENT_MCU_RESET) == TRUE)
	{
		DrvTimer_StopEvent(TIMER_EVENT_MCU_RESET);
		NVIC_SystemReset();
	}

	
	AckLength = 0;
	
	if(TRUE == DrvDbg_InqPackStatus(&DataLength))
	{
		DrvDbg_InqPackData(TempData,DataLength);
		switch(TempData[2])
		{
			case MCU_RESET:
					AckBuff[0] = TempData[2];
					AckBuff[1] = TRUE;
					AckLength = 2;
					DrvTimer_StartEvent(TIMER_EVENT_MCU_RESET,20);
				break;
			case MCU_INQ:
					DevDbg_PrintStringHDLC(DevAcu_InqVersion(),ENABLE);
							break;
			case CODE_UPDATE_CONNECT:
			case CODE_UPDATE_BIGIN:
			case CODE_UPDATE_WORKING:
			case CODE_UPDATE_STOP:
			case CODE_UPDATE_ACTIVE:
			case CODE_UPDATE_ACTIVE_LATER:	
								AckBuff[0] = TempData[2];
								DevDbg_UpData(&TempData[2],AckBuff,&AckLength);
				break;


	
			
			
			default:break;
		}
	}
	
	if(AckLength != 0)
	{
		if(DrvDbg_InqTxQueueStatus() != BUSY)
			DevDbg_TxPack(AckBuff,AckLength,ENABLE);
	}

}
/***********************************************************************************
*Name      : DevDbg_PrintString
*Brief  	 : 打印字符串
*Parameter : INT8U* Data 字符串首地址
*return    : NA
***********************************************************************************/
void DevDbg_PrintString(INT8U* Data)
{
	INT8U i;
	
	for(i=0;i<255;i++)
	{
		if(Data[i] == 0x00)
			break;
	}
	
	DrvTimer_StartEvent(TIMER_EVENT_TIMEOUT,100);
	while((DrvUart_InqUartStatus(DBG_UART) != IDLE)&&(TRUE != DrvTimer_InqEventStatus(TIMER_EVENT_TIMEOUT)));
	DrvDbg_Buff(Data,i);

}
/***********************************************************************************
*Name      : DevDbg_PrintString
*Brief  	 : 打印字符串 HDLC
*Parameter : INT8U* Data 字符串首地址
*return    : NA
***********************************************************************************/
void DevDbg_PrintStringHDLC(INT8U* Data,INT8U queueEnable)
{
	INT8U i;
	
	if(CONFIG_BOOT_PRINT_ENABLE != ENABLE)
		return;
	
	AckBuff[0] = PRINT_STRING;
	for(i=0;i<255;i++)
	{
		AckBuff[i+1] = Data[i]; 
		if(Data[i] == 0x00)
			break;
	}
	DevDbg_TxPack(AckBuff,i+1,queueEnable);
}
/***********************************************************************************
*Name      : DevDbg_TxPack
*Brief  	 : DBG发送打包函数
*Parameter : INT8U* Data 字符串首地址
             INT8U Length 长度
*return    : NA
***********************************************************************************/
void  DevDbg_TxPack(INT8U *Buff,INT8U Length,INT8U queueEnable)
{
	INT8U i;
	INT8U j;
	
	i = 0;
	TxBuff[i++] = Length + 1;
	TxBuff[i++] = DRVDBG_ADD;
	for(j=0;j<Length;j++)
	{
		TxBuff[i++] = Buff[j];
	}
	DrvDbg_TxPack(TxBuff,i,queueEnable);
}
/*
*Name      : DevDbg_InqFlag
*Brief  	 : 查询升级标志
*Parameter : NA
*return    : DbgPara.UpdateFlsg;
*/
INT8U  DevDbg_InqFlag(void)
{
	return DbgPara.UpdateFlsg;
}
/*
*Name      : DevDbg_UpData
*Brief  	 : 升级代码处理函数
*Parameter : INT8U *Buff,
							INT8U *AckBuff,  回复缓存
							INT8U *AckLength 回复长度
*return    : NA
*/
static INT8U BuffCache[255];
static INT8U BuffCacheCnt = 0;
INT8U  DevDbg_UpData(INT8U *Buff,INT8U *AckBuff,INT8U *AckLength)
{
	INT32U TempBuff[6];
	INT8U Return;
	static INT32U Size = 0;
	static INT32U Sum = 0;
	static INT32U FileNum = 0;
	static INT8U Device = DEVICE_MCU;
	INT32U FileNumTemp ;
	
	Return = SUCC;
	switch(Buff[0])
	{
		case CODE_UPDATE_CONNECT ://升级前链接
					AckBuff[0] = Buff[0];
					AckBuff[1] = SUCC;
					*AckLength = 2;
					DbgPara.UpdateFlsg = TRUE;
			break;
		case CODE_UPDATE_BIGIN ://开始升级,擦除对应FLASH区域
					DevDbg_ByteToInt(&Buff[2],&TempBuff[0],2);
					Size = TempBuff[0];
					Sum = TempBuff[1];
					Device = Buff[1];
					DrvUpData_SetCodeInf(Size,Sum,Device);
					AckBuff[0] = Buff[0];
					AckBuff[1] = DrvUpData_Begin();
					*AckLength = 2;
					FileNum = 1;
					BuffCacheCnt = 0;
		
					DbgPara.UpdateFlsg = TRUE;
					break;
		
		case CODE_UPDATE_WORKING ://升级中
					FileNumTemp = ((INT32U)Buff[2]) | ((INT32U)Buff[3] << 8);
					if(FileNumTemp == FileNum )	
					{
						if(BuffCacheCnt != 0)
							DrvUpData_WriteBuff(BuffCache,BuffCacheCnt);
						BuffCacheCnt = Buff[1];
						for(INT8U i =0;i<BuffCacheCnt;i++)
						{
							BuffCache[i] = Buff[i + 4];
						}
						AckBuff[0] = Buff[0];
						AckBuff[1] = SUCC;
						*AckLength = 2;
						FileNum ++;
					}	
					else if(FileNumTemp == (FileNum - 1))		
					{
						BuffCacheCnt = Buff[1];
						for(INT8U i =0;i<BuffCacheCnt;i++)
						{
							BuffCache[i] = Buff[i + 4];
						}
						AckBuff[0] = Buff[0];
						AckBuff[1] = SUCC;
						*AckLength = 2;
						//FileNum ++;
					}	
					else
					{
						AckBuff[0] = Buff[0];
						AckBuff[1] = FAIL;
						*AckLength = 2;
					}	
					break;
		case CODE_UPDATE_STOP ://升级完成
					if(BuffCacheCnt != 0)
							DrvUpData_WriteBuff(BuffCache,BuffCacheCnt);
					AckBuff[0] = Buff[0];
					if(Device == DEVICE_MCU)
						TempBuff[0] = DrvUpData_CheckSum(FLASH_ADD_CACHE,Size);
					else
						TempBuff[0] = DrvUpData_CheckSum(FLASH_ADD_FPGA,Size);

					DevDbg_IntToByte(&AckBuff[1],&TempBuff[0],1);
					*AckLength = 5;

		
					break;
		case CODE_UPDATE_ACTIVE ://激活对应设备
			if(FileNum < 10)
			{
					AckBuff[0] = Buff[0];
					AckBuff[1] = FAIL;
					*AckLength = 2;
			}
			else
			{
					AckBuff[0] = Buff[0];
					AckBuff[1] = DrvUpData_SetFlag(0x55555555);//
					*AckLength = 2;
					DrvTimer_StartEvent(TIMER_EVENT_MCU_RESET,200);
			}
				
					break;
		case CODE_UPDATE_ACTIVE_LATER ://下次掉电再激活
			if(FileNum < 10)
			{
					AckBuff[0] = Buff[0];
					AckBuff[1] = FAIL;
					*AckLength = 2;
			}
			else
			{
					AckBuff[0] = Buff[0];
					AckBuff[1] = DrvUpData_SetFlag(0x55555555);//
					*AckLength = 2;
			}
				
					break;	
		default:Return = FAIL;break;
	}
	
	return Return;
}

/*
*Name      : DevDbg_ByteToInt
*Brief  	 : DBG吧字节转成words
*Parameter : INT8U *Buff8
             INT32U Buff32
							INT8U Cnt
*return    : NA
***********************************************************************************/
void  DevDbg_ByteToInt(INT8U *Buff8,INT32U *Buff32,INT8U Cnt)
{
	INT8U i;
	
	for(i=0;i<Cnt;i++)
	{
		Buff32[i] = *(INT32U*)(&Buff8[0] + i*4);
	}
}
/***********************************************************************************
*Name      : DevDbg_ByteToInt
*Brief  	 : DBG吧字节转成words
*Parameter : INT8U *Buff8
             INT32U Buff32
							INT8U Cnt
*return    : NA
***********************************************************************************/
void  DevDbg_ByteToFLOAT(INT8U *Buff8,FLOAT32S *Buff32,INT8U Cnt)
{
	INT8U i;
	union 
	{
		INT8U Data8[4];
		INT32U Data32;
		FLOAT32S Dataf;
	}UData;	
	
	for(i=0;i<Cnt;i++)
	{
		UData.Data8[0] = Buff8[i*4+0];
		UData.Data8[1] = Buff8[i*4+1];
		UData.Data8[2] = Buff8[i*4+2];
		UData.Data8[3] = Buff8[i*4+3];
		Buff32[i] = UData.Dataf;
	}
}
/***********************************************************************************
*Name      : DevDbg_ByteToInt
*Brief  	 : DBG吧INT 转到 byte
*Parameter : INT8U *Buff8
             INT32U Buff32
							INT8U Cnt
*return    : NA
***********************************************************************************/
void  DevDbg_IntToByte(INT8U *Buff8,INT32U *Buff32,INT8U Cnt)
{
	INT8U i;
	union{
		INT8U Data8[4];
		INT32U Data32;
	}UData;
	
	for(i=0;i<(Cnt);i++)
	{
		UData.Data32 = Buff32[i];
		Buff8[i*4+0] = UData.Data8[0];
		Buff8[i*4+1] = UData.Data8[1];
		Buff8[i*4+2] = UData.Data8[2];
		Buff8[i*4+3] = UData.Data8[3];
	}
}





