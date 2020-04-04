
#include "include.h"


static STypeFlash FlashPara;
static  INT32U WriteBuff[70];


/*
*Name      : DrvUpData_Init
*Brief  	 : 自动升级程序 参数初始化
*Parameter : NA
*return    : NA
*/
void DrvUpData_Init(void)
{
	FlashPara.CodeSize = 0;
	FlashPara.CodeSum = 0;
	FlashPara.Device = DEVICE_MCU;
	FlashPara.NowAdd = FLASH_ADD_CACHE;
}
/*
*Name      : DrvUpData_Init
*Brief  	 : 自动升级程序 参数设置
*Parameter : NA
*return    : NA
*/
void DrvUpData_SetCodeInf(INT32U Size,INT32U CheckSum,INT8U device)
{
	FlashPara.CodeSize = Size;
	FlashPara.CodeSum = CheckSum;
	FlashPara.Device = device;
	if(FlashPara.Device == DEVICE_MCU)
		FlashPara.NowAdd = FLASH_ADD_CACHE;
	else
		FlashPara.NowAdd = FLASH_ADD_FPGA;
}
/*
*Name      : DrvUpData_Begin
*Brief  	 : 自动升级程序 擦缓存扇区
*Parameter : NA
*return    : 擦除成功或失败，TRUE or FALSE
*/
INT8U DrvUpData_Begin(void)
{
	if(FlashPara.Device == DEVICE_MCU)
		return DrvFlash_Erase(FLASH_ADD_CACHE,FLASH_SIZE_CACHE);
	else
		return DrvFlash_Erase(FLASH_ADD_FPGA,FLASH_SIZE_FPGA);
}
/*
*Name      : DrvUpData_WriteBuff
*Brief  	 : 自动升级程序 写数据
*Parameter : INT32U* Buff 写数据地址 
							INT8U Length 数据长度   per bytes
*return    : 写成功或失败，TRUE or FALSE
*/
INT8U DrvUpData_WritePack(INT32U* Buff,INT8U Length)
{
	INT8U result;

	result = SUCC;
	if(Length % 8)
	{
		return FAIL;//FLASH error
	}
//	if((FlashPara.NowAdd < FLASH_ADD_CACHE) || (FlashPara.NowAdd > FLASH_ADD_CACHE + FLASH_SIZE_CACHE))
//		return FAIL;//FLASH error
	result = DrvFlash_WriteBuff(FlashPara.NowAdd, &Buff[0], Length);
	FlashPara.NowAdd += Length;
	
	
	return result;
}
/*
*Name      : DrvUpData_WriteBuff
*Brief  	 : 自动升级程序 写数据 打包
*Parameter : INT8U* Buff 写数据地址 
							INT8U Length 数据长度   per bytes
*return    : 写成功或失败，TRUE or FALSE
*/
INT8U DrvUpData_WriteBuff(INT8U* Buff,INT8U Length)
{
	INT8U i;
	INT8U TempData;
	INT8U DataCnt;;
	
	DataCnt = Length;
	
	
	TempData = DataCnt % 8;
	if( TempData != 0)
	{
		for(i=0;i<TempData;i++)
		{
			Buff[DataCnt + i] = 0;
		}
		DataCnt = DataCnt + (8 - TempData);
	}
	for(i=0;i<DataCnt;i++)
	{
		*((INT8U*)(&WriteBuff[0])+i) = Buff[i];
	}


	return DrvUpData_WritePack(WriteBuff,DataCnt);
}

/*
*Name      : DrvUpData_CheckSum
*Brief  	 : 自动升级程序 检查程序正确性 
*Parameter : NA
*return    : 返回校验
*/
INT32U DrvUpData_CheckSum(INT32U Add,INT32U Size)
{	
	INT32U Sum;
	INT32U i;
	
	Sum = 0;
	for(i=0;i<Size;i++)
	{
		Sum += *((INT8U*)(Add + i));
	}
	return Sum;	
}
/*
*Name      : DrvUpData_SetFlag
*Brief  	 : 自动升级程序 写FLAG
*Parameter : INT8U Flag 
*return    : 返回写入标志
*/
INT8U DrvUpData_SetFlag(INT32U Flag)
{	
	INT8U result;
	
	WriteBuff[0] = Flag;
	WriteBuff[1] = FlashPara.CodeSize;
	WriteBuff[2] = FlashPara.CodeSum;
	WriteBuff[3] = Flag + FlashPara.CodeSize + FlashPara.CodeSum;
	if(FlashPara.Device == DEVICE_MCU)
	{
		DrvFlash_Erase(FLASH_ADD_FLAG,FLASH_SIZE_FLAG);
		result = DrvFlash_WriteBuff(FLASH_ADD_FLAG, &WriteBuff[0], 16);
	}
	else
	{
		DrvFlash_Erase(FLASH_ADD_FPGA_FLAG,FLASH_SIZE_FLAG);
		result = DrvFlash_WriteBuff(FLASH_ADD_FPGA_FLAG, &WriteBuff[0], 16);
	}		

	return result;
}















