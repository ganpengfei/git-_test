#include "include.h"

static STypeDef_FpgaPara FpgaPara;
/***********************************************************************************
*Name      : CtlFpga_Init
*Brief  	 : 跟新FPGA程序初始化
*Parameter : NA
*return    : NA
***********************************************************************************/
void CtlFpga_Init(void)
{
	FpgaPara.ReadAdd = FLASH_ADD_FPGA;
	
	
	DrvCfg_GpioInit(GPIOB,0u,kGPIO_DigitalOutput,1);// nCONFIG
	DrvCfg_GpioInit(GPIOD,12u,kGPIO_DigitalOutput,1);//DCLK
	DrvCfg_GpioInit(GPIOD,13u,kGPIO_DigitalOutput,1);//DATA
	
	DrvCfg_GpioInit(GPIOA,28u,kGPIO_DigitalInput,0);//CONFIGDONE
	DrvCfg_GpioInit(GPIOA,29u,kGPIO_DigitalInput,0);//nSTATUS
	
	nCONFIG = 1;
	DCLK = 1;
	DATA = 1;
}
/***********************************************************************************
*Name      : CtlFpga_Handle
*Brief  	 : 跟新FPGA程序
*Parameter : NA
*return    : NA
***********************************************************************************/
void CtlFpga_Handle(void)
{
	INT32U DataBuff[100];
	INT8U FpgaStatus;
	//INT32U MaskValue;
	INT8U i ;

	//MaskValue = DisableGlobalIRQ();
	
	DrvFlash_ReadBuff(FLASH_ADD_FPGA_FLAG,DataBuff,16); 
	FpgaPara.CodeFlag = DataBuff[0];
	FpgaPara.CodeSize = DataBuff[1];
	FpgaPara.CodeSum =  DataBuff[2];
	FpgaPara.FlagSum =  DataBuff[3];
	

	if(FpgaPara.CodeFlag == 0x55555555)//是否有代码
	{	
		DevDbg_PrintStringHDLC((INT8U*)"Fpga loading...",DISABLE);
		DrvTimer_Delayms(10);
		for(i=0;i<10;i++)         //如果不成功，跟新10次
		{
			FpgaStatus = CtlFpga_Updata();
			if(FpgaStatus == 0)     //成功，退出
			{
				break;
			}
			DevDbg_PrintStringHDLC((INT8U*)"Error,Reloading!",DISABLE);
		}
		DrvTimer_Delayms(10);
		switch(FpgaStatus)
		{
			case 0:DevDbg_PrintStringHDLC((INT8U*)"Fpga Load SUCC!",DISABLE);break;
			case 11:DevDbg_PrintStringHDLC((INT8U*)"Fpga No Ack!",DISABLE);break;
			case 12:DevDbg_PrintStringHDLC((INT8U*)"Fpga No Ack2!!",DISABLE);break;
			case 13:DevDbg_PrintStringHDLC((INT8U*)"Fpga Load error!",DISABLE);break;
			case 14:DevDbg_PrintStringHDLC((INT8U*)"Fpga Load error2!",DISABLE);break;
			default:DevDbg_PrintStringHDLC((INT8U*)"I'm cool!",DISABLE);break;
		}
		DrvTimer_Delayms(10);
	}	
	else
	{
		DevDbg_PrintStringHDLC((INT8U*)"No Fpga Code Found!",DISABLE);
		DrvTimer_Delayms(10);
	}

	//EnableGlobalIRQ(MaskValue);	
}
/***********************************************************************************
*Name      : CtlFpga_Updata
*Brief  	 : 跟新FPGA程序
*Parameter : NA
*return    : STATUS
***********************************************************************************/
INT8U CtlFpga_Updata(void)
{
	INT32U WriteData;
	INT8U i;
	INT32U WaitTime;
	
	nCONFIG = LOW;
	DCLK = LOW;
	
	WaitTime = 0;
	while(nSTATUS != LOW)
	{
		WaitTime ++;	
		if(WaitTime > 10000)
			return 11;//FAIL
	}
	
	nCONFIG = HIGH;
	while(nSTATUS != HIGH)
	{
		WaitTime ++;
		if(WaitTime > 10000)
			return 12;//FAIL
	}
	CtlFpga_Delayus(2);
	FpgaPara.ReadAdd = FLASH_ADD_FPGA;
	while(FpgaPara.ReadAdd < FLASH_ADD_FPGA + FpgaPara.CodeSize)//加载程序
	{
		WriteData = *((INT32U*)(FpgaPara.ReadAdd));
		for(i=0;i<32;i++)
		{
			if(WriteData & 0x00000001)
				DATA = 1;
			else
				DATA = 0;
			__nop();
			__nop();
			DCLK = 1;
			__nop();
			__nop();
			__nop();
			__nop();
			__nop();
			DCLK = 0;
			__nop();
			__nop();
			if(!nSTATUS)
			{
				return 13;//FAIL
			}	
			WriteData = WriteData >> 1;
		}
		FpgaPara.ReadAdd += 4;
	}
	CtlFpga_Delayus(20);
	if(CONFIGDONE == 0)
	{
		return 14;//FAIL
	}
	for(i=0;i<11;i++)//发10个脉冲 使FPGA运行
	{
			DCLK = 1;
			__nop();
			__nop();
			__nop();
			__nop();
			DCLK = 0;
			__nop();
			__nop();
			__nop();
			__nop();
	}
		
	return 0;//SUCC
}
/***********************************************************************************
*Name      : CtlFpga_Delayus
*Brief  	 : 延时us
*Parameter : INT32U cnt        cnt = 200 延时 1ms     
*return    : NA
***********************************************************************************/
void CtlFpga_Delayus(INT32U cnt)
{
	INT32U i;
	INT32U j;
	for(i=0;i<cnt;i++)
	{
		for(j=0;j<120;j++)
			__nop();
	}
}	















