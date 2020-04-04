
#include "include.h"


static STypeDef_BootPara BootPara;
/***********************************************************************************
*Name      : CtlBoot_Init
*Brief  	 : MCu BooT
*Parameter : NA
*return    : NA
***********************************************************************************/

void CtlBoot_Init(void)
{
	BootPara.RunStatus = 0;
}
/***********************************************************************************
*Name      : CtlBoot_Handle
*Brief  	 : MCu BooT 处理
*Parameter : NA
*return    : NA
***********************************************************************************/

void CtlBoot_Handle(void)
{
	INT32U Sum;
	INT32U DataBuff[100];
	INT32U NowAdd;
			
	DrvFlash_ReadBuff(FLASH_ADD_FLAG,DataBuff,16); 
	BootPara.CodeFlag = DataBuff[0];
	BootPara.CodeSize = DataBuff[1];
	BootPara.CodeSum =  DataBuff[2];
	BootPara.FlagSum =  DataBuff[3];
	
	Sum = BootPara.CodeFlag + BootPara.CodeSize + BootPara.CodeSum;

	if((BootPara.CodeFlag == 0x55555555) && (Sum == BootPara.FlagSum))
	{	
		DataBuff[0] = 0xffffffff;
		DataBuff[1] = 0xffffffff;
		DataBuff[2] = 0xffffffff;
		DataBuff[3] = 0xffffffff;
		DrvFlash_Erase(FLASH_ADD_FLAG,FLASH_SIZE_FLAG);
		DrvFlash_WriteBuff(FLASH_ADD_FLAG, DataBuff, 16);
		
		Sum = DrvUpData_CheckSum(FLASH_ADD_CACHE,BootPara.CodeSize);
		if(Sum == BootPara.CodeSum )//校验通过开始拷贝程序
		{
			DrvFlash_Erase(FLASH_ADD_APP,FLASH_SIZE_APP);  
			NowAdd = 0;				
			while(NowAdd < BootPara.CodeSize) //拷贝程序到运行区域
			{
				DrvFlash_ReadBuff(FLASH_ADD_CACHE+NowAdd,DataBuff,240); 
				if((FAIL == DrvFlash_WriteBuff(FLASH_ADD_APP+NowAdd, DataBuff, 240))
						||(NowAdd > FLASH_SIZE_APP))
				{
					return;            //error ： can't recover
				}
				NowAdd += 240;
			}
			Sum = DrvUpData_CheckSum(FLASH_ADD_APP,BootPara.CodeSize);
			if(Sum == BootPara.CodeSum) //校验通过程序可以运行    
			{
				CtlBoot_JumpToUserApplication(FLASH_ADD_APP,FLASH_ADD_APP + 4);
			}
			else
			{
				return;            //error ： can't recover
			}	
			

		}
		else                       //校验不通过运行源程序
		{
			CtlBoot_JumpToUserApplication(FLASH_ADD_APP,FLASH_ADD_APP + 4);
		}	
	}
	else                         //没有缓存数据 运行源程序
	{
		CtlBoot_JumpToUserApplication(FLASH_ADD_APP,FLASH_ADD_APP + 4);

	}
}
/***********************************************************************************
*Name      : CtlBoot_JumpToUserApplication
*Brief  	 : MCu BooT 跳转
*Parameter : NA
*return    : NA
***********************************************************************************/

void CtlBoot_JumpToUserApplication(INT32U userSP, INT32U userStartup)
{
	if(*(INT32U*)(FLASH_ADD_APP) != 0xFFFFFFFF)//有程序
	{		
		DevDbg_PrintStringHDLC((INT8U*)"Jump To App!",DISABLE);
		DrvTimer_Delayms(10);
		DevAcu_DriverStop();
		
		DisableGlobalIRQ();
			// Turn off interrupts.
		__disable_irq();

		// Set the VTOR to default.
		SCB->VTOR = userSP;

		// Memory barriers for good measure.
		__ISB();
		__DSB();

		// Set main stack pointer and process stack pointer.
		__set_MSP(*((uint32_t*)userSP));

		__set_PSP(*((uint32_t*)userSP));

		// Jump to flashloader entry point, does not return.
		void (*entry)(void) = (void (*)(void))(*((uint32_t*)userStartup));
		entry();
	}
	else
		return;
	
	/*
	__set_MSP(userSP);
	__set_PSP(userSP);
	static void (*farewellBootloader)(void) = 0;
	farewellBootloader = userStartup；
	farewellBootloader();
	
	// set up stack pointer
  __asm("msr msp, r0");
  __asm("msr psp, r0");
  // Jump to PC (r1)
  __asm("mov pc, r1"); 
	*/
}



















