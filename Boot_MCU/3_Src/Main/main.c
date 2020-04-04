
#include "include.h"

const INT8U CodeTime[] = __TIME__;            		// 获取系统编译时间
const INT8U CodeData[] = __DATE__;             		// 获取系统编译日期
const INT8U CodeName[] = CONFIG_BOOT_VERSION;    	// 软件版本号 在config.h文件交互界面模式下修改

int main(void)
{
	DevAcu_Init();                              						 // 核心板初始化化
	
	DevDbg_PrintStringHDLC((INT8U*)"Boot Running...",DISABLE);
	
	DrvTimer_Delayms(20);                       
																		// 打印软件版本信息
	DevAcu_PrintVersion((INT8U*)&CodeName,(INT8U*)&CodeData,(INT8U*)&CodeTime);		
	DrvTimer_Delayms(20);


	if(CONFIG_BOOT_ENABLE != ENABLE)            						// 没使能Boot直接跳到APP
		CtlBoot_JumpToUserApplication(FLASH_ADD_APP,FLASH_ADD_APP + 4);
	
	DrvTimer_StartEvent(TIMER_EVENT_APP,CONFIG_BOOT_RUN_TIME);          //开始定时时间---BOOT等待时间事件
	while(1)                                    
	{
		DevAcu_Handle();

		if((DrvTimer_InqEventStatus(TIMER_EVENT_APP) == TRUE)			// Boot等待升级超时，跳到APP
			&& (DevDbg_InqFlag() == FALSE))
		{
			DrvTimer_StopEvent(TIMER_EVENT_APP);                 
			DevDbg_PrintStringHDLC((INT8U*)"Time Out!",DISABLE);
			DrvTimer_Delayms(10);
			if(ENABLE == CONFIG_BOOT_FPGA_ENABLE)
				CtlFpga_Handle();                                 		// 加载FPGA
			CtlBoot_Handle();                                     		// 加载APP
			DevDbg_PrintStringHDLC((INT8U*)"Jump to App Fail!",DISABLE);// 加载失败
		}
	}

}







