
#include "include.h"

const INT8U CodeTime[] = __TIME__;            		// ��ȡϵͳ����ʱ��
const INT8U CodeData[] = __DATE__;             		// ��ȡϵͳ��������
const INT8U CodeName[] = CONFIG_BOOT_VERSION;    	// ����汾�� ��config.h�ļ���������ģʽ���޸�

int main(void)
{
	DevAcu_Init();                              						 // ���İ��ʼ����
	
	DevDbg_PrintStringHDLC((INT8U*)"Boot Running...",DISABLE);
	
	DrvTimer_Delayms(20);                       
																		// ��ӡ����汾��Ϣ
	DevAcu_PrintVersion((INT8U*)&CodeName,(INT8U*)&CodeData,(INT8U*)&CodeTime);		
	DrvTimer_Delayms(20);


	if(CONFIG_BOOT_ENABLE != ENABLE)            						// ûʹ��Bootֱ������APP
		CtlBoot_JumpToUserApplication(FLASH_ADD_APP,FLASH_ADD_APP + 4);
	
	DrvTimer_StartEvent(TIMER_EVENT_APP,CONFIG_BOOT_RUN_TIME);          //��ʼ��ʱʱ��---BOOT�ȴ�ʱ���¼�
	while(1)                                    
	{
		DevAcu_Handle();

		if((DrvTimer_InqEventStatus(TIMER_EVENT_APP) == TRUE)			// Boot�ȴ�������ʱ������APP
			&& (DevDbg_InqFlag() == FALSE))
		{
			DrvTimer_StopEvent(TIMER_EVENT_APP);                 
			DevDbg_PrintStringHDLC((INT8U*)"Time Out!",DISABLE);
			DrvTimer_Delayms(10);
			if(ENABLE == CONFIG_BOOT_FPGA_ENABLE)
				CtlFpga_Handle();                                 		// ����FPGA
			CtlBoot_Handle();                                     		// ����APP
			DevDbg_PrintStringHDLC((INT8U*)"Jump to App Fail!",DISABLE);// ����ʧ��
		}
	}

}







