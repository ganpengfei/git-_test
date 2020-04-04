#include "include.h"

static INT8U CodeVersion[50];
static ETypeDev NowDevice;
/***********************************************************************************
*Name      : DevAcu_Init
*Brief  	 : �Զ����Ƶ�Ԫ�����İ��й������������ʼ����ִ��һ��
*Parameter : NA
*return    : NA
***********************************************************************************/
void DevAcu_Init(void)
{
	/*MCU������ʼ��*/
	DrvCfg_Init();    
	/*MCU��ʱ����ʼ����1ms��*/
	DrvTimer_Init();
	DrvTimer_Delayms(50);
	/*����ģ������*/
	DevAcu_DriverInit();
	/*������ָʾ��*/
	//DevAcu_LedInit();
	/*����ģ���ʼ��*/
	DrvUpData_Init();	
	
	CtlBoot_Init();
	if(ENABLE == CONFIG_BOOT_FPGA_ENABLE)
		CtlFpga_Init();
	 //ͨ��ģ���ʼ��  
	DevDbg_Init();                              
}


/***********************************************************************************
*Name      : DevAcu_Handle
*Brief  	 : �Զ����Ƶ�Ԫ���Ӻ�����ɨ��һֱִ��
*Parameter : NA
*return    : NA
***********************************************************************************/
void DevAcu_Handle(void)
{
	DevDbg_Handle();
	//DevAcu_LedHandle();
}


/***********************************************************************************
*Name      : DevAcu_LedInit
*Brief  	 : LED��ʼ��
*Parameter : NA
*return    : NA
***********************************************************************************/
void DevAcu_LedInit(void)
{
	DrvCfg_GpioInit(GPIOE,27u,kGPIO_DigitalOutput,0);
}

/***********************************************************************************
*Name      : DevAcu_LedHandle
*Brief  	 : LED����
*Parameter : NA
*return    : NA
***********************************************************************************/
void DevAcu_LedHandle(void)
{
	static INT32U LastTime = 0;
	static INT8U LedStatus = 0;

	if(DrvTimer_InqSysTime() - LastTime > 30)
	{
		LastTime = DrvTimer_InqSysTime();
		LedStatus = !LedStatus;
		
		if(LedStatus)
		{
			LEDTEST = 1;
		}
		else
		{
			LEDTEST = 0;
		}
	}		
}
/***********************************************************************************
*Name      : DevAcu_PrintVersion
*Brief  	 : ��ӡ�汾��Ϣ
*Parameter :  INT8U *Name��
							INT8U *Data: ����
							INT8U *Time: ʱ��
*return    : NA
***********************************************************************************/

void DevAcu_PrintVersion(INT8U *Name,INT8U *Data,INT8U *Time)
{
	memset(CodeVersion,0,sizeof(CodeVersion));
	sprintf((char*)(&CodeVersion),"Version:%s(%s %s)",Name,Data,Time);
	DevDbg_PrintStringHDLC(CodeVersion,DISABLE);
}
/***********************************************************************************
*Name      : DevAcu_InqVersion
*Brief  	 : ��ѯ�汾��Ϣ
*Parameter :  NA
*return    : INT8U *
***********************************************************************************/
INT8U *DevAcu_InqVersion(void)
{
	return &CodeVersion[0];  
}
/***********************************************************************************
*Name      : DevAcu_DriverInit
*Brief  	 : ��ʼ����������
*Parameter :  NA
*return    : NA
***********************************************************************************/
void DevAcu_DriverInit(void)
{
	INT32U WriteBuff[4];
	
	if(CONFIG_BOOT_ENABLE != ENABLE)
		return;	

	if(CONFIG_COM0_ENABLE == ENABLE) 
		DrvUart_Init(UART0,CONFIG_COM0_BARDRATE,(uart_parity_mode_t)CONFIG_COM0_CHECK);

	if(CONFIG_COM1_ENABLE == ENABLE) 
		DrvUart_Init(UART1,CONFIG_COM1_BARDRATE,(uart_parity_mode_t)CONFIG_COM1_CHECK);

	if(CONFIG_COM2_ENABLE == ENABLE) 
		DrvUart_Init(UART2,CONFIG_COM2_BARDRATE,(uart_parity_mode_t)CONFIG_COM2_CHECK);
	
#ifdef CPU_MK66FN2M0VMD18	
	if(CONFIG_COM3_ENABLE == ENABLE) 
		DrvUart_Init(UART3,CONFIG_COM3_BARDRATE,(uart_parity_mode_t)CONFIG_COM3_CHECK);
	if(CONFIG_COM4_ENABLE == ENABLE) 
		DrvUart_Init(UART4,CONFIG_COM4_BARDRATE,(uart_parity_mode_t)CONFIG_COM4_CHECK);
#endif	
	if(CONFIG_CAN0_ENABLE == ENABLE) 
		DrvCan_Init(CAN0,CONFIG_CAN0_BARDRATE,CONFIG_CAN0_ID);
	if(CONFIG_CAN1_ENABLE == ENABLE) 
		DrvCan_Init(CAN1,CONFIG_CAN1_BARDRATE,CONFIG_CAN1_ID);
	
	
	DrvFlash_ReadBuff(FLASH_ADD_DATA ,&WriteBuff[0], 4);
	
	if(WriteBuff[0] > (INT32U)DEV_CAN1)
	{	
		NowDevice = DEV_UART0;
		DevAcu_SetDev(DEV_CAN0);
	}
	else
		NowDevice = (ETypeDev)WriteBuff[0];
}
/***********************************************************************************
*Name      : DevAcu_DriverStop
*Brief  	 : �رո�������
*Parameter :  NA
*return    : na
***********************************************************************************/
void DevAcu_DriverStop(void)
{
	if(CONFIG_COM0_ENABLE == ENABLE) 
		DrvUart_Stop(UART0);
	if(CONFIG_COM1_ENABLE == ENABLE) 
		DrvUart_Stop(UART1);
	if(CONFIG_COM2_ENABLE == ENABLE) 
		DrvUart_Stop(UART2);
#ifdef CPU_MK66FN2M0VMD18		
	if(CONFIG_COM3_ENABLE == ENABLE) 
		DrvUart_Stop(UART3);
	if(CONFIG_COM4_ENABLE == ENABLE) 
		DrvUart_Stop(UART4);
#endif	
	if(CONFIG_CAN0_ENABLE == ENABLE) 
		DrvCan_Stop(CAN0);
	if(CONFIG_CAN1_ENABLE == ENABLE) 
		DrvCan_Stop(CAN1);
	
	DrvTimer_Stop();
	DrvDbg_FreeLine();
	
}
/***********************************************************************************
*Name      : DevAcu_SetDev
*Brief  	 : ���õ�ǰͨ���豸
*Parameter :  ETypeDev
*return    : NA
***********************************************************************************/
void DevAcu_SetDev(ETypeDev device)
{
	INT32U WriteBuff[4];
	
	
	if(NowDevice != device)
	{
		NowDevice = device;
		WriteBuff[0] = NowDevice;
		DrvFlash_Erase(FLASH_ADD_DATA,FLASH_SIZE_FLAG);
		DrvFlash_WriteBuff(FLASH_ADD_DATA, &WriteBuff[0], 8);	
	}
}
ETypeDev DevAcu_InqDev(void)
{

	return NowDevice;
}


