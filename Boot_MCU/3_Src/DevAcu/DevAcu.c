#include "include.h"

static INT8U CodeVersion[50];
static ETypeDev NowDevice;
/***********************************************************************************
*Name      : DevAcu_Init
*Brief  	 : 自动控制单元，核心板有关外设在这儿初始化，执行一次
*Parameter : NA
*return    : NA
***********************************************************************************/
void DevAcu_Init(void)
{
	/*MCU环境初始化*/
	DrvCfg_Init();    
	/*MCU定时器初始化（1ms）*/
	DrvTimer_Init();
	DrvTimer_Delayms(50);
	/*外设模块配置*/
	DevAcu_DriverInit();
	/*驱动板指示灯*/
	//DevAcu_LedInit();
	/*升级模块初始化*/
	DrvUpData_Init();	
	
	CtlBoot_Init();
	if(ENABLE == CONFIG_BOOT_FPGA_ENABLE)
		CtlFpga_Init();
	 //通信模块初始化  
	DevDbg_Init();                              
}


/***********************************************************************************
*Name      : DevAcu_Handle
*Brief  	 : 自动控制单元钩子函数，扫描一直执行
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
*Brief  	 : LED初始化
*Parameter : NA
*return    : NA
***********************************************************************************/
void DevAcu_LedInit(void)
{
	DrvCfg_GpioInit(GPIOE,27u,kGPIO_DigitalOutput,0);
}

/***********************************************************************************
*Name      : DevAcu_LedHandle
*Brief  	 : LED控制
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
*Brief  	 : 打印版本信息
*Parameter :  INT8U *Name：
							INT8U *Data: 日期
							INT8U *Time: 时间
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
*Brief  	 : 查询版本信息
*Parameter :  NA
*return    : INT8U *
***********************************************************************************/
INT8U *DevAcu_InqVersion(void)
{
	return &CodeVersion[0];  
}
/***********************************************************************************
*Name      : DevAcu_DriverInit
*Brief  	 : 初始化各个驱动
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
*Brief  	 : 关闭各个驱动
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
*Brief  	 : 设置当前通信设备
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


