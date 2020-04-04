#include "include.h"

static TypeDef_Timer TimerEvent[EVNT_NUM];
static INT32U SysTime = 0;
/***********************************************************************************
*Name      : DrvTimer_InitConfig
*Brief  	 : 定时器初始化
*Parameter : NA
*return    : NA
***********************************************************************************/
void DrvTimer_Init(void)
{
	INT8U i;
	for(i=0;i<EVNT_NUM;i++)
	{
		TimerEvent[i].Enable = FALSE;
		TimerEvent[i].Cnt = 0;
		TimerEvent[i].TargetCnt = 0;
		TimerEvent[i].Triger = FALSE;
	}		
	SysTime = 0;
	DrvTimer_InitConfig();
}
/***********************************************************************************
*Name      : DrvTimer_Stop
*Brief  	 : 定时器停止
*Parameter : NA
*return    : NA
***********************************************************************************/
void DrvTimer_Stop(void)
{
	PIT_DisableInterrupts(PIT, kPIT_Chnl_0, kPIT_TimerInterruptEnable);
	DisableIRQ(PIT0_IRQn);
	PIT_Deinit(PIT);
}
/***********************************************************************************
*Name      : DrvTimer_InitConfig
*Brief  	 : 1ms定时器配置
*Parameter : NA
*return    : NA
***********************************************************************************/
void DrvTimer_InitConfig(void)
{
	pit_config_t pitConfig;
			
	/*
	 * pitConfig.enableRunInDebug = false;
	 */
	PIT_GetDefaultConfig(&pitConfig);

	/* Init pit module */
	PIT_Init(PIT, &pitConfig);

	/* Set timer period for channel 0 */
	PIT_SetTimerPeriod(PIT, kPIT_Chnl_0, USEC_TO_COUNT(1000U, CLOCK_GetFreq(kCLOCK_BusClk)));

	/* Enable timer interrupts for channel 0 */
	PIT_EnableInterrupts(PIT, kPIT_Chnl_0, kPIT_TimerInterruptEnable);

	/* Enable at the NVIC */
	EnableIRQ(PIT0_IRQn);

	/* Start channel 0 */
	PIT_StartTimer(PIT, kPIT_Chnl_0);
	
	//NVIC_SetPriority(PIT0_IRQn,1);
}


/***********************************************************************************
*Name      : DrvTimer_1msIsr
*Brief  	 : 1ms定时时间中断处理程序
*Parameter : NA
*return    : NA
***********************************************************************************/
void DrvTimer_1msIsr(void)
{
	static INT8U i;
	PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, PIT_TFLG_TIF_MASK);
	SysTime++;
 	for(i=0;i<EVNT_NUM;i++)
	{
		if(TimerEvent[i].Enable == TRUE)
		{
			TimerEvent[i].Cnt ++;
			if(TimerEvent[i].Cnt >= TimerEvent[i].TargetCnt)
			{
				TimerEvent[i].Enable = FALSE;
				TimerEvent[i].Triger = TRUE;
				TimerEvent[i].Cnt = 0;
			}
		}
	}
}


/***********************************************************************************
*Name      : DrvTimer_InqEventStatus
*Brief  	 : 查询定时事件状态
*Parameter : INT8U Num        定时事件编号
*return    : INT8U       是否触发
***********************************************************************************/
INT8U DrvTimer_InqEventStatus(INT8U Num)
{
	return TimerEvent[Num].Triger;
}
/***********************************************************************************
*Name      : DrvTimer_startEvent
*Brief  	 : 开始某个定时事件
*Parameter : INT8U Num       定时事件编号
			 INT32U Time     定时时间
*return    : NA
***********************************************************************************/
void DrvTimer_StartEvent(INT8U Num,INT32U Time)
{
	TimerEvent[Num].Enable = TRUE;
	TimerEvent[Num].Cnt = 0;
	TimerEvent[Num].TargetCnt = Time;
	TimerEvent[Num].Triger = FALSE;
}
/***********************************************************************************
*Name      : DrvTimer_StopEvent
*Brief  	 :停止某个定时事件
*Parameter : INT8U Num       定时事件编号	
*return    : NA
***********************************************************************************/
void DrvTimer_StopEvent(INT8U Num)
{
	TimerEvent[Num].Enable = FALSE;
	TimerEvent[Num].Triger = FALSE;
}
/***********************************************************************************
*Name      : DrvTimer_InqSysTime
*Brief  	 : 查询系统时间
*Parameter : NA
*return    : SysTime
***********************************************************************************/
INT32U DrvTimer_InqSysTime(void)
{
	return SysTime;
}
/***********************************************************************************
*Name      : DrvTimer_Delayms
*Brief  	 :阻塞延时
*Parameter : INT32U time   (ms)
*return    : NA
***********************************************************************************/	
void DrvTimer_Delayms(INT32U time)
{
	static INT32U LastTime;
	LastTime = SysTime;
	while((SysTime - LastTime) <= time);
}

