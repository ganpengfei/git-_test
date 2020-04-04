#include "include.h"

static TypeDef_Timer TimerEvent[EVNT_NUM];
static INT32U SysTime = 0;
/***********************************************************************************
*Name      : DrvTimer_InitConfig
*Brief  	 : ��ʱ����ʼ��
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
*Brief  	 : ��ʱ��ֹͣ
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
*Brief  	 : 1ms��ʱ������
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
*Brief  	 : 1ms��ʱʱ���жϴ������
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
*Brief  	 : ��ѯ��ʱ�¼�״̬
*Parameter : INT8U Num        ��ʱ�¼����
*return    : INT8U       �Ƿ񴥷�
***********************************************************************************/
INT8U DrvTimer_InqEventStatus(INT8U Num)
{
	return TimerEvent[Num].Triger;
}
/***********************************************************************************
*Name      : DrvTimer_startEvent
*Brief  	 : ��ʼĳ����ʱ�¼�
*Parameter : INT8U Num       ��ʱ�¼����
			 INT32U Time     ��ʱʱ��
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
*Brief  	 :ֹͣĳ����ʱ�¼�
*Parameter : INT8U Num       ��ʱ�¼����	
*return    : NA
***********************************************************************************/
void DrvTimer_StopEvent(INT8U Num)
{
	TimerEvent[Num].Enable = FALSE;
	TimerEvent[Num].Triger = FALSE;
}
/***********************************************************************************
*Name      : DrvTimer_InqSysTime
*Brief  	 : ��ѯϵͳʱ��
*Parameter : NA
*return    : SysTime
***********************************************************************************/
INT32U DrvTimer_InqSysTime(void)
{
	return SysTime;
}
/***********************************************************************************
*Name      : DrvTimer_Delayms
*Brief  	 :������ʱ
*Parameter : INT32U time   (ms)
*return    : NA
***********************************************************************************/	
void DrvTimer_Delayms(INT32U time)
{
	static INT32U LastTime;
	LastTime = SysTime;
	while((SysTime - LastTime) <= time);
}

