#ifndef _DRVTIMER_
#define _DRVTIMER_



#define EVNT_NUM        ((INT8U)5)                    //��ʱ�¼���Ŀ Ӧ��С��30

#define TIMER_EVENT_APP 			((INT8U)0)        //ϵͳ��ʱ�¼�
#define TIMER_EVENT_MCU_RESET ((INT8U)1)              //MCU��λ�¼�
#define TIMER_EVENT_TIMEOUT     ((INT8U)2)            //MCU��λ�¼�



#define DrvTimer_1msIsr  PIT0_IRQHandler


typedef struct
{
	INT32U Cnt;
	INT32U TargetCnt;
	INT8U Triger;
	INT8U Enable;
}TypeDef_Timer;

void DrvTimer_Init(void);
void DrvTimer_InitConfig(void);

INT8U DrvTimer_InqEventStatus(INT8U Num);
void DrvTimer_StartEvent(INT8U Num,INT32U Time);
void DrvTimer_StopEvent(INT8U Num);
INT32U DrvTimer_InqSysTime(void);
void DrvTimer_Delayms(INT32U time);
void DrvTimer_Stop(void);














#endif
