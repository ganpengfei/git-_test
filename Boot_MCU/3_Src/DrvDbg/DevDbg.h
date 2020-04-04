
#ifndef _DEVDBG_
#define _DEVDBG_


#define  DBG_UART        UART1
#define  DBG_UART_BPS    115200
#define  DBG_UART_PARITY UART_PARITY_NONE

#define  DBG_PORT_UART        ((INT8U)0)
#define  DBG_PORT_CAN         ((INT8U)1)


#define  MCU_RESET         				((INT8U)0x00)// MCU reset
#define  CODE_UPDATE_BIGIN       	((INT8U)0x01)// 自动升级
#define  CODE_UPDATE_WORKING      ((INT8U)0x02)// 自动升级
#define  CODE_UPDATE_STOP       	((INT8U)0x03)// 自动升级
#define  CODE_UPDATE_ACTIVE       ((INT8U)0x04)// 自动升级
#define  MCU_INQ         				  ((INT8U)0x05)// MCU inq vision
#define  CODE_UPDATE_ACTIVE_LATER  ((INT8U)0x20)// 自动升级
#define  CODE_UPDATE_CONNECT  ((INT8U)0x21)// 自动升级
#define  DBG_ACU_RUN_STATUS       ((INT8U)0x06)// 
#define  DBG_ACU_TIMER_OUT        ((INT8U)0x07)// 
#define  DBG_ACU_TIMER_PARA       ((INT8U)0x08)// 
#define  DBG_ACU_TIMER_TO_PC      ((INT8U)0x09)//  

#define DBG_NANUAL_SPEED          ((INT8U)0x10)//设置电机速度
#define DBG_NANUAL_DISTANCE       ((INT8U)0x11)//设置电机步进
#define DBG_NANUAL_EM       			((INT8U)0x12)//设置电磁铁


#define PRINT_STRING              ((INT8U)0x80)  
#define DBG_CTLCMD_TEST           ((INT8U)0x81)  








typedef struct
{
	INT8U DbgDevice;
	INT8U EnTimerOut;
	INT8U TxPeriod;
	INT8U TxPara1;
	INT8U TxPara2;
	
	INT8U UpdateFlsg;
}STypedefDbg;	












void DevDbg_SetFlag(INT8U data);
INT8U DevDbg_InqFlag(void);
void DevDbg_Init(void);
void DevDbg_Handle(void);
void DevDbg_TxHandle(void);
void DevDbg_RxHandle(void);

void DevDbg_PrintString(INT8U* Data);
void DevDbg_PrintStringHDLC(INT8U* Data,INT8U queueEnable);
void DevDbg_TxPack(INT8U *Buff,INT8U Length,INT8U queueEnable);

INT8U  DevDbg_UpData(INT8U *Buff,INT8U *AckBuff,INT8U *AckLength);
void  DevDbg_ByteToInt(INT8U *Buff8,INT32U *Buff32,INT8U Cnt);
void  DevDbg_IntToByte(INT8U *Buff8,INT32U *Buff32,INT8U Cnt);
void  DevDbg_ByteToFLOAT(INT8U *Buff8,FLOAT32S *Buff32,INT8U Cnt);
INT8U  DevDbg_Manual(INT8U *Buff,INT8U *AckBuff,INT8U *AckLength);

FLOAT32S DevDbg_InqPara(INT8U paraId);
void DevDbg_TxPara(void);

INT8U  DevDbg_InqFlag(void);


#endif
