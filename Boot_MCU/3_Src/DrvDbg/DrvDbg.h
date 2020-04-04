#ifndef _DRVDBG_
#define _DRVDBG_



#define MAX_BUFF_LENGTH         ((INT8U)255)
#define DRVDBG_ADD              ((INT8U)0x01)

typedef struct
{
	INT8U RxBuff[MAX_BUFF_LENGTH];
	INT8U Length;
	INT8U FindHead;
	INT8U Sum;
	INT8U LastDecode;
	
	INT8U TempBuff[MAX_BUFF_LENGTH];
	INT8U TempFind;
	INT8U TempLength;
	INT32U TempPack; 
}sType_DbgPara;


#define DBG_TX_QUEUE_LENGTH   ((INT8U)10)
typedef struct DbgLine
{
	struct DbgLine *Prev;
	struct DbgLine *Next;
	INT8U  EmptyFlag;
	INT8U  TxCnt;
	INT8U  *Buff;
	INT32U SendTime;
}STypeDbgLines;


STypeDbgLines *DrvDbg_CreatLine(INT8U element);
INT8U DrvDbg_TxBuffQueue(INT8U *txBuff,INT8U cnt);
INT8U DrvDbg_InqTxQueueStatus(void);
INT8U DrvDbg_InqTxQueueCnt(void);
void DrvDbg_Init(void);
void DrvDbg_Handle(void);
void DrvDbg_RxChar(INT8U Data,ETypeDev device);
INT8U DrvDbg_InqPackStatus(INT8U *Length);
void DrvDbg_InqPackData(INT8U *RxBuff,INT8U Length);
void DrvDbg_TxPack(INT8U *Buff,INT8U Length,INT8U queueEnable);
void DrvDbg_Buff(INT8U *Buff,INT8U Length);
void DrvDbg_FreeLine(void);
INT8U DrvDbg_InqDevStatus(void);





#endif
