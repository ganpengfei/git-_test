#ifndef _DRVUPDATE_
#define _DRVUPDATE_




#define DEVICE_MCU            ((INT8U)1)
#define DEVICE_FPGA           ((INT8U)2)

typedef struct
{
	INT32U NowAdd;
	INT32U CodeSize;
	INT32U CodeSum;
	INT32U FlagSum; 
	INT8U  Device;
}STypeFlash;


void DrvUpData_Init(void);
void DrvUpData_SetCodeInf(INT32U Size,INT32U CheckSum,INT8U device);
INT8U DrvUpData_Begin(void);
INT8U DrvUpData_WritePack(INT32U* Buff,INT8U Length);
INT8U DrvUpData_WriteBuff(INT8U* Buff,INT8U Length);
INT32U DrvUpData_CheckSum(INT32U Add,INT32U Size);
INT8U DrvUpData_SetFlag(INT32U Flag);



void DrvUpData_Check(INT32U *data,INT32U *data1);


#endif
