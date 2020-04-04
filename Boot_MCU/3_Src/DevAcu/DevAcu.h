#ifndef _DEVACU_
#define _DEVACU_


void DevAcu_Init(void);
void DevAcu_Handle(void);

void DevAcu_LedInit(void);
void DevAcu_LedHandle(void);
void DevAcu_PrintVersion(INT8U *Name,INT8U *Data,INT8U *Time);
INT8U *DevAcu_InqVersion(void);
void DevAcu_DriverInit(void);
void DevAcu_SetDev(ETypeDev device);
ETypeDev DevAcu_InqDev(void);
void DevAcu_DriverStop(void);







#endif
