#ifndef _CTLBOOT_
#define _CTLBOOT_


typedef struct
{
	INT8U RunStatus;
	INT32U CodeFlag;
	INT32U CodeSize;
	INT32U CodeSum;
	INT32U FlagSum;

}STypeDef_BootPara;




void CtlBoot_Init(void);
void CtlBoot_Handle(void);
void CtlBoot_JumpToUserApplication(INT32U userSP,
                       INT32U userStartup);





#endif
