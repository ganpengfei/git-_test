#ifndef _DRVCFG_
#define _DRVCFG_






#define BOARD_XTAL32K_CLK_HZ  32768U

typedef struct
{
	INT32U R0;
	INT32U R1;
	INT32U R2;
	INT32U R3;
	INT32U R12;
	INT32U LR;
	INT32U PC;
	INT32U xPSR;
}SType_Stack;

void DrvCfg_Init(void);
void DrvCfg_GpioInit(GPIO_Type *base, INT32U pin,
											gpio_pin_direction_t Direct,INT8U InitStatus);
void DrvCfg_InitClock(void);

#endif

