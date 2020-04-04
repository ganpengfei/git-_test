#include "include.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief Clock configuration structure. */
typedef struct _clock_config
{
    mcg_config_t mcgConfig;       /*!< MCG configuration.      */
    sim_clock_config_t simConfig; /*!< SIM configuration.      */
    osc_config_t oscConfig;       /*!< OSC configuration.      */
    uint32_t coreClock;           /*!< core clock frequency.   */
} clock_config_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* Configuration for enter RUN mode. Core clock = 120MHz. */
const clock_config_t g_defaultClockConfigRun = {
    .mcgConfig =
        {
            .mcgMode = kMCG_ModePEE,             /* Work in PEE mode. */
            .irclkEnableMode = kMCG_IrclkEnable, /* MCGIRCLK enable. */
            .ircs = kMCG_IrcSlow,                /* Select IRC32k. */
            .fcrdiv = 0U,                        /* FCRDIV is 0. */

            .frdiv = 4U,
            .drs = kMCG_DrsLow,         /* Low frequency range */
            .dmx32 = kMCG_Dmx32Default, /* DCO has a default range of 25% */
            .oscsel = kMCG_OscselOsc,   /* Select OSC */

            .pll0Config =
                {
                    .enableMode = 0U, .prdiv = 0x00U, .vdiv = 0x04U,
                },
            .pllcs = kMCG_PllClkSelPll0,
        },
    .simConfig =
        {
            .pllFllSel = 1U, /* PLLFLLSEL select PLL. */
            .pllFllDiv = 0U,
            .pllFllFrac = 0U,
            .er32kSrc = 2U,         /* ERCLK32K selection, use RTC. */
            .clkdiv1 = 0x01140000U, /* SIM_CLKDIV1. */
        },
    .oscConfig = {.freq = BOARD_XTAL0_CLK_HZ,
                  .capLoad = 0,
                  .workMode = kOSC_ModeOscLowPower,
                  .oscerConfig =
                      {
                          .enableMode = kOSC_ErClkEnable,
#if (defined(FSL_FEATURE_OSC_HAS_EXT_REF_CLOCK_DIVIDER) && FSL_FEATURE_OSC_HAS_EXT_REF_CLOCK_DIVIDER)
                          .erclkDiv = 0U,
#endif
                      }},
    .coreClock = 120000000U, /* Core clock frequency */
};
/***********************************************************************************
*Name      : CLOCK_SetSimSafeDivs
*Brief  	 : 系统时钟初始化
*Parameter : NA
*return    : NA
***********************************************************************************/
void DrvCfg_InitClock(void)
{
    CLOCK_SetSimSafeDivs();

    CLOCK_InitOsc0(&g_defaultClockConfigRun.oscConfig);
    CLOCK_SetXtal0Freq(BOARD_XTAL0_CLK_HZ);

    CLOCK_BootToPeeMode(g_defaultClockConfigRun.mcgConfig.oscsel, kMCG_PllClkSelPll0,
                        &g_defaultClockConfigRun.mcgConfig.pll0Config);

    CLOCK_SetInternalRefClkConfig(g_defaultClockConfigRun.mcgConfig.irclkEnableMode,
                                  g_defaultClockConfigRun.mcgConfig.ircs, g_defaultClockConfigRun.mcgConfig.fcrdiv);

    CLOCK_SetSimConfig(&g_defaultClockConfigRun.simConfig);

    SystemCoreClock = g_defaultClockConfigRun.coreClock;
}
/***********************************************************************************
*Name      : DrvCfg_Init
*Brief  	 : 芯片配置初始化
*Parameter : NA
*return    : NA
***********************************************************************************/
void DrvCfg_Init(void)
{
	DrvCfg_InitClock();
	
	CLOCK_EnableClock(kCLOCK_PortA);//不考虑省电，吧可能用到时钟都打开
	CLOCK_EnableClock(kCLOCK_PortB);
	CLOCK_EnableClock(kCLOCK_PortC);
	CLOCK_EnableClock(kCLOCK_PortD);
	CLOCK_EnableClock(kCLOCK_PortE);
	CLOCK_EnableClock(kCLOCK_Uart0);
	CLOCK_EnableClock(kCLOCK_Uart1);
	CLOCK_EnableClock(kCLOCK_Uart2);
#ifdef CPU_MK66FN2M0VMD18		
	CLOCK_EnableClock(kCLOCK_Uart3);
	CLOCK_EnableClock(kCLOCK_Uart4);
#endif	
	CLOCK_EnableClock(kCLOCK_Flexcan0);
	CLOCK_EnableClock(kCLOCK_Flexcan1);
}

/***********************************************************************************
*Name      : DrvCfg_GpioInit
*Brief  	 : IO初始化
*Parameter : EnumGpio Gpiox               : PortA  PortB PortC PortD PortE
							INT32U pin                  : 0-31
							gpio_pin_direction_t Direct : kGPIO_DigitalInput or kGPIO_DigitalOutput
							INT8U InitStatus            : 0 or 1 only for output
*return    : NA
***********************************************************************************/
void DrvCfg_GpioInit(GPIO_Type *base, INT32U pin,
											gpio_pin_direction_t Direct,INT8U InitStatus)
{
	gpio_pin_config_t config;

	config.pinDirection = Direct;
	config.outputLogic = InitStatus;


	switch((INT32U)base)
	{
		case (INT32U)GPIOA:PORT_SetPinMux(PORTA, pin, kPORT_MuxAsGpio);break;
		case (INT32U)GPIOB:PORT_SetPinMux(PORTB, pin, kPORT_MuxAsGpio);break;
		case (INT32U)GPIOC:PORT_SetPinMux(PORTC, pin, kPORT_MuxAsGpio);break;
		case (INT32U)GPIOD:PORT_SetPinMux(PORTD, pin, kPORT_MuxAsGpio);break;
		case (INT32U)GPIOE:PORT_SetPinMux(PORTE, pin, kPORT_MuxAsGpio);break;
		default:break;
	}
	GPIO_PinInit(base, pin, &config);		
}

/***********************************************************************************
*Name      : HardFault_Handler
*Brief  	 : 错误中断管理
*Parameter : NA
*return    : NA
***********************************************************************************/
void HardFault_Handler_c(SType_Stack * Stack)
{
	INT8U PrintData[100];
	memset(PrintData,0,sizeof(PrintData));
	sprintf((char*)PrintData,"R0:%08x,R1:%08x,R2:%08x,R3:%08x,R12:%08x,LR:%08x,PC:%08x,xPSR:%08x\n",
			Stack->R0,Stack->R1,Stack->R2,Stack->R3,Stack->R12,Stack->LR,Stack->PC,Stack->xPSR);

	while(1)
	{
		for(INT32U i=0;i<10000000;i++);//阻塞延时
		DrvDbg_Buff(PrintData,strlen((char*)PrintData));
	}
}	
__asm void HardFault_Handler(void)
{
	TST   LR, #4
	ITE   EQ
	MRSEQ R0, MSP
	MRSNE R0, PSP
	B     __cpp(HardFault_Handler_c)     
}	
/***********************************************************************************
*Name      : MemManage_Handler
*Brief  	 : 错误中断管理
*Parameter : NA
*return    : NA
***********************************************************************************/
void MemManage_Handler(void)
{

	while(1);
}
/***********************************************************************************
*Name      : BusFault_Handler
*Brief  	 : 错误中断管理
*Parameter : NA
*return    : NA
***********************************************************************************/
void BusFault_Handler(void)
{

	while(1);
}
/***********************************************************************************
*Name      : UsageFault_Handler
*Brief  	 : 错误中断管理
*Parameter : NA
*return    : NA
***********************************************************************************/
void UsageFault_Handler(void)
{

	while(1);
}





