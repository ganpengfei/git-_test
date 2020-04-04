#include "include.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief Clock configuration structure. */
typedef struct _clock_config
{
    mcg_config_t mcgConfig;       /*!< MCG configuration.    */
    sim_clock_config_t simConfig; /*!< SIM configuration.    */
    osc_config_t oscConfig;       /*!< OSC configuration.    */
    uint32_t coreClock;           /*!< core clock frequency. */
} clock_config_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* System clock frequency. */
extern uint32_t SystemCoreClock;

/* Configuration for enter VLPR mode. Core clock = 4MHz. */
const clock_config_t g_defaultClockConfigVlpr = {
    .mcgConfig =
        {
            .mcgMode = kMCG_ModeBLPI, /* Work in BLPI mode */
            .irclkEnableMode = kMCG_IrclkEnable,  /* MCGIRCLK enable */
            .ircs = kMCG_IrcFast,     /* Select IRC4M */
            .fcrdiv = 0U,             /* FCRDIV is 0 */

            .frdiv = 0U,
            .drs = kMCG_DrsLow,         /* Low frequency range */
            .dmx32 = kMCG_Dmx32Default, /* DCO has a default range of 25% */
            .oscsel = kMCG_OscselOsc,   /* Select OSC */

            .pll0Config =
                {
                    .enableMode = 0U, /* Don't enable PLL */
                    .prdiv = 0U,
                    .vdiv = 0U,
                },
        },
    .simConfig =
        {
            .pllFllSel = 3U,        /* PLLFLLSEL select IRC48MCLK */
            .er32kSrc = 2U,         /* ERCLK32K selection, use RTC */
            .clkdiv1 = 0x00040000U, /* SIM_CLKDIV1 */
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
    .coreClock = 4000000U, /* Core clock frequency */
};

/* Configuration for enter RUN mode. Core clock = 72MHz. */
const clock_config_t g_defaultClockConfigRun = {
    .mcgConfig =
        {
            .mcgMode = kMCG_ModePEE,             /* Work in PEE mode */
            .irclkEnableMode = kMCG_IrclkEnable, /* MCGIRCLK enable */
            .ircs = kMCG_IrcSlow,                /* Select IRC32k */
            .fcrdiv = 0U,                        /* FCRDIV is 0 */

            .frdiv = 3U,
            .drs = kMCG_DrsLow,         /* Low frequency range */
            .dmx32 = kMCG_Dmx32Default, /* DCO has a default range of 25% */
            .oscsel = kMCG_OscselOsc,   /* Select OSC */

#if (BOARD_XTAL0_CLK_HZ == 8000000)
			.pll0Config =
                {
                    .enableMode = 0U, .prdiv = 0x2U, .vdiv = 0x6U,
                },
#endif
#if (BOARD_XTAL0_CLK_HZ == 12000000)	
            .pll0Config =
                {
                    .enableMode = 0U, .prdiv = 0x5U, .vdiv = 0x10U,
                },
#endif				
        },
    .simConfig =
        {
            .pllFllSel = 1U,        /* PLLFLLSEL select PLL */
            .er32kSrc = 2U,         /* ERCLK32K selection, use RTC */
            .clkdiv1 = 0x01020000U, /* SIM_CLKDIV1 */
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
    .coreClock = 80000000U, /* Core clock frequency */
};

/* Configuration for enter RUN mode. Core clock = 120MHz. */
const clock_config_t g_defaultClockConfigHsrun = {
    .mcgConfig =
        {
            .mcgMode = kMCG_ModePEE,             /* Work in PEE mode */
            .irclkEnableMode = kMCG_IrclkEnable, /* MCGIRCLK enable */
            .ircs = kMCG_IrcSlow,                /* Select IRC32k */
            .fcrdiv = 0U,                        /* FCRDIV is 0 */

            .frdiv = 3U,
            .drs = kMCG_DrsLow,         /* Low frequency range */
            .dmx32 = kMCG_Dmx32Default, /* DCO has a default range of 25% */
            .oscsel = kMCG_OscselOsc,   /* Select OSC */

            .pll0Config =
                {
                    .enableMode = 0U, .prdiv = 0x1U, .vdiv = 0x6U,
                },
        },
    .simConfig =
        {
            .pllFllSel = 1U,        /* PLLFLLSEL select PLL */
            .er32kSrc = 2U,         /* ERCLK32K selection, use RTC */
            .clkdiv1 = 0x12040000U, /* SIM_CLKDIV1 */
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

/*******************************************************************************
 * Code
 ******************************************************************************/
/*
 * How to setup clock using clock driver functions:
 *
 * 1. CLOCK_SetSimSafeDivs, to make sure core clock, bus clock, flexbus clock
 *    and flash clock are in allowed range during clock mode switch.
 *
 * 2. Call CLOCK_Osc0Init to setup OSC clock, if it is used in target mode.
 *
 * 3. Set MCG configuration, MCG includes three parts: FLL clock, PLL clock and
 *    internal reference clock(MCGIRCLK). Follow the steps to setup:
 *
 *    1). Call CLOCK_BootToXxxMode to set MCG to target mode.
 *
 *    2). If target mode is FBI/BLPI/PBI mode, the MCGIRCLK has been configured
 *        correctly. For other modes, need to call CLOCK_SetInternalRefClkConfig
 *        explicitly to setup MCGIRCLK.
 *
 *    3). Don't need to configure FLL explicitly, because if target mode is FLL
 *        mode, then FLL has been configured by the function CLOCK_BootToXxxMode,
 *        if the target mode is not FLL mode, the FLL is disabled.
 *
 *    4). If target mode is PEE/PBE/PEI/PBI mode, then the related PLL has been
 *        setup by CLOCK_BootToXxxMode. In FBE/FBI/FEE/FBE mode, the PLL could
 *        be enabled independently, call CLOCK_EnablePll0 explicitly in this case.
 *
 * 4. Call CLOCK_SetSimConfig to set the clock configuration in SIM.
 */

void BOARD_BootClockVLPR(void)
{
    CLOCK_SetSimSafeDivs();

    CLOCK_BootToBlpiMode(g_defaultClockConfigVlpr.mcgConfig.fcrdiv, g_defaultClockConfigVlpr.mcgConfig.ircs,
                         g_defaultClockConfigVlpr.mcgConfig.irclkEnableMode);

    CLOCK_SetSimConfig(&g_defaultClockConfigVlpr.simConfig);

    SystemCoreClock = g_defaultClockConfigVlpr.coreClock;

    SMC_SetPowerModeProtection(SMC, kSMC_AllowPowerModeAll);
    SMC_SetPowerModeVlpr(SMC);
    while (SMC_GetPowerModeState(SMC) != kSMC_PowerStateVlpr)
    {
    }
}

void BOARD_BootClockRUN(void)
{
    CLOCK_SetSimSafeDivs();

    CLOCK_InitOsc0(&g_defaultClockConfigRun.oscConfig);

    CLOCK_BootToPeeMode(g_defaultClockConfigRun.mcgConfig.oscsel, kMCG_PllClkSelPll0,
                        &g_defaultClockConfigRun.mcgConfig.pll0Config);

    CLOCK_SetInternalRefClkConfig(g_defaultClockConfigRun.mcgConfig.irclkEnableMode,
                               g_defaultClockConfigRun.mcgConfig.ircs, g_defaultClockConfigRun.mcgConfig.fcrdiv);

    CLOCK_SetSimConfig(&g_defaultClockConfigRun.simConfig);

    SystemCoreClock = g_defaultClockConfigRun.coreClock;
}

void BOARD_BootClockHSRUN(void)
{
    SMC_SetPowerModeProtection(SMC, kSMC_AllowPowerModeAll);
    SMC_SetPowerModeHsrun(SMC);
    while (SMC_GetPowerModeState(SMC) != kSMC_PowerStateHsrun)
    {
    }

    CLOCK_SetSimSafeDivs();

    CLOCK_InitOsc0(&g_defaultClockConfigHsrun.oscConfig);

    CLOCK_BootToPeeMode(g_defaultClockConfigHsrun.mcgConfig.oscsel, kMCG_PllClkSelPll0,
                        &g_defaultClockConfigHsrun.mcgConfig.pll0Config);

    CLOCK_SetInternalRefClkConfig(g_defaultClockConfigHsrun.mcgConfig.irclkEnableMode,
                               g_defaultClockConfigHsrun.mcgConfig.ircs, g_defaultClockConfigHsrun.mcgConfig.fcrdiv);

    CLOCK_SetSimConfig(&g_defaultClockConfigHsrun.simConfig);

    SystemCoreClock = g_defaultClockConfigHsrun.coreClock;
}


/*
*Name      : DrvCfg_Init
*Brief  	 : 芯片配置初始化
*Parameter : NA
*return    : NA
*/
uint32_t g_xtal0Freq = BOARD_XTAL0_CLK_HZ;
uint32_t g_xtal32Freq = BOARD_XTAL32K_CLK_HZ;
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
	CLOCK_EnableClock(kCLOCK_Adc0);
//	CLOCK_EnableClock(kCLOCK_Uart4);
//	CLOCK_EnableClock(kCLOCK_Flexcan0);
}

/*
*Name      : DrvCfg_GpioInit
*Brief  	 : IO初始化
*Parameter : EnumGpio Gpiox               : GPIOA  GPIOB GPIOC GPIOD GPIOE
							INT32U pin                  : 0-31
							gpio_pin_direction_t Direct : kGPIO_DigitalInput or kGPIO_DigitalOutput
							INT8U InitStatus            : 0 or 1 only for output
*return    : NA
*/
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
/*
*Name      : CLOCK_SetSimSafeDivs
*Brief  	 : 系统时钟初始化
*Parameter : NA
*return    : NA
*/
void DrvCfg_InitClock(void)
{
    BOARD_BootClockRUN();
	//BOARD_BootClockHSRUN();
}

/*
*Name      : HardFault_Handler
*Brief  	 : 错误中断管理
*Parameter : NA
*return    : NA
*/
void HardFault_Handler(void)
{
	
	while(1);
}	
/*
*Name      : MemManage_Handler
*Brief  	 : 错误中断管理
*Parameter : NA
*return    : NA
*/
void MemManage_Handler(void)
{

	while(1);
}
/*
*Name      : BusFault_Handler
*Brief  	 : 错误中断管理
*Parameter : NA
*return    : NA
*/
void BusFault_Handler(void)
{

	while(1);
}
/*
*Name      : UsageFault_Handler
*Brief  	 : 错误中断管理
*Parameter : NA
*return    : NA
*/
void UsageFault_Handler(void)
{

	while(1);
}
/*
*Name      : NMI_Handler
*Brief  	 : 错误中断管理
*Parameter : NA
*return    : NA
*/
void NMI_Handler(void)
{

	while(1);
}




