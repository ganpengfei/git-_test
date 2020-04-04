#ifndef _INCLUDE_
#define _INCLUDE_


typedef unsigned char INT8U;
typedef unsigned short INT16U;	
typedef unsigned int INT32U;

typedef signed char INT8S;
typedef signed short INT16S;	
typedef signed int INT32S;

typedef float FLOAT32S;

#ifndef ENABLE
#define ENABLE ((INT8U)1)
#endif

#ifndef DISABLE
#define DISABLE ((INT8U)0)
#endif

#ifndef TRUE
#define TRUE ((INT8U)1)
#endif

#ifndef FALSE
#define FALSE ((INT8U)0)
#endif

#ifndef BUSY
#define BUSY ((INT8U)1)
#endif

#ifndef IDLE
#define IDLE ((INT8U)0)
#endif

#ifndef SUCC
#define SUCC ((INT8U)1)
#endif

#ifndef FAIL
#define FAIL ((INT8U)0)
#endif
#ifndef HIGH
#define HIGH ((INT8U)1)
#endif

#ifndef LOW
#define LOW ((INT8U)0)
#endif



typedef enum
{
	DEV_UART0 = 0,
	DEV_UART1,
	DEV_UART2,
	DEV_UART3,
	DEV_UART4,
	DEV_CAN0,
	DEV_CAN1
}ETypeDev;



#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>


#include "fsl_clock.h"
#include "fsl_common.h"
#include "fsl_mcg.h"
#include "fsl_osc.h"
#include "fsl_smc.h"
#include "fsl_uart.h"
#include "fsl_pit.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "fsl_flexcan.h"
#include "fsl_dmamux.h"
#include "fsl_edma.h"
#include "fsl_uart_edma.h"
#include "fsl_dma_manager.h"
#include "fsl_adc16.h"
#include "fsl_flash.h"

#include "..\3_Src\DrvCan\DrvCan.h"
#include "..\3_Src\DrvUart\DrvUart.h"
#include "..\3_Src\DrvTimer\DrvTimer.h"
#include "..\2_Proj_K22\OwnCode\DrvCfg\DrvCfg.h"
#include "..\3_Src\DrvDbg\DrvDbg.h"
#include "..\3_Src\DrvUpDate\DrvUpDate.h"
#include "..\3_Src\DrvFlash\DrvFlash.h"

#include "..\3_Src\DevAcu\DevAcu.h"
#include "..\3_Src\DrvDbg\DevDbg.h"
#include "..\3_Src\CtlBoot\CtlBoot.h"
#include "..\3_Src\CtlFpga\CtlFpga.h"


#include "..\2_Proj_K22\OwnCode\config.h"
 


#endif
