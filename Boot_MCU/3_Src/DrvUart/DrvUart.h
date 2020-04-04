#ifndef _DRVUART_
#define _DRVUART_

/*接受中断*/
#define DrvUart_Com0RxCharIrq UART0_RX_TX_IRQHandler
#define DrvUart_Com1RxCharIrq UART1_RX_TX_IRQHandler
#define DrvUart_Com2RxCharIrq UART2_RX_TX_IRQHandler
#define DrvUart_Com3RxCharIrq UART3_RX_TX_IRQHandler
#define DrvUart_Com4RxCharIrq UART4_RX_TX_IRQHandler


/*DMA配置*/
#define UART_DMA                    DMAMUX0
#define UART0_DMA_CHANNAL         ((INT8U)0)
#define UART1_DMA_CHANNAL         ((INT8U)1)
#define UART2_DMA_CHANNAL         ((INT8U)2)
#define UART3_DMA_CHANNAL         ((INT8U)3)
#define UART4_DMA_CHANNAL         ((INT8U)4)

#define UART0_TX_DMA_SOURCE         (kDmaRequestMux0UART0Tx)
#define UART1_TX_DMA_SOURCE         (kDmaRequestMux0UART1Tx)
#define UART2_TX_DMA_SOURCE         (kDmaRequestMux0UART2Tx)
#define UART3_TX_DMA_SOURCE         (kDmaRequestMux0UART3Tx)
#define UART4_TX_DMA_SOURCE         (kDmaRequestMux0UART4)

/*奇偶校验配置*/
#define UART_PARITY_NONE    kUART_ParityDisabled  
#define UART_PARITY_EVEN    kUART_ParityEven
#define UART_PARITY_ODD    	kUART_ParityOdd


#ifdef CPU_MK66FN2M0VMD18
/*端口复用*/
#define UART0_PORT 		 				 PORTA 
#define UART0_PORT_TX_PIN 		 ((INT8U)14) 
#define UART0_PORT_RX_PIN 		 ((INT8U)15) 
#define UART0_PORT_MUX_FUN      kPORT_MuxAlt3

#define UART1_PORT 		 				 PORTE 
#define UART1_PORT_TX_PIN 		 ((INT8U)0) 
#define UART1_PORT_RX_PIN 		 ((INT8U)1) 
#define UART1_PORT_MUX_FUN      kPORT_MuxAlt3

#define UART2_PORT 		 				 PORTD 
#define UART2_PORT_TX_PIN 		 ((INT8U)3) 
#define UART2_PORT_RX_PIN 		 ((INT8U)2) 
#define UART2_PORT_MUX_FUN      kPORT_MuxAlt3

#endif
#ifdef CPU_MKS22FN256VLL12
/*端口复用*/
#define UART0_PORT 		 				 PORTB 
#define UART0_PORT_TX_PIN 		 ((INT8U)16) 
#define UART0_PORT_RX_PIN 		 ((INT8U)17) 
#define UART0_PORT_MUX_FUN      kPORT_MuxAlt3

#define UART1_PORT 		 				 PORTE 
#define UART1_PORT_TX_PIN 		 ((INT8U)0) 
#define UART1_PORT_RX_PIN 		 ((INT8U)1) 
#define UART1_PORT_MUX_FUN      kPORT_MuxAlt3

#define UART2_PORT 		 				 PORTD 
#define UART2_PORT_TX_PIN 		 ((INT8U)3) 
#define UART2_PORT_RX_PIN 		 ((INT8U)2) 
#define UART2_PORT_MUX_FUN      kPORT_MuxAlt3


#endif


#define UART3_PORT 		 				 PORTE 
#define UART3_PORT_TX_PIN 		 ((INT8U)4) 
#define UART3_PORT_RX_PIN 		 ((INT8U)5) 
#define UART3_PORT_MUX_FUN      kPORT_MuxAlt3

#define UART4_PORT 		 				 PORTE 
#define UART4_PORT_TX_PIN 		 ((INT8U)24) 
#define UART4_PORT_RX_PIN 		 ((INT8U)25) 
#define UART4_PORT_MUX_FUN      kPORT_MuxAlt3






void DrvUart_Init(UART_Type *Uartx,INT32U Baud,uart_parity_mode_t Parity);
void DrvUart_TxBuff(UART_Type *Uartx,INT8U* Buff,INT8U Lenth);
INT8U DrvUart_InqUartStatus(UART_Type *Uartx);
void DrvUart_Stop(UART_Type *Uartx);








#endif
