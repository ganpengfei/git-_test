#ifndef _CTLFPGA_
#define _CTLFPGA_




typedef union
{
	
	struct{
	INT32U Pin0 : 1,	
					Pin1 : 1,
				Pin2 : 1,
				Pin3 : 1,
				Pin4 : 1,
				Pin5 : 1,
				Pin6 : 1,
				Pin7 : 1,
				Pin8 : 1,
				Pin9 : 1,
				Pin10 : 1,
				Pin11 : 1,
				Pin12 : 1,
				Pin13 : 1,
				Pin14 : 1,
				Pin15 : 1,
				Pin16 : 1,
				Pin17 : 1,
				Pin18 : 1,
				Pin19 : 1,
				Pin20 : 1,
				Pin21 : 1,
				Pin22 : 1,
				Pin23 : 1,
				Pin24 : 1,
				Pin25 : 1,
				Pin26 : 1,
				Pin27 : 1,
				Pin28 : 1,
				Pin29 : 1,
				Pin30 : 1,
				Pin31 : 1;
			}Pin;	
	INT32U Pins;			
}UTypedefPins;	

typedef struct
{
	volatile  UTypedefPins PDOR;  //output reg                            
  volatile  UTypedefPins PSOR;                             
  volatile  UTypedefPins PCOR;                              
  volatile  UTypedefPins PTOR;                            
  volatile  UTypedefPins PDIR;  //input reg                            
  volatile  UTypedefPins PDDR;                            
}STypedefPins;

#define uGPIOA  ((STypedefPins*)(0x400FF000))
#define uGPIOB  ((STypedefPins*)(0x400FF040))
#define uGPIOC  ((STypedefPins*)(0x400FF080))
#define uGPIOD  ((STypedefPins*)(0x400FF0C0))
#define uGPIOE  ((STypedefPins*)(0x400FF100))


#define nCONFIG  		uGPIOB->PDOR.Pin.Pin0
#define DCLK  			uGPIOD->PDOR.Pin.Pin12
#define DATA  			uGPIOD->PDOR.Pin.Pin13
#define CONFIGDONE  uGPIOA->PDIR.Pin.Pin28
#define nSTATUS  		uGPIOA->PDIR.Pin.Pin29
#define FPGA_MSEL  	1          // 低电平表示JTAG模式

#define LEDTEST     uGPIOE->PDOR.Pin.Pin27 
#define LEDTEST2    nCONFIG

typedef struct
{
	INT32U CodeFlag;
	INT32U CodeSize;
	INT32U CodeSum;
	INT32U FlagSum;
	
	INT32U ReadAdd;

}STypeDef_FpgaPara;


void CtlFpga_Init(void);
void CtlFpga_Handle(void);
INT8U CtlFpga_Updata(void);
void CtlFpga_Delayus(INT32U cnt);







#endif
