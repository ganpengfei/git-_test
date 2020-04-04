#ifndef _DRVCAN_
#define _DRVCAN_

#define DrvCan0_IRQHandler CAN0_ORed_Message_buffer_IRQHandler
#define DrvCan1_IRQHandler CAN1_ORed_Message_buffer_IRQHandler


#define TYPE_FORMAT  flexcan_frame_format_t
#define CAN_STD      kFLEXCAN_FrameFormatStandard
#define CAN_EXT      kFLEXCAN_FrameFormatExtend

#define TYPE_FRAME   flexcan_frame_type_t
#define CAN_DATA     kFLEXCAN_FrameTypeData
#define CAN_REMOT    kFLEXCAN_FrameTypeRemote


/*≈‰÷√œÓ*/
#define CAN0_PORT               PORTA  
#define CAN0_PORT_TX_PIN        12U 
#define CAN0_PORT_RX_PIN        13U 
#define CAN0_PORT_MUX_SOURCE    kPORT_MuxAlt2  
#define CAN0_RX_MESSAGE_BUFFER_NUM  (8)
#define CAN0_TX_MESSAGE_BUFFER_NUM  (9)


#define CAN1_PORT               PORTC 
#define CAN1_PORT_TX_PIN        17U 
#define CAN1_PORT_RX_PIN        16U 
#define CAN1_PORT_MUX_SOURCE    kPORT_MuxAlt2  
#define CAN1_RX_MESSAGE_BUFFER_NUM  (6)
#define CAN1_TX_MESSAGE_BUFFER_NUM  (7)

#define CAN_MASK_BIT               (INT16U)0x00

typedef struct
{
	TYPE_FORMAT Format;
	TYPE_FRAME Frame;
	INT8U *DataAdd;
	INT16U Add;
	INT16U TxLength;	
	INT8U Status;
}STypeCanPara;



void DrvCan_Init(CAN_Type *canx,INT32U bps,INT8U id);

void DrvCan_TxBuff(CAN_Type *canx,TYPE_FORMAT Format,TYPE_FRAME Frame,
	  INT16U Add,INT8U *Buff,INT8U Length);
INT8U DrvCan_InqTxStatus(CAN_Type *canx);
void DrvCan_Stop(CAN_Type *canx);









#endif
