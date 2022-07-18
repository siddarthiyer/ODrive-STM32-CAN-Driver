W/*
 * CAN.c
 *
 *  Created on: May 26, 2022
 *      Author: Kunal
 */



#include "CAN.h"


/*
 * @func 			:	CAN_Init
 * @rev				:	1
 * @Comment			:	None
 * @input param		:	mailbox
 * @output param	:	none
 * @operation		:	Configures the CAN controller as per the CAN_Config struct.
 *                      Take a look at the examples and the CAN_Config structure for
 *                      more information
 */
int CAN_Init(CAN_Init_Typedef *init)
{
    RCC -> APB1ENR &= ~RCC_APB1ENR_CAN1EN;
    GPIO_Pin_Setup(GPIOA, 12, ALTERNATE_FUNCTION_OUTPUT_PUSHPULL, 9);
    GPIO_Pin_Setup(GPIOA, 11, ALTERNATE_FUNCTION_OUTPUT_PUSHPULL, 9);
    RCC -> APB1ENR |= RCC_APB1ENR_CAN1EN;


	init -> CAN_INSTANCE -> MCR	|= CAN_MCR_INRQ;
	init -> CAN_INSTANCE -> MCR |= CAN_MCR_NART;
	init -> CAN_INSTANCE -> IER |= init -> interrupt;
	init -> CAN_INSTANCE -> BTR = init -> baudrate;

	return 1;
}

int CAN_Filter_Init(CAN_Init_Typedef *init, CAN_Filter_TypeDef *filter)
{
	uint32_t can_id = 0;
	init -> CAN_INSTANCE -> FMR |= CAN_FMR_FINIT;

	if(filter->filter_id > 13)
	{
		return -1;
	}
	else
	{
// ID INFORMATION
		if(filter -> id_type == CAN_ID_Standard)
		{
			can_id = (filter->ID << 21) | 0;
		}
		else
		{
			can_id = (filter->ID << 3) | 4;
		}

		if(filter -> frame_type == CAN_Frame_Remote)
		{
			can_id |= 2;
		}

		init -> CAN_INSTANCE -> FA1R &= ~(1 << filter -> filter_id);

		init -> CAN_INSTANCE -> FS1R |=  (1 << filter -> filter_id);

		init -> CAN_INSTANCE -> sFilterRegister[filter->filter_id].FR1 = can_id;
		init -> CAN_INSTANCE -> sFilterRegister[filter->filter_id].FR2 = can_id;

		init -> CAN_INSTANCE -> FFA1R &= ~(1 << filter->filter_id);
		init -> CAN_INSTANCE -> FA1R  |= (1 << filter->filter_id);

		init -> CAN_INSTANCE -> FMR &= ~CAN_FMR_FINIT;

	}
	return 1;
}

void CAN_Start(CAN_Init_Typedef *init)
{
	init -> CAN_INSTANCE -> MCR &= ~CAN_MCR_SLEEP;
	init -> CAN_INSTANCE -> MCR &= ~CAN_MCR_INRQ;
	while((init -> CAN_INSTANCE -> MSR & CAN_MSR_SLAK)){}
	while((init -> CAN_INSTANCE ->MSR & CAN_MSR_INAK));
}

void CAN_Send_Packet(CAN_Init_Typedef *init, CAN_TX_Typedef *tx)
{
	init -> CAN_INSTANCE -> sTxMailBox[0].TDHR &= ~0xFFFFFFFF;
	init -> CAN_INSTANCE -> sTxMailBox[0].TDLR &= ~0xFFFFFFFF;
	init -> CAN_INSTANCE -> sTxMailBox[0].TDTR &= ~0xFFFFFFFF;
	init -> CAN_INSTANCE -> sTxMailBox[0].TIR  &= ~0xFFFFFFFF;
//---------------------------------------------------------------------------------------------------------------------
	switch (tx->id_type)
	{
		case CAN_ID_Standard:
		{
			init -> CAN_INSTANCE -> sTxMailBox[0].TIR  = tx->ID << 21;
			init -> CAN_INSTANCE -> sTxMailBox[0].TIR  &= ~1 << 2;
		}
			break;
		case CAN_ID_Extended:
		{
			init -> CAN_INSTANCE -> sTxMailBox[0].TIR  = tx->ID << 3;
			init -> CAN_INSTANCE -> sTxMailBox[0].TIR  |= 1 << 2;
		}
			break;
	}
//---------------------------------------------------------------------------------------------------------------------
	switch (tx->frame_type)
	{
		case CAN_Frame_Data:
		{
			init -> CAN_INSTANCE -> sTxMailBox[0].TIR  &= ~(1 << 1);
		}
			break;
		case CAN_Frame_Remote:
		{
			init -> CAN_INSTANCE -> sTxMailBox[0].TIR  |= (1 << 1);
		}
			break;
	}
//---------------------------------------------------------------------------------------------------------------------
	init -> CAN_INSTANCE -> sTxMailBox[0].TDTR = tx->data_length;
	init -> CAN_INSTANCE -> sTxMailBox[0].TDTR &= ~CAN_TDT0R_TGT;
	init -> CAN_INSTANCE -> sTxMailBox[0].TDHR = tx->data[7] << 24 | tx->data[6] << 16 | tx->data[5] << 8 | tx->data[4] << 0;
	init -> CAN_INSTANCE -> sTxMailBox[0].TDLR = tx->data[3] << 24 | tx->data[2] << 16 | tx->data[1] << 8 | tx->data[0] << 0;
	init -> CAN_INSTANCE -> sTxMailBox[0].TIR  |= (1 << 0);
	while(init -> CAN_INSTANCE -> sTxMailBox[0].TIR & (1 << 0)){}

}

void CAN_Get_Packet(CAN_Init_Typedef *init, CAN_RX_Typedef *rx)
{


	int frame_type = 0;
	int id_type = 0;



	id_type =  (CAN1 -> sFIFOMailBox[0].RIR & CAN_RI0R_IDE_Msk) >> CAN_RI0R_IDE_Pos ;
	frame_type = (CAN1 -> sFIFOMailBox[0].RIR & CAN_RI0R_RTR_Msk) >> CAN_RI0R_RTR_Pos ;

	if(id_type)
	{
		//Extended ID
		rx->id_type = CAN_ID_Extended;
		rx->ID = (init -> CAN_INSTANCE -> sFIFOMailBox[0].RIR & CAN_RI0R_EXID_Msk) >> CAN_RI0R_EXID_Pos;
	}
	else
	{
		//Standard ID
		rx->id_type = CAN_ID_Standard;
		rx->ID = (init -> CAN_INSTANCE -> sFIFOMailBox[0].RIR & CAN_RI0R_STID_Msk) >> CAN_RI0R_STID_Pos;
	}

	if(frame_type)
	{
		//RTR Frame
		rx->frame_type = CAN_Frame_Remote;
		rx->data_length = (init -> CAN_INSTANCE -> sFIFOMailBox[0].RDTR & CAN_RDT0R_DLC_Msk) >> CAN_RDT0R_DLC_Pos;
		init -> CAN_INSTANCE -> RF0R |= CAN_RF0R_RFOM0;
		while((init -> CAN_INSTANCE -> RF0R & CAN_RF0R_RFOM0)){}
	}
	else
	{
		//Data Frame
		rx->frame_type = CAN_Frame_Data;
		rx->data_length = (init -> CAN_INSTANCE -> sFIFOMailBox[0].RDTR & CAN_RDT0R_DLC_Msk) >> CAN_RDT0R_DLC_Pos;
		for(int i = 0; i < rx->data_length; i++)
		{
			if(i < 4)
			{
				rx->data[i] =  (init -> CAN_INSTANCE -> sFIFOMailBox[0].RDLR & ( 0xFF << (8*i))) >> (8*i);
			}
			else
			{
				rx->data[i] =  (init -> CAN_INSTANCE -> sFIFOMailBox[0].RDHR & ( 0xFF << (8*(i-4)))) >> (8*(i-4));
			}
		}

		init -> CAN_INSTANCE -> RF0R |= CAN_RF0R_RFOM0;	}


}
