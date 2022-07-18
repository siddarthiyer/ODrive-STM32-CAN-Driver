/*
 * ODrive.c
 *
 *  Created on: 07-Jul-2022
 *      Author: sidiyer27
 */


#include "ODrive.h"

CAN_TX_Typedef TX;
CAN_RX_Typedef RX;
CAN_Filter_TypeDef filter;
CAN_Init_Typedef CAN;


void CAN_Setup(CAN_TypeDef *CAN_INSTANCE, int32_t baudrate){
	  CAN.CAN_INSTANCE = CAN_INSTANCE;
	  CAN.baudrate = baudrate;
	  CAN.interrupt = Fifo0_Message_Pending;
	  CAN_Init(&CAN);
	  filter.ID = 0x0;
	  filter.filter_id = 0;
	  filter.id_type = CAN_ID_Standard;
	  filter.frame_type = CAN_Frame_Data;
	  CAN_Filter_Init(&CAN, &filter);
	  CAN_Start(&CAN);
	  __disable_irq();
	  HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
	  HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
	  __enable_irq();
}

void Set_TX_Param(int AXIS_ID, int COMMAND_ID, int id_type, int frame_type, int data_length){
	TX.ID = (AXIS_ID << 5) | COMMAND_ID;
	TX.id_type = id_type;
	TX.frame_type = frame_type;
	TX.data_length = data_length;
}

void Set_Axis_Requested_State(Axis Axis, Axis_State state){
	Set_TX_Param(Axis.AXIS_ID, SET_AXIS_REQUESTED_STATE, CAN_ID_Standard, CAN_Frame_Data, 4);
	unsigned int Requested_State = state;
	uint8_t *ptrToFloat;
	ptrToFloat = (uint8_t *)&Requested_State;
	TX.data[0] = ptrToFloat[0];
	TX.data[1] = ptrToFloat[1];
	TX.data[2] = ptrToFloat[2];
	TX.data[3] = ptrToFloat[3];
	CAN_Send_Packet(&CAN, &TX);
}

void Set_Input_Vel(Axis Axis, float vel, float torqueff){
	Set_TX_Param(Axis.AXIS_ID, SET_INPUT_VEL, CAN_ID_Standard, CAN_Frame_Data, 8);
	uint8_t *ptrVel;
	ptrVel = (uint8_t *)&vel;
	uint8_t *ptrTor;
	ptrTor = (uint8_t *)&torqueff;
	TX.data[0] = ptrVel[0];
	TX.data[1] = ptrVel[1];
	TX.data[2] = ptrVel[2];
	TX.data[3] = ptrVel[3];
	TX.data[4] = ptrTor[0];
	TX.data[5] = ptrTor[1];
	TX.data[6] = ptrTor[2];
	TX.data[7] = ptrTor[3];
	CAN_Send_Packet(&CAN, &TX);
}

void Clear_Errors(Axis Axis){
	Set_TX_Param(Axis.AXIS_ID, CLEAR_ERRORS, CAN_ID_Standard, CAN_Frame_Data, 0);
	CAN_Send_Packet(&CAN, &TX);
}

void Reboot_ODrive(Axis Axis){
	Set_TX_Param(Axis.AXIS_ID, REBOOT_ODRIVE, CAN_ID_Standard, CAN_Frame_Data, 0);
	CAN_Send_Packet(&CAN, &TX);
}

void Set_Controller_Modes(Axis Axis, Control_Mode ControlMode, Input_Mode InputMode){
	Set_TX_Param(Axis.AXIS_ID, SET_CONTROLLER_MODES, CAN_ID_Standard, CAN_Frame_Data, 8);
	int Control = ControlMode;
	int Input = InputMode;
	uint8_t *ptrControl;
	ptrControl = (uint8_t *)&Control;
	uint8_t *ptrInput;
	ptrInput = (uint8_t *)&Input;
	TX.data[0] = ptrControl[0];
	TX.data[1] = ptrControl[1];
	TX.data[2] = ptrControl[2];
	TX.data[3] = ptrControl[3];
	TX.data[4] = ptrInput[0];
	TX.data[5] = ptrInput[1];
	TX.data[6] = ptrInput[2];
	TX.data[7] = ptrInput[3];
	CAN_Send_Packet(&CAN, &TX);
}

void Set_Input_Pos(Axis Axis, float Input_Pos, int Vel_FF, int Torque_FF){
	Set_TX_Param(Axis.AXIS_ID, SET_INPUT_POS, CAN_ID_Standard, CAN_Frame_Data, 8);
	uint8_t *ptrPos;
	ptrPos = (uint8_t *)&Input_Pos;
	uint8_t *ptrVel;
	ptrVel = (uint8_t *)&Vel_FF;
	uint8_t *ptrTor;
	ptrTor = (uint8_t *)&Torque_FF;
	TX.data[0] = ptrPos[0];
	TX.data[1] = ptrPos[1];
	TX.data[2] = ptrPos[2];
	TX.data[3] = ptrPos[3];
	TX.data[4] = ptrVel[0];
	TX.data[5] = ptrVel[1];
	TX.data[6] = ptrTor[0];
	TX.data[7] = ptrTor[1];
	CAN_Send_Packet(&CAN, &TX);
}

void Get_Encoder_Count(Axis Axis){
	Set_TX_Param(Axis.AXIS_ID, GET_ENCODER_COUNT, CAN_ID_Standard, CAN_Frame_Remote, 0);
	CAN_Send_Packet(&CAN, &TX);
}

void Set_Input_Torque(Axis Axis, float torque){
	Set_TX_Param(Axis.AXIS_ID, SET_INPUT_TORQUE, CAN_ID_Standard, CAN_Frame_Data, 4);
	uint8_t *ptrTor;
	ptrTor = (uint8_t *)&torque;
	TX.data[0] = ptrTor[0];
	TX.data[1] = ptrTor[1];
	TX.data[2] = ptrTor[2];
	TX.data[3] = ptrTor[3];
	CAN_Send_Packet(&CAN, &TX);
}

void Get_Bus_Voltage_Current(Axis Axis){
	Set_TX_Param(Axis.AXIS_ID, GET_BUS_VOLTAGE_CURRENT, CAN_ID_Standard, CAN_Frame_Remote, 0);
	CAN_Send_Packet(&CAN, &TX);
}

void Get_IQ(Axis Axis){
	Set_TX_Param(Axis.AXIS_ID, GET_IQ, CAN_ID_Standard, CAN_Frame_Remote, 0);
	CAN_Send_Packet(&CAN, &TX);
}

void Set_Position_Gain(Axis Axis, float pos_gain){
	Set_TX_Param(Axis.AXIS_ID, SET_POSITION_GAIN, CAN_ID_Standard, CAN_Frame_Data, 4);
	uint8_t *ptrPos;
	ptrPos = (uint8_t *)&pos_gain;
	TX.data[0] = ptrPos[0];
	TX.data[1] = ptrPos[1];
	TX.data[2] = ptrPos[2];
	TX.data[3] = ptrPos[3];
	CAN_Send_Packet(&CAN, &TX);
}

void Set_Vel_Gains(Axis Axis, float Vel_Gain, float Vel_Int_Gain){
	Set_TX_Param(Axis.AXIS_ID, SET_VEL_GAINS, CAN_ID_Standard, CAN_Frame_Data, 8);
	uint8_t *ptrVelGain;
	ptrVelGain = (uint8_t *)&Vel_Gain;
	uint8_t *ptrVelIntGain;
	ptrVelIntGain = (uint8_t *)&Vel_Int_Gain;
	TX.data[0] = ptrVelGain[0];
	TX.data[1] = ptrVelGain[1];
	TX.data[2] = ptrVelGain[2];
	TX.data[3] = ptrVelGain[3];
	TX.data[4] = ptrVelIntGain[0];
	TX.data[5] = ptrVelIntGain[1];
	TX.data[6] = ptrVelIntGain[2];
	TX.data[7] = ptrVelIntGain[3];
	CAN_Send_Packet(&CAN, &TX);
}

void Set_Axis_Node_ID(Axis Axis, uint32_t node_id){
	Set_TX_Param(Axis.AXIS_ID, SET_AXIS_NODE_ID, CAN_ID_Standard, CAN_Frame_Data, 4);
	uint8_t *ptrNodeId;
	ptrNodeId = (uint8_t *)&node_id;
	TX.data[0] = ptrNodeId[0];
	TX.data[1] = ptrNodeId[1];
	TX.data[2] = ptrNodeId[2];
	TX.data[3] = ptrNodeId[3];
	CAN_Send_Packet(&CAN, &TX);
}

void Set_Limits(Axis Axis, float vel_lim, float curr_lim){
	Set_TX_Param(Axis.AXIS_ID, SET_LIMITS, CAN_ID_Standard, CAN_Frame_Data, 8);
	uint8_t *ptrVelLim;
	ptrVelLim = (uint8_t *)&vel_lim;
	uint8_t *ptrCurrLim;
	ptrCurrLim = (uint8_t *)&curr_lim;
	TX.data[0] = ptrVelLim[0];
	TX.data[1] = ptrVelLim[1];
	TX.data[2] = ptrVelLim[2];
	TX.data[3] = ptrVelLim[3];
	TX.data[4] = ptrCurrLim[0];
	TX.data[5] = ptrCurrLim[1];
	TX.data[6] = ptrCurrLim[2];
	TX.data[7] = ptrCurrLim[3];
	CAN_Send_Packet(&CAN, &TX);

}

void ODrive_RX_CallBack(Axis *AXIS){
	int32_t ID = 0;
	CAN_Get_Packet(&CAN, &RX);
	ID = RX.ID;
	int32_t NODE_ID = (ID >> 5);
	int32_t CMD_ID = (ID & 0x01F);


	if(NODE_ID == AXIS->AXIS_ID){

		switch(CMD_ID){

			case ODRIVE_HEARTBEAT_MESSAGE:
				AXIS->AXIS_Error = (RX.data[0] | RX.data[1]<<8 | RX.data[2]<<16 | RX.data[3]<<24);
				AXIS->AXIS_Current_State = RX.data[4];
				AXIS->Controller_Status = RX.data[5];
				break;


			case ENCODER_ESTIMATES: ;

				uint32_t *ptrEncPos;
				ptrEncPos = (uint32_t *)&(AXIS->AXIS_Encoder_Pos);
				*ptrEncPos = (RX.data[0] + (RX.data[1]<<8) + (RX.data[2]<<16) + (RX.data[3]<<24));
				uint32_t *ptrEncVel;
				ptrEncVel = (uint32_t *)&(AXIS->AXIS_Encoder_Vel);
				*ptrEncVel = (RX.data[4] + (RX.data[5]<<8) + (RX.data[6]<<16) + (RX.data[7]<<24));
				break;

			case GET_ENCODER_COUNT:
				AXIS->AXIS_Encoder_Shadow = (RX.data[0] | RX.data[1]<<8 | RX.data[2]<<16 | RX.data[3]<<24);
				AXIS->AXIS_Encoder_CPR = (RX.data[4] | RX.data[5]<<8 | RX.data[6]<<16 | RX.data[7]<<24);
				break;

			case GET_BUS_VOLTAGE_CURRENT: ;

				uint32_t *ptrBusV;
				ptrBusV = (uint32_t *)&(AXIS->AXIS_Bus_Voltage);
				*ptrBusV = (RX.data[0] + (RX.data[1]<<8) + (RX.data[2]<<16) + (RX.data[3]<<24));
				uint32_t *ptrBusI;
				ptrBusI = (uint32_t *)&(AXIS->AXIS_Bus_Current);
				*ptrBusI = (RX.data[4] + (RX.data[5]<<8) + (RX.data[6]<<16) + (RX.data[7]<<24));
				break;


			case GET_IQ: ;

				uint32_t *ptrIqSet;
				ptrIqSet = (uint32_t *)&(AXIS->AXIS_Iq_Setpoint);
				*ptrIqSet = (RX.data[0] + (RX.data[1]<<8) + (RX.data[2]<<16) + (RX.data[3]<<24));
				uint32_t *ptrIqMsr;
				ptrIqMsr = (uint32_t *)&(AXIS->AXIS_Iq_Measured);
				*ptrIqMsr = (RX.data[4] + (RX.data[5]<<8) + (RX.data[6]<<16) + (RX.data[7]<<24));
				break;


		}
	}
}
