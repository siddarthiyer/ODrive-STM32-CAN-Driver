/*
 * ODrive.h
 *
 *  Created on: 07-Jul-2022
 *      Author: sidiyer27
 */

#ifndef ODRIVE_ODRIVE_H_
#define ODRIVE_ODRIVE_H_

#include "main.h"
#include "CAN.h"


//COMMAND ID
#define ODRIVE_HEARTBEAT_MESSAGE		0x001
#define SET_AXIS_NODE_ID				0x006
#define SET_AXIS_REQUESTED_STATE 		0x007
#define ENCODER_ESTIMATES				0x009
#define GET_ENCODER_COUNT				0x00A
#define SET_CONTROLLER_MODES			0x00B
#define SET_INPUT_POS					0x00C
#define SET_INPUT_VEL					0x00D
#define SET_INPUT_TORQUE				0x00E
#define SET_LIMITS						0x00F
#define GET_IQ							0x014
#define REBOOT_ODRIVE					0x016
#define GET_BUS_VOLTAGE_CURRENT			0x017
#define CLEAR_ERRORS					0x018
#define SET_POSITION_GAIN				0x01A
#define SET_VEL_GAINS					0x01B


//Axis Parameters
typedef struct Axis
{
	int AXIS_ID;
	float AXIS_Encoder_Pos;
	float AXIS_Encoder_Vel;
	int32_t AXIS_Encoder_CPR;
	int32_t AXIS_Encoder_Shadow;
	float AXIS_Bus_Voltage;
	float AXIS_Bus_Current;
	float AXIS_Iq_Setpoint;
	float AXIS_Iq_Measured;
	uint32_t AXIS_Error;
	uint8_t AXIS_Current_State;
	uint8_t Controller_Status;
	CAN_TypeDef *CAN_INSTANCE;
}Axis;

//Axis States
typedef enum{
	UNDEFINED = 0x0,
	IDLE = 0x1,
	STARTUP_SEQUENCE = 0x2,
	FULL_CALIBRATION_SEQUENCE = 0x3,
	MOTOR_CALIBRATION = 0x4,
	ENCODER_INDEX_SEARCH = 0x6,
	ENCODER_OFFSET_CALIBRATION = 0x7,
	CLOSED_LOOP_CONTROL = 0x8,
	LOCKIN_SPIN = 0x9,
	ENCODER_DIR_FIND = 0xA,
	HOMING = 0xB,
	ENCODER_HALL_POLARITY_CALIBRATION = 0xC,
	ENCODER_HALL_PHASE_CALIBRATION = 0xD
} Axis_State;

//Control Modes
typedef enum{
	VOLTAGE_CONTROL = 0x0,
	TORQUE_CONTROL = 0x1,
	VELOCITY_CONTROL = 0x2,
	POSITION_CONTROL = 0x3
} Control_Mode;

//Input Modes
typedef enum{
	INACTIVE = 0x0,
	PASSTHROUGH = 0x1,
	VEL_RAMP = 0x2,
	POS_FILTER = 0x3,
	MIX_CHANNELS = 0x4,
	TRAP_TRAJ = 0x5,
	TORQUE_RAMP = 0x6,
	MIRROR = 0x7,
	TUNING = 0x8
} Input_Mode;


/*
 * @function	: CAN_Setup
 * @version		: 1
 * @date 		: 14-07-2022
 * @parameter	: 1. CAN_INSTANCE
 * 					@Type: CAN_TypeDef *
 * 					@brief: Defines which CAN Instance is used(CAN1, CAN2)
 * 				  2. baudrate
 * 				  	@Type: int32_t
 * 					@brief: Sets baudrate of CAN. (Use macros defined in CAN_Defines. E.g CAN_BAUDRATE_250_KBPS)

 * @return type	: none
 * @brief		: Used to initialize the CAN instance to be used with the ODrive CAN Bus. Pins used: CAN1(PA11, PA12), CAN2(PB12,PB13)).
 * 				  These pins will be connected to a CAN transceiver as a front end. Baudrate to be set depending on baudrate of ODrive.
 */

void CAN_Setup(CAN_TypeDef *CAN_INSTANCE, int32_t baudrate);



/*
 * @function	: Set_Axis_Requested_State
 * @version		: 1
 * @date 		: 14-07-2022
 * @parameter	: 1. Axis
 * 					@Type: Axis (struct)
 * 					@brief: Defines Axis Instance
 * 				  2. state
 * 				  	@Type: Axis_State (enum)
 * 					@brief: Used to select Axis_State
 * @return type	: none
 * @brief		: Sends a CAN Data Packet with the required Axis ID and Command ID to command the axis
 * 				  to change state or perform certain routines. List of Axis States listed above.
 */

void Set_Axis_Requested_State(Axis Axis, Axis_State state);



/*
 * @function	: Set_Input_Vel
 * @version		: 1
 * @date 		: 14-07-2022
 * @parameter	: 1. Axis
 * 					@Type: Axis (struct)
 * 					@brief: Defines Axis Instance
 * 				  2. vel
 * 				  	@Type: float
 * 					@brief: Sets the desired velocity of the axis
 * 				  3. torqueff
 * 				    @Type: float
 * 				    @brief: sets the feed-forward torque of the torque controller
 * @return type	: none
 * @brief		: Sends a CAN Data Packet with the required Axis ID and Command ID to set the desired
 * 				  velocity of the axis and the the feed-forward torque of the torque controller
 */

void Set_Input_Vel(Axis Axis, float vel, float torqueff);



/*
 * @function	: Set_TX_Param
 * @version		: 1
 * @date 		: 14-07-2022
 * @parameter	: 1. AXIS_ID
 * 					@Type: int
 * 					@brief: Axis ID of the given Axis
 * 				  2. COMMAND_ID
 * 				  	@Type: int
 * 					@brief: Command ID of command to be executed
 * 				  3. id_type
 * 				    @Type: int
 * 				    @brief: Used to select standard or extended ID(CAN_ID_Standard,CAN_ID_Extended)
 * 				  4. frame_type
 * 				  	@Type: int
 * 					@brief: Select frame type: RTR or Data(CAN_Frame_Data, CAN_Frame_Remote)
 * 				  5. data_length
 * 				    @Type: int
 * 				    @brief: Set data length of packet to be sent
 * @return type	: none
 * @brief		: Used to set the CAN TX Struct Parameters such as data_length, frame_type, id_type, ID
 */

void Set_TX_Param(int AXIS_ID, int COMMAND_ID, int id_type, int frame_type, int data_length);



/*
 * @function	: Clear_Errors
 * @version		: 1
 * @date 		: 14-07-2022
 * @parameter	: 1. Axis
 * 					@Type: Axis (struct)
 * 					@brief: Defines Axis Instance

 * @return type	: none
 * @brief		: Sends a CAN Data Packet with the required Axis ID and Command ID to clear all
 * 				  the errors of this device including all contained submodules.
 */

void Clear_Errors(Axis Axis);



/*
 * @function	: Reboot_ODrive
 * @version		: 1
 * @date 		: 14-07-2022
 * @parameter	: 1. Axis
 * 					@Type: Axis (struct)
 * 					@brief: Defines Axis Instance

 * @return type	: none
 * @brief		: Sends a CAN Data Packet with the required Axis ID and Command ID to reboot
 * 				  the controller without saving the current configuraiton.
 * 				  Message can be sent to either address on a given ODrive board.
 */

void Reboot_ODrive(Axis AXIS_ID);



/*
 * @function	: Set_Controller_Modes
 * @version		: 1
 * @date 		: 14-07-2022
 * @parameter	: 1. Axis
 * 					@Type: Axis (struct)
 * 					@brief: Defines Axis Instance
 * 				  2. ControlMode
 * 				  	@Type: Control_Mode (enum declared above)
 * 					@brief: Sets the control mode of specified axis
 * 				  3. torqueff
 * 				    @Type: Input_Mode (enum declared above)
 * 				    @brief: sets the input mode of specified axis
 * @return type	: none
 * @brief		: Sends a CAN Data Packet with the required Axis ID and Command ID to
 * 				  set the Control Mode and Input Mode of given axis
 */

void Set_Controller_Modes(Axis Axis, Control_Mode ControlMode, Input_Mode InputMode);



/*
 * @function	: Set_Input_Pos
 * @version		: 1
 * @date 		: 14-07-2022
 * @parameter	: 1. Axis
 * 					@Type: Axis (struct)
 * 					@brief: Defines Axis Instance
 * 				  2. Input_Pos
 * 				  	@Type: float
 * 					@brief: Set the desired position of the axis
 * 				  3. Vel_FF
 * 				    @Type: float
 * 				    @brief: sets the feed-forward velocity of the velocity controller
 * 				 3. Torque_FF
 * 				    @Type: float
 * 				    @brief: sets the feed-forward torque of the torque controller
 * @return type	: none
 * @brief		: Sends a CAN Data Packet with the required Axis ID and Command ID to set
 * 				  the desired position of the axis as well as the feed-forward velocity
 * 				  and feed-forward torque
 */

void Set_Input_Pos(Axis Axis, float Input_Pos, int Vel_FF, int Torque_FF);



/*
 * @function	: Get_Encoder_Count
 * @version		: 1
 * @date 		: 14-07-2022
 * @parameter	: 1. Axis
 * 					@Type: Axis (struct)
 * 					@brief: Defines Axis Instance
 * @return type	: none
 * @brief		: Sends a CAN RTR Frame with required ID to the ODrive to request Encoder Shadow Count
 * 				  and Encoder Count in CPR.
 * 				  Note: Function only sends the RTR frame. Data will be received and variables will be updated
 * 				  via the CallBack function when an reception interrupt is triggered.
 *
 */

void Get_Encoder_Count(Axis Axis);



/*
 * @function	: Set_Input_Torque
 * @version		: 1
 * @date 		: 14-07-2022
 * @parameter	: 1. Axis
 * 					@Type: Axis (struct)
 * 					@brief: Defines Axis Instance
 * 				  2. torque
 * 				  	@Type: float
 * 					@brief: Sets the desired output torque of the axis
 * @return type	: none
 * @brief		: Sends a CAN Data Packet with the required Axis ID and Command ID to set
 * 				  the desired torque of the axis.
 */

void Set_Input_Torque(Axis Axis, float torque);



/*
 * @function	: Get_Bus_Voltage_Current
 * @version		: 1
 * @date 		: 14-07-2022
 * @parameter	: 1. Axis
 * 					@Type: Axis (struct)
 * 					@brief: Defines Axis Instance
 * @return type	: none
 * @brief		: Sends a CAN RTR Frame with required ID to the ODrive to request Bus Voltage and Bus Current.
 * 				  Note: Function only sends the RTR frame. Data will be received and variables will be updated
 * 				  via the CallBack function when an reception interrupt is triggered.
 *
 */

void Get_Bus_Voltage_Current(Axis Axis);



/*
 * @function	: Get_IQ
 * @version		: 1
 * @date 		: 14-07-2022
 * @parameter	: 1. Axis
 * 					@Type: Axis (struct)
 * 					@brief: Defines Axis Instance
 * @return type	: none
 * @brief		: Sends a CAN RTR Frame with required ID to the ODrive to request Iq Setpoint and Iq Measured.
 * 				  Note: Function only sends the RTR frame. Data will be received and variables will be updated
 * 				  via the CallBack function when an reception interrupt is triggered.
 *
 */

void Get_IQ(Axis Axis);



/*
 * @function	: Set_Position_Gain
 * @version		: 1
 * @date 		: 14-07-2022
 * @parameter	: 1. Axis
 * 					@Type: Axis (struct)
 * 					@brief: Defines Axis Instance
 * 				  2. pos_gain
 * 				  	@Type: float
 * 					@brief: Sets the desired position gain
 * @return type	: none
 * @brief		: Sends a CAN Data Packet with the required Axis ID and Command ID to set
 * 				  the desired position gain
 */

void Set_Position_Gain(Axis Axis, float pos_gain);



/*
 * @function	: Set_Position_Gain
 * @version		: 1
 * @date 		: 14-07-2022
 * @parameter	: 1. Axis
 * 					@Type: Axis (struct)
 * 					@brief: Defines Axis Instance
 * 				  2. Vel_Gain
 * 				  	@Type: float
 * 					@brief: Sets the desired velocity gain
 * 				  3. Vel_Int_Gain
 * 				  	@Type: float
 * 					@brief: Sets the desired velocity integrator gain
 * @return type	: none
 * @brief		: Sends a CAN Data Packet with the required Axis ID and Command ID to set
 * 				  the desired velocity gain and velocity integrator gain
 */

void Set_Vel_Gains(Axis Axis, float Vel_Gain, float Vel_Int_Gain);



/*
 * @function	: Set_Position_Gain
 * @version		: 1
 * @date 		: 14-07-2022
 * @parameter	: 1. Axis
 * 					@Type: Axis (struct)
 * 					@brief: Defines Axis Instance
 * 				  2. node_id
 * 				  	@Type: uint32_t
 * 					@brief: Updated Node Id
 * @return type	: none
 * @brief		: Sends a CAN Data Packet with the required Axis ID and Command ID to update
 * 				  the given axis' Node Id
 */

void Set_Axis_Node_ID(Axis Axis, uint32_t node_id);



/*
 * @function	: Set_Limits
 * @version		: 1
 * @date 		: 14-07-2022
 * @parameter	: 1. Axis
 * 					@Type: Axis (struct)
 * 					@brief: Defines Axis Instance
 * 				  2. vel_lim
 * 				  	@Type: float
 * 					@brief: Sets the velocity limit
 * 				  3. curr_lim
 * 				  	@Type: float
 * 					@brief: Sets the current limit
 * @return type	: none
 * @brief		: Sends a CAN Data Packet with the required Axis ID and Command ID to update
 * 				  the given axis' Node Id
 */

void Set_Limits(Axis Axis, float vel_lim, float curr_lim);



/*
 * @function	: ODrive_RX_CallBack
 * @version		: 1
 * @date 		: 14-07-2022
 * @parameter	: 1. AXIS
 * 					@Type: Axis * (pointer to struct)
 * 					@brief: Pointer to Axis Instance
 * @return type	: none
 * @brief		: This function is used as a callback function whenever an CAN RX interrupt occurs.
 * 				  It is used to receive data from the ODrive when an RTR frame is sent to it as well
 * 				  as the heartbeat message and Encoder Count sent at a set frequency. The data is used
 * 				  to update the required variables in the Axis struct.
 */

void ODrive_RX_CallBack(Axis *AXIS);



#endif /* ODRIVE_ODRIVE_H_ */
