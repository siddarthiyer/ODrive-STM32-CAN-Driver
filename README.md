# ODrive CAN Driver(STM32)

Library written by me to control ODrive Motors with STM32 MCU via CAN </br></br>
[Reference](https://docs.odriverobotics.com/v/beta/can-protocol.html)
<h3> Functions: </h3>

```C
void CAN_Setup(CAN_TypeDef *CAN_INSTANCE, int32_t baudrate);
void Set_Axis_Requested_State(Axis Axis, Axis_State state);
void Set_Input_Vel(Axis Axis, float vel, float torqueff);
void Set_TX_Param(int AXIS_ID, int COMMAND_ID, int id_type, int frame_type, int data_length);
void Clear_Errors(Axis Axis);
void Reboot_ODrive(Axis AXIS_ID);
void Set_Controller_Modes(Axis Axis, Control_Mode ControlMode, Input_Mode InputMode);
void Set_Input_Pos(Axis Axis, float Input_Pos, int Vel_FF, int Torque_FF);
void Get_Encoder_Count(Axis Axis);
void Set_Input_Torque(Axis Axis, float torque);
void Get_Bus_Voltage_Current(Axis Axis);
void Get_IQ(Axis Axis);
void Set_Position_Gain(Axis Axis, float pos_gain);
void Set_Vel_Gains(Axis Axis, float Vel_Gain, float Vel_Int_Gain);
void Set_Axis_Node_ID(Axis Axis, uint32_t node_id);
void Set_Limits(Axis Axis, float vel_lim, float curr_lim);
void ODrive_RX_CallBack(Axis *AXIS);

```

<h4> Declare a struct of type 'Axis'</h4>

```C
struct Axis AXIS0;
```

<h4>
  Struct declaration
</h4>

```C
typedef struct Axis{
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


```
