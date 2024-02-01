#ifndef _GIMBAL_TASK
#define _GIMBAL_TASK

#include "struct_typedef.h"
#include "pid.h"


#define YAW_MOTOR_INIT_POS 4950
#define PITCH_MOTOR_INIT_POS 4016
#define Gimbal_No_Force (uint8_t)0
#define Gimbal_Normal_Mode (uint8_t)1
#define Gimbal_Auto_Aiming (uint8_t)2
#define Gimbal_Motor_Encoder (uint8_t)1
#define Gimbal_Motor_INS (uint8_t)2
#define Yaw (uint8_t)0
#define Pitch (uint8_t)1

typedef struct
{
	fp32 ENC_ecd;
	fp32 ENC_last_ecd;

	fp32 ENC_angle; // Encoder angle [0,8191)
	fp32 ENC_relative_angle;

	fp32 angle_set;
	fp32 INS_speed;
	fp32 INS_speed_set;
	fp32 INS_angle;
	fp32 INS_angle_offset;
	fp32 ENC_speed; // Encoder speed (rpm -> deg/s)
	fp32 ENC_speed_set; 
	fp32 ENC_round_cnt; // circle count
	fp32 origin;

	uint8_t motor_mode;
	uint8_t last_motor_mode;

	pid_type_def ENC_speed_pid; // pid for speed loop
	pid_type_def INS_speed_pid;
	pid_type_def ENC_angle_pid; // pid for angle loop
	pid_type_def INS_angle_pid;
	pid_type_def auto_aim_pid;

	int16_t give_current; 
} gimbal_motor_t;

typedef struct
{
	gimbal_motor_t gimbal_m6020[2];
	uint8_t mode;
	uint8_t last_mode;
	int init_flag;
} gimbal_control_t;

extern gimbal_control_t gimbal_control;

void Gimbal_Task(void const * argument);

#endif
