#ifndef _CHASSIS_TASK
#define _CHASSIS_TASK

#include "main.h"
#include "struct_typedef.h"
#include "pid.h"

#define Chassis_No_Force 0
#define Chassis_Follow_Gimbal 1
#define Chassis_No_Follow_Gimbal 2
#define Chassis_Rotate 3

#define M3508_MOTOR_RPM_TO_VECTOR 0.001615809748903494517209f // 1:19 

typedef struct
{
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;
	
	pid_type_def pid;
}chassis_motor_t;

typedef struct
{
  fp32 vx;
	fp32 vy;
	fp32 wz;
  uint8_t mode;
  uint8_t last_mode;
	
  fp32 chassis_follow_gimbal_angle;	
	pid_type_def chassis_follow_gimbal_pid;
}chassis_control_t;

extern chassis_motor_t chassis_m3508[4];

void Chassis_Task(void const * argument);

#endif
