#ifndef _SHOOT_TASK
#define _SHOOT_TASK

#include "main.h"
#include "Gimbal_Task.h"
#include "struct_typedef.h"
#include "pid.h"

#define MOTOR_RPM_TO_SPEED          0.00290888208665721596153948461415f


typedef enum {
    SHOOT_STOP = 0,
    SHOOT_READY,
	SHOOT_SINGLE,
	SHOOT_CONTINUOUS,
} shoot_mode_e;

// Trigger motor (Bo Dan Lun Dian Ji)
typedef struct
{
	fp32 speed;
  	fp32 speed_set;
	fp32 angle;
  	fp32 angle_set;
	fp32 ENC_angle;
	fp32 ENC_angle_set;
	fp32 trigger_angle;
  	int16_t give_current;
	fp32 ENC_ecd;
	fp32 ENC_last_ecd;

	int ENC_round_cnt;
	int block_time;
	int reverse_time;
	
	uint16_t heat_limit;
    uint16_t heat;

	pid_type_def speed_pid;
	pid_type_def angle_pid;

	shoot_mode_e shoot_mode;
} trigger_motor_t;

// Friction motor
typedef struct
{
  fp32 speed;
  fp32 speed_set;
	fp32 angle;
  fp32 angle_set;
	fp32 ENC_angle;
  int16_t give_current;
	
	int block_time;
	int reverse_time;
	
	pid_type_def speed_pid;
	
	pid_type_def angle_pid;
} friction_motor_t;

extern trigger_motor_t shoot_m2006;
extern friction_motor_t fric_m3508[2];

void Shoot_Task(void const * argument);

#endif
