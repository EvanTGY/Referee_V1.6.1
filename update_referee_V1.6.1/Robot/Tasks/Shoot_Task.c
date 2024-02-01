#include "Shoot_Task.h"
#include "Gimbal_Task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "remote_control.h"
#include "bsp_can.h"
#include "bsp_tim.h"
#include "can.h"
#include "detect_task.h"
#include "referee.h"



#define BLOCK_TRIGGER_SPEED         1.0f //1.0f
#define BLOCK_TIME                  500
#define REVERSE_TIME                500
#define REVERSE_SPEED_LIMIT         13.0f

#define SHOOT_MOTOR_SPEED_PID_KP 8.0f
#define SHOOT_MOTOR_SPEED_PID_KI 0.01f
#define SHOOT_MOTOR_SPEED_PID_KD 0.0f
#define SHOOT_MOTOR_SPEED_PID_MAX_OUT 16000.0f
#define SHOOT_MOTOR_SPEED_PID_MAX_IOUT 1000.0f

#define SHOOT_MOTOR_ANGLE_PID_KP 18.0f
#define SHOOT_MOTOR_ANGLE_PID_KI 0.2f	 
#define SHOOT_MOTOR_ANGLE_PID_KD 0.0f
#define SHOOT_MOTOR_ANGLE_PID_MAX_OUT 2000.0f
#define SHOOT_MOTOR_ANGLE_PID_MAX_IOUT 1000.0f

#define FRIC_MOTOR_SPEED_PID_KP 10.0f
#define FRIC_MOTOR_SPEED_PID_KI 0.15f
#define FRIC_MOTOR_SPEED_PID_KD 0.0f
#define FRIC_MOTOR_SPEED_PID_MAX_OUT 15000.0f
#define FRIC_MOTOR_SPEED_PID_MAX_IOUT 10000.0f


trigger_motor_t shoot_m2006;
friction_motor_t fric_m3508[2];
ext_game_robot_state_t *robot_state_shoot;

shoot_mode_e *mode;

uint16_t trigger_RC_mode;
uint16_t trigger_RC_last_mode;
int stop_time;
int threshold;
int speed_set;
fp32 angle_single;


fp32 fabs_1( fp32 temp_val )  // abs
{
	if( temp_val < 0.0f )
		return -temp_val;
	return temp_val;
}


void Shoot_Motor_Init(void)
{
	const static fp32 shoot_motor_speed_pid[3] = {SHOOT_MOTOR_SPEED_PID_KP, SHOOT_MOTOR_SPEED_PID_KI, SHOOT_MOTOR_SPEED_PID_KD};
	const static fp32 shoot_motor_angle_pid[3] = {SHOOT_MOTOR_ANGLE_PID_KP, SHOOT_MOTOR_ANGLE_PID_KI, SHOOT_MOTOR_ANGLE_PID_KD};
	const static fp32 fric_motor_speed_pid[3] = {FRIC_MOTOR_SPEED_PID_KP, FRIC_MOTOR_SPEED_PID_KI, FRIC_MOTOR_SPEED_PID_KD};
	robot_state_shoot = get_robot_status_point();
	shoot_m2006.shoot_mode = SHOOT_STOP;
	stop_time = 299;
	speed_set=0;
	shoot_m2006.ENC_round_cnt=1;

	PID_init(&shoot_m2006.speed_pid,PID_POSITION,shoot_motor_speed_pid,SHOOT_MOTOR_SPEED_PID_MAX_OUT,SHOOT_MOTOR_SPEED_PID_MAX_IOUT);
	PID_init(&shoot_m2006.angle_pid,PID_POSITION,shoot_motor_angle_pid,SHOOT_MOTOR_ANGLE_PID_MAX_OUT,SHOOT_MOTOR_ANGLE_PID_MAX_IOUT);
	

	for(uint8_t i=0;i<2;i++)
	{
		fric_m3508[i].speed=0;
		fric_m3508[i].speed_set=0;
		fric_m3508[i].angle=0;
		fric_m3508[i].angle_set=0;
		fric_m3508[i].ENC_angle=0;
		fric_m3508[i].give_current=0;
		PID_init(&fric_m3508[i].speed_pid,PID_POSITION,fric_motor_speed_pid,FRIC_MOTOR_SPEED_PID_MAX_OUT,FRIC_MOTOR_SPEED_PID_MAX_IOUT);
	}
	
	mode = &shoot_m2006.shoot_mode;
}

void Shoot_Motor_Data_Update(void)
{
	static fp32 speed_fliter_1 = 0.0f;
  	static fp32 speed_fliter_2 = 0.0f;
  	static fp32 speed_fliter_3 = 0.0f;
	
	static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

  	speed_fliter_1 = speed_fliter_2;
 	speed_fliter_2 = speed_fliter_3;
  	speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (motor_measure_shoot[2].speed_rpm * MOTOR_RPM_TO_SPEED) * fliter_num[2];
 
  	shoot_m2006.speed = speed_fliter_3 * 180.0f / 3.1415926f;

	shoot_m2006.ENC_last_ecd = shoot_m2006.ENC_ecd;
	shoot_m2006.ENC_ecd = motor_measure_shoot[2].ecd;
	if (shoot_m2006.ENC_last_ecd < 2500 && shoot_m2006.ENC_ecd > 5600)
		shoot_m2006.ENC_round_cnt--;
	if (shoot_m2006.ENC_last_ecd > 5600 && shoot_m2006.ENC_ecd < 2500)
		shoot_m2006.ENC_round_cnt++; 
	shoot_m2006.ENC_angle = ( shoot_m2006.ENC_ecd / 8192.0f * 360.0f + shoot_m2006.ENC_round_cnt * 360.0f );
	shoot_m2006.trigger_angle = shoot_m2006.ENC_angle / 36.0f;

	// friction
	fric_m3508[0].speed = motor_measure_shoot[0].speed_rpm;
	fric_m3508[0].ENC_angle = motor_measure_shoot[0].ecd;
	
	fric_m3508[1].speed = motor_measure_shoot[1].speed_rpm;
	fric_m3508[1].ENC_angle = motor_measure_shoot[1].ecd;
}

void Trigger_Motor_Control(void)
{
	if ( shoot_m2006.shoot_mode == SHOOT_STOP){
		shoot_m2006.give_current = 0;
		shoot_m2006.angle_set = shoot_m2006.trigger_angle;
		return;
	}
	if (shoot_m2006.shoot_mode == SHOOT_READY)
	{
		PID_calc(&shoot_m2006.speed_pid,shoot_m2006.speed,shoot_m2006.speed_set);
		shoot_m2006.give_current = shoot_m2006.speed_pid.out;
		return;
	}
	shoot_m2006.give_current=0;
	// -ve => position direction
	if (shoot_m2006.shoot_mode == SHOOT_CONTINUOUS || shoot_m2006.shoot_mode == SHOOT_SINGLE){
		//block logic
		if (fabs_1(shoot_m2006.speed) < BLOCK_TRIGGER_SPEED && shoot_m2006.block_time < BLOCK_TIME)
		{
			shoot_m2006.block_time++;
			shoot_m2006.reverse_time = 0;
		}
		else if (shoot_m2006.block_time == BLOCK_TIME && shoot_m2006.reverse_time < REVERSE_TIME)
			shoot_m2006.reverse_time++;
		else
			shoot_m2006.block_time = 0;

		PID_calc(&shoot_m2006.angle_pid,shoot_m2006.trigger_angle,shoot_m2006.angle_set);
		shoot_m2006.speed_set = shoot_m2006.angle_pid.out;

		if (REVERSE_TIME < 0) // TBD: block
			shoot_m2006.speed_set = 10.0f;
		else if (shoot_m2006.shoot_mode == SHOOT_CONTINUOUS ) // continuous shoot
			shoot_m2006.speed_set = -10.0f * 180.0f / 3.1415926f;
		PID_calc(&shoot_m2006.speed_pid,shoot_m2006.speed,shoot_m2006.speed_set);
		shoot_m2006.give_current = shoot_m2006.speed_pid.out;
	}
}

void Fric_Motor_Control(void)
{

	if(shoot_m2006.shoot_mode == SHOOT_STOP)
	{
		speed_set -= 5; // 2s to acc
		if (speed_set < 0)
			speed_set = 0;
	}
	else if (shoot_m2006.shoot_mode == SHOOT_READY){
		speed_set += 5;
		if (speed_set > 9000)
			speed_set = 9000;
	}
	else
		speed_set = 9000;
	// speed_set = -speed_set;
	PID_calc( &fric_m3508[0].speed_pid, fric_m3508[0].speed, speed_set );
	fric_m3508[0].give_current = fric_m3508[0].speed_pid.out;
	PID_calc( &fric_m3508[1].speed_pid, fric_m3508[1].speed, -speed_set );
	fric_m3508[1].give_current = fric_m3508[1].speed_pid.out;
}

void Shoot_Motor_Mode_Set()
{	
	
	trigger_RC_last_mode = trigger_RC_mode;
	trigger_RC_mode = rc_ctrl.rc.s[1];
	// left switch down -> stop
	if (switch_is_down(trigger_RC_mode)) {
		*mode = SHOOT_STOP;
		return;
	}
	if (*mode == SHOOT_STOP){
		if (switch_is_mid(trigger_RC_mode)){
			*mode = SHOOT_READY;
		}
		return;
	}
	if (*mode == SHOOT_SINGLE){
		if (fabs_1(shoot_m2006.angle_set-shoot_m2006.trigger_angle)>1.0f || fabs_1(shoot_m2006.speed)>1.0f )
		{
			*mode = SHOOT_SINGLE;
		}
		else
		{
			*mode = SHOOT_READY;
		}
		return;
	}
	// if (!toe_is_error(REFEREE_TOE))
	if (0)
    {
		get_shoot_heat1_limit_and_heat0(&shoot_m2006.heat_limit, &shoot_m2006.heat);	
		if(stop_time)
		{
			stop_time--;
		}
		else if ((shoot_m2006.heat + 40 > shoot_m2006.heat_limit))// if overheat, enter fric enable state
		{
			shoot_m2006.shoot_mode = SHOOT_STOP;
			stop_time = 300;
			return;
		}
    }

	if (*mode == SHOOT_READY){
		shoot_m2006.speed_set = 0.0f;
		if (switch_is_mid(trigger_RC_last_mode) && switch_is_up(trigger_RC_mode)){
			*mode = SHOOT_SINGLE;
			shoot_m2006.angle_set = shoot_m2006.trigger_angle - 45.0f;
		}
		else if (rc_ctrl.rc.ch[4] > 330){
			*mode = SHOOT_CONTINUOUS;
		}
		return;
	}

	if (*mode == SHOOT_CONTINUOUS){
		if (rc_ctrl.rc.ch[4] < 330){
			*mode = SHOOT_READY;
		}
		return;
	}
}

void Shoot_Task(void const * argument)
{
	Shoot_Motor_Init();
	vTaskDelay(200);
	
	while(1)
	{
		Shoot_Motor_Data_Update();
		Shoot_Motor_Mode_Set();
		Fric_Motor_Control();
		Trigger_Motor_Control();
		// fric_m3508[0].give_current = 0; // debug
		// fric_m3508[1].give_current = 0; // debug
		CAN_Shoot_CMD( fric_m3508[0].give_current, fric_m3508[1].give_current, shoot_m2006.give_current );
		// printf("%.3f\r\n", shoot_m2006.angle_pid.out); //debug
		// printf("%.3f\r\n", shoot_m2006.speed_pid.out); //debug
		vTaskDelay(2);
	}
}
