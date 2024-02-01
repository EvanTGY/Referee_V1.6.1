#include "Chassis_Task.h"
#include "power_control.h"
#include "Gimbal_Task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "can.h"
#include "remote_control.h"
#include "bsp_can.h"
#include "arm_math.h"

#define M3505_MOTOR_SPEED_PID_KP 100.0f
#define M3505_MOTOR_SPEED_PID_KI 0.05f
#define M3505_MOTOR_SPEED_PID_KD 0.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT 16000.0f
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

#define CHASSIS_FOLLOW_GIMBAL_PID_KP 0.5f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.00f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 5.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 2.0f


chassis_motor_t chassis_m3508[4];
chassis_control_t chassis_control;

extern gimbal_control_t gimbal_control;


void Chassis_Motor_Init(void)
{
	const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
	
	for(uint8_t i=0;i<4;i++)
	{
		chassis_m3508[i].speed=0;
		chassis_m3508[i].speed_set=0;
		chassis_m3508[i].give_current=0;
		
		PID_init(&chassis_m3508[i].pid,PID_POSITION,motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
	}

	const static fp32 chassis_follow_gimbal_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP,CHASSIS_FOLLOW_GIMBAL_PID_KI,CHASSIS_FOLLOW_GIMBAL_PID_KD};
	PID_init(&chassis_control.chassis_follow_gimbal_pid,PID_POSITION,chassis_follow_gimbal_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);

}

void Chassis_Motor_Data_Update(void)
{
	for(uint8_t i=0;i<4;i++)
	{
		chassis_m3508[i].speed= M3508_MOTOR_RPM_TO_VECTOR * motor_measure_chassis[i].speed_rpm;
	}
}

static void Chassis_Vector_to_Mecanum_Wheel_Speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])
{

	wheel_speed[0] = -(-vx_set*sqrt(2) - vy_set*sqrt(2) + wz_set * 0.4)/0.0763;
	wheel_speed[1] = -(vx_set*sqrt(2) - vy_set*sqrt(2) + wz_set * 0.4)/0.0763;
	wheel_speed[2] = -(vx_set*sqrt(2) + vy_set*sqrt(2) + wz_set * 0.4)/0.0763;
	wheel_speed[3] = -(-vx_set*sqrt(2) + vy_set*sqrt(2) + wz_set * 0.4)/0.0763;
}	

void Chassis_PID_Calculator(void)
{
	if ( chassis_control.mode == Chassis_No_Force ) 
	{
		for( uint8_t i = 0; i < 4; i++ )
			chassis_m3508[i].give_current = 0;
	}
	else
	{
		for( uint8_t i = 0; i < 4; i++ )
		{
			PID_calc(&chassis_m3508[i].pid,chassis_m3508[i].speed,chassis_m3508[i].speed_set);
			chassis_m3508[i].give_current=chassis_m3508[i].pid.out;	
		}
	}
	
}

void Chassis_RC_Mode_Set(void)
{
	chassis_control.last_mode = chassis_control.mode;
	if( switch_is_down(rc_ctrl.rc.s[0]) )
		chassis_control.mode = Chassis_No_Force;
	else if( switch_is_up(rc_ctrl.rc.s[0]) )
		chassis_control.mode = Chassis_No_Follow_Gimbal;
	else if ( switch_is_mid(rc_ctrl.rc.s[0]) )
	{
		chassis_control.mode = Chassis_Follow_Gimbal;	
		if( rc_ctrl.rc.ch[4] < -330 )
			chassis_control.mode = Chassis_Rotate;
		else if (gimbal_control.init_flag)
			chassis_control.mode = Chassis_No_Force;
	}
	else
		chassis_control.mode = Chassis_No_Force;	
}

static void Chassis_Motor_Control(void)
{

	fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
	fp32 vx,vy;

	if ( chassis_control.mode == Chassis_Follow_Gimbal && gimbal_control.init_flag == 0)
	{
		vx=(fp32)rc_ctrl.rc.ch[1]/500.0f; // be careful about the direction of vx and vy
		vy=(fp32)rc_ctrl.rc.ch[0]/500.0f;
		sin_yaw = arm_sin_f32(gimbal_control.gimbal_m6020[0].ENC_relative_angle/180.0f*3.14159f);
		cos_yaw = arm_cos_f32(gimbal_control.gimbal_m6020[0].ENC_relative_angle/180.0f*3.14159f);
		chassis_control.vx = cos_yaw * vx + sin_yaw * vy;
    	chassis_control.vy = sin_yaw * vx - cos_yaw * vy;
		// Mind this minus sign
		chassis_control.wz = -PID_calc(&chassis_control.chassis_follow_gimbal_pid, gimbal_control.gimbal_m6020[0].ENC_relative_angle, 0.0f);
	}
	else if (chassis_control.mode == Chassis_Rotate )
	{
		//rotate
		vx=(fp32)rc_ctrl.rc.ch[1]/500.0f;
		vy=(fp32)rc_ctrl.rc.ch[0]/500.0f;
		sin_yaw = arm_sin_f32(gimbal_control.gimbal_m6020[0].ENC_relative_angle/180.0f*3.14159f);
		cos_yaw = arm_cos_f32(gimbal_control.gimbal_m6020[0].ENC_relative_angle/180.0f*3.14159f);
		chassis_control.vx = cos_yaw * vx + sin_yaw * vy;
   		chassis_control.vy = sin_yaw * vx - cos_yaw * vy;
		chassis_control.wz = 2.0f;
	}
	else if ( chassis_control.mode == Chassis_No_Follow_Gimbal )
	{	
		vx=(fp32)rc_ctrl.rc.ch[1]/500.0f; // be careful about the direction of vx and vy
		vy=(fp32)rc_ctrl.rc.ch[0]/500.0f;
		sin_yaw = arm_sin_f32(gimbal_control.gimbal_m6020[0].ENC_relative_angle/180.0f*3.14159f);
		cos_yaw = arm_cos_f32(gimbal_control.gimbal_m6020[0].ENC_relative_angle/180.0f*3.14159f);
		chassis_control.vx = cos_yaw * vx + sin_yaw * vy;
		chassis_control.vy = sin_yaw * vx - cos_yaw * vy;
		chassis_control.wz = 0;
	}
	else if ( chassis_control.mode == Chassis_No_Force )
	{
		chassis_control.vx = 0;
		chassis_control.vy = 0;
		chassis_control.wz = 0;
	}


	fp32 motor_speed[4];
	Chassis_Vector_to_Mecanum_Wheel_Speed(chassis_control.vx,chassis_control.vy,chassis_control.wz,motor_speed);

	for(uint8_t i=0;i<4;i++)
	{
		chassis_m3508[i].speed_set = motor_speed[i];
	}

	Chassis_PID_Calculator();
}

void Chassis_Task(void const * argument)
{
	Chassis_Motor_Init();
	vTaskDelay(200);	

	while(1)
	{
		Chassis_Motor_Data_Update();
		Chassis_RC_Mode_Set();
		Chassis_Motor_Control();		
		
		power_control(&chassis_control); // power control without capacitor

		CAN_Chassis_CMD(chassis_m3508[0].give_current,chassis_m3508[1].give_current,chassis_m3508[2].give_current,chassis_m3508[3].give_current);
		vTaskDelay(2);
	}
}
