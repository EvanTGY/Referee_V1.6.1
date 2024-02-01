#include "Gimbal_Task.h"
#include "INS_Task.h"
#include "Shoot_Task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "can.h"
#include "remote_control.h"
#include "bsp_can.h"
// #include "auto_aim.h"
#include "USB_Task.h"

#include "arm_math.h"

#define PITCH_UP_BOUND 39.3f
#define PITCH_DOWN_BOUND -32.4f

#define YAW_MOTOR_ENC_SPEED_PID_KP 0.0f
#define YAW_MOTOR_ENC_SPEED_PID_KI 0.0f
#define YAW_MOTOR_ENC_SPEED_PID_KD 0.0f
#define YAW_MOTOR_ENC_SPEED_PID_MAX_OUT 0.0f
#define YAW_MOTOR_ENC_SPEED_PID_MAX_IOUT 0.0f

#define YAW_MOTOR_ENC_ANGLE_PID_KP 10.5f
#define YAW_MOTOR_ENC_ANGLE_PID_KI 0.0f
#define YAW_MOTOR_ENC_ANGLE_PID_KD 0.0f
#define YAW_MOTOR_ENC_ANGLE_PID_MAX_OUT 70.0f
#define YAW_MOTOR_ENC_ANGLE_PID_MAX_IOUT 0.0f

#define YAW_MOTOR_INS_SPEED_PID_KP 200.0f // 70.0f
#define YAW_MOTOR_INS_SPEED_PID_KI 1.1f
#define YAW_MOTOR_INS_SPEED_PID_KD 0.1f
#define YAW_MOTOR_INS_SPEED_PID_MAX_OUT 30000.0f
#define YAW_MOTOR_INS_SPEED_PID_MAX_IOUT 10000.0f

#define YAW_MOTOR_INS_ANGLE_PID_KP 30.0f // 15.0f
// #define YAW_MOTOR_INS_ANGLE_PID_KP 10.5f
#define YAW_MOTOR_INS_ANGLE_PID_KI 0.0f
#define YAW_MOTOR_INS_ANGLE_PID_KD 0.0f
#define YAW_MOTOR_INS_ANGLE_PID_MAX_OUT 100.0f
#define YAW_MOTOR_INS_ANGLE_PID_MAX_IOUT 0.0f

#define YAW_MOTOR_AUTO_AIM_PID_KP 0.05f
#define YAW_MOTOR_AUTO_AIM_PID_KI 0.0f
#define YAW_MOTOR_AUTO_AIM_PID_KD 0.0f
#define YAW_MOTOR_AUTO_AIM_PID_MAX_OUT 3.0f
#define YAW_MOTOR_AUTO_AIM_PID_MAX_IOUT 100.0f

#define PITCH_MOTOR_ENC_SPEED_PID_KP 100.0f
#define PITCH_MOTOR_ENC_SPEED_PID_KI 1.0f
#define PITCH_MOTOR_ENC_SPEED_PID_KD 0.0f
#define PITCH_MOTOR_ENC_SPEED_PID_MAX_OUT 30000.0f
#define PITCH_MOTOR_ENC_SPEED_PID_MAX_IOUT 10000.0f

#define PITCH_MOTOR_ENC_ANGLE_PID_KP 7.0f
#define PITCH_MOTOR_ENC_ANGLE_PID_KI 0.0f
#define PITCH_MOTOR_ENC_ANGLE_PID_KD 0.0f
#define PITCH_MOTOR_ENC_ANGLE_PID_MAX_OUT 100.0f
#define PITCH_MOTOR_ENC_ANGLE_PID_MAX_IOUT 1.0f

#define PITCH_MOTOR_INS_SPEED_PID_KP 100.0f
#define PITCH_MOTOR_INS_SPEED_PID_KI 1.0f
#define PITCH_MOTOR_INS_SPEED_PID_KD 0.0f
#define PITCH_MOTOR_INS_SPEED_PID_MAX_OUT 30000.0f
#define PITCH_MOTOR_INS_SPEED_PID_MAX_IOUT 10000.0f

#define PITCH_MOTOR_INS_ANGLE_PID_KP 7.0f
#define PITCH_MOTOR_INS_ANGLE_PID_KI 0.0f
#define PITCH_MOTOR_INS_ANGLE_PID_KD 0.0f
#define PITCH_MOTOR_INS_ANGLE_PID_MAX_OUT 100.0f
#define PITCH_MOTOR_INS_ANGLE_PID_MAX_IOUT 1.0f

#define PITCH_MOTOR_AUTO_AIM_PID_KP 0.01f
#define PITCH_MOTOR_AUTO_AIM_PID_KI 0.0f
#define PITCH_MOTOR_AUTO_AIM_PID_KD 0.0f
#define PITCH_MOTOR_AUTO_AIM_PID_MAX_OUT 3.0f
#define PITCH_MOTOR_AUTO_AIM_PID_MAX_IOUT 100.0f

gimbal_control_t gimbal_control;
gimbal_motor_t *pitch, *yaw;
extern vision_rx_t vision_rx;


fp32 fabs_2( fp32 temp_val )
{
	if( temp_val < 0.0f )
		return -temp_val;
	return temp_val;
}

gimbal_motor_t* get_point(uint8_t X)
{
	return &gimbal_control.gimbal_m6020[X];
}

void Gimbal_Motor_Init(void)
{
	pitch = get_point(Pitch);
	yaw = get_point(Yaw);
	const static fp32 yaw_motor_enc_speed_pid[3] = {YAW_MOTOR_ENC_SPEED_PID_KP, YAW_MOTOR_ENC_SPEED_PID_KI, YAW_MOTOR_ENC_SPEED_PID_KD};
	const static fp32 yaw_motor_enc_angle_pid[3] = {YAW_MOTOR_ENC_ANGLE_PID_KP, YAW_MOTOR_ENC_ANGLE_PID_KI, YAW_MOTOR_ENC_ANGLE_PID_KD};
	const static fp32 yaw_motor_ins_speed_pid[3] = {YAW_MOTOR_INS_SPEED_PID_KP, YAW_MOTOR_INS_SPEED_PID_KI, YAW_MOTOR_INS_SPEED_PID_KD};
	const static fp32 yaw_motor_ins_angle_pid[3] = {YAW_MOTOR_INS_ANGLE_PID_KP, YAW_MOTOR_INS_ANGLE_PID_KI, YAW_MOTOR_INS_ANGLE_PID_KD};
	const static fp32 yaw_motor_auto_aim_pid[3] = {YAW_MOTOR_AUTO_AIM_PID_KP, YAW_MOTOR_AUTO_AIM_PID_KI, YAW_MOTOR_AUTO_AIM_PID_KD};
	
	const static fp32 pitch_motor_enc_speed_pid[3] = {PITCH_MOTOR_ENC_SPEED_PID_KP, PITCH_MOTOR_ENC_SPEED_PID_KI, PITCH_MOTOR_ENC_SPEED_PID_KD};
	const static fp32 pitch_motor_enc_angle_pid[3] = {PITCH_MOTOR_ENC_ANGLE_PID_KP, PITCH_MOTOR_ENC_ANGLE_PID_KI, PITCH_MOTOR_ENC_ANGLE_PID_KD};
	const static fp32 pitch_motor_ins_speed_pid[3] = {PITCH_MOTOR_INS_SPEED_PID_KP, PITCH_MOTOR_INS_SPEED_PID_KI, PITCH_MOTOR_INS_SPEED_PID_KD};
	const static fp32 pitch_motor_ins_angle_pid[3] = {PITCH_MOTOR_INS_ANGLE_PID_KP, PITCH_MOTOR_INS_ANGLE_PID_KI, PITCH_MOTOR_INS_ANGLE_PID_KD};
	const static fp32 pitch_motor_auto_aim_pid[3] = {PITCH_MOTOR_AUTO_AIM_PID_KP, PITCH_MOTOR_AUTO_AIM_PID_KI, PITCH_MOTOR_AUTO_AIM_PID_KD};
	
	for(uint8_t i=0;i<2;i++)
	{
		gimbal_control.gimbal_m6020[i].INS_speed=0;
		gimbal_control.gimbal_m6020[i].INS_speed_set=0;
		gimbal_control.gimbal_m6020[i].INS_angle=0;
		gimbal_control.gimbal_m6020[i].angle_set=0;
		gimbal_control.gimbal_m6020[i].ENC_angle=0;
		gimbal_control.gimbal_m6020[i].angle_set=0;
		gimbal_control.gimbal_m6020[i].give_current=0;
		gimbal_control.mode = Gimbal_No_Force;
		gimbal_control.last_mode = Gimbal_No_Force;
	}
	PID_init(&(yaw->ENC_speed_pid),PID_POSITION,yaw_motor_enc_speed_pid,YAW_MOTOR_ENC_SPEED_PID_MAX_OUT,YAW_MOTOR_ENC_SPEED_PID_MAX_IOUT);
	PID_init(&(yaw->ENC_angle_pid),PID_POSITION,yaw_motor_enc_angle_pid,YAW_MOTOR_ENC_ANGLE_PID_MAX_OUT,YAW_MOTOR_ENC_ANGLE_PID_MAX_IOUT);
	PID_init(&(yaw->INS_speed_pid),PID_POSITION,yaw_motor_ins_speed_pid,YAW_MOTOR_INS_SPEED_PID_MAX_OUT,YAW_MOTOR_INS_SPEED_PID_MAX_IOUT);
	PID_init(&(yaw->INS_angle_pid),PID_POSITION,yaw_motor_ins_angle_pid,YAW_MOTOR_INS_ANGLE_PID_MAX_OUT,YAW_MOTOR_INS_ANGLE_PID_MAX_IOUT);
	PID_init(&(yaw->auto_aim_pid),PID_POSITION,yaw_motor_auto_aim_pid,YAW_MOTOR_AUTO_AIM_PID_MAX_OUT,YAW_MOTOR_AUTO_AIM_PID_MAX_IOUT);
	
	PID_init(&pitch->ENC_speed_pid,PID_POSITION,pitch_motor_enc_speed_pid,PITCH_MOTOR_ENC_SPEED_PID_MAX_OUT,PITCH_MOTOR_ENC_SPEED_PID_MAX_IOUT);
	PID_init(&pitch->ENC_angle_pid,PID_POSITION,pitch_motor_enc_angle_pid,PITCH_MOTOR_ENC_ANGLE_PID_MAX_OUT,PITCH_MOTOR_ENC_ANGLE_PID_MAX_IOUT);	
	PID_init(&pitch->INS_speed_pid,PID_POSITION,pitch_motor_ins_speed_pid,PITCH_MOTOR_INS_SPEED_PID_MAX_OUT,PITCH_MOTOR_INS_SPEED_PID_MAX_IOUT);
	PID_init(&pitch->INS_angle_pid,PID_POSITION,pitch_motor_ins_angle_pid,PITCH_MOTOR_INS_ANGLE_PID_MAX_OUT,PITCH_MOTOR_INS_ANGLE_PID_MAX_IOUT);	
	PID_init(&pitch->auto_aim_pid,PID_POSITION,pitch_motor_auto_aim_pid,PITCH_MOTOR_AUTO_AIM_PID_MAX_OUT,PITCH_MOTOR_AUTO_AIM_PID_MAX_IOUT);	

}

void Gimbal_Motor_Data_Update(void)
{

	//Pitch
	pitch->ENC_last_ecd = pitch->ENC_ecd;
	pitch->ENC_ecd = motor_measure_gimbal[1].ecd;
	pitch->ENC_angle = (pitch->ENC_ecd / 8192.0f * 360.0f);
	pitch->origin = PITCH_MOTOR_INIT_POS / 8192.0f * 360.0f;
	pitch->ENC_relative_angle = pitch->ENC_angle - pitch->origin;
	pitch->ENC_speed = motor_measure_gimbal[1].speed_rpm/60.0f * 360.0f;

	
	// CHECK THE DIRECTION (ROLL/PITCH) IN DEBUG MODE
	pitch->INS_speed = bmi088_real_data.gyro[0]*180.0f / 3.14159f;
	pitch->INS_angle = INS_angle_deg[2]; 
	

	//Yaw
	yaw->ENC_last_ecd = yaw->ENC_ecd;
	yaw->ENC_ecd = motor_measure_gimbal[0].ecd;
	yaw->ENC_speed = motor_measure_gimbal[0].speed_rpm / 60.0f * 360.0f;
	//ENC angle
	if (yaw->ENC_last_ecd < 100 && yaw->ENC_ecd > 8091)
		yaw->ENC_round_cnt--;
	if (yaw->ENC_last_ecd > 8091 && yaw->ENC_ecd < 100)
		yaw->ENC_round_cnt++;
	yaw->ENC_angle = ( yaw->ENC_ecd / 8192.0f * 360.0f + yaw->ENC_round_cnt * 360.0f );
	yaw->origin = YAW_MOTOR_INIT_POS / 8192.0f * 360.0f + ( yaw->ENC_round_cnt * 360.0f );
	yaw->ENC_relative_angle = yaw->ENC_angle - yaw->origin;
	if (yaw->ENC_relative_angle < -180.0f )
		yaw->ENC_relative_angle = 360.0f + yaw->ENC_relative_angle;
	if (yaw->ENC_relative_angle > 180.0f )
		yaw->ENC_relative_angle = yaw->ENC_relative_angle - 360.0f;


	//INS angle
	yaw->INS_angle = INS_angle_deg[0] - yaw->INS_angle_offset; // -180 < angle_deg < 180
	if ( yaw->INS_angle > 180.0f)
		yaw->INS_angle = yaw->INS_angle - 360.0f;
	if ( yaw->INS_angle < -180.0f)
		yaw->INS_angle = yaw->INS_angle + 360.0f;
			
	// cos(pitch)*gyro[yaw]-sin(pitch)*gyro[(index corresponding to ins_speed)]
	yaw->INS_speed = arm_cos_f32(pitch->ENC_relative_angle * 3.14159f /180.0f) * bmi088_real_data.gyro[2] - arm_sin_f32(pitch->ENC_relative_angle * 3.14159f /180.0f) * bmi088_real_data.gyro[0];
	yaw->INS_speed = yaw->INS_speed / 3.1415f * 180.0f ;
}

void Gimbal_RC_Mode_Set(void)
{

	gimbal_control.last_mode = gimbal_control.mode;
	switch (rc_ctrl.rc.s[0]) // right switch
	{
	case RC_SW_DOWN:
		gimbal_control.mode = Gimbal_No_Force;
		break;
	case RC_SW_UP:
		gimbal_control.mode = Gimbal_Auto_Aiming;
		break;
	case RC_SW_MID:
		gimbal_control.mode = Gimbal_Normal_Mode;
		break;
	default:
		break;
	}
	
}


void Gimbal_Pos_Init(void){
	float init_range = 2.0f;//the initial bearing angle

	yaw->motor_mode = Gimbal_Motor_Encoder;
	pitch->motor_mode = Gimbal_Motor_Encoder;

	// Yaw
	while( fabs_2(yaw->ENC_relative_angle) > init_range ){
		Gimbal_Motor_Data_Update();

		PID_calc(&yaw->ENC_angle_pid, yaw->ENC_relative_angle, 0.0f );
		yaw->INS_speed_set = yaw->ENC_angle_pid.out;
		PID_calc( &yaw->INS_speed_pid, yaw->INS_speed, yaw->INS_speed_set );
		yaw->give_current = yaw->INS_speed_pid.out;

		CAN_YAW_CMD(1,yaw->give_current);
		vTaskDelay(1);
	}
	yaw->INS_angle_offset = INS_angle_deg[0];

	// Pitch		
	while( fabs_2(pitch->ENC_relative_angle) > init_range ){
		Gimbal_Motor_Data_Update();
		
		PID_calc(&pitch->ENC_angle_pid, pitch->ENC_relative_angle, 0.0f );
		pitch->INS_speed_set = pitch->ENC_angle_pid.out;
		PID_calc( &pitch->INS_speed_pid, pitch->INS_speed, pitch->INS_speed_set );
		pitch->give_current = pitch->INS_speed_pid.out;

		CAN_PITCH_CMD(2, pitch->give_current);
		vTaskDelay(1);
	}

	yaw->motor_mode = yaw->last_motor_mode;
	pitch->motor_mode = pitch->last_motor_mode;
}

void Yaw_Motor_Control(void)
{
	fp32 err;
	if (gimbal_control.mode == Gimbal_No_Force)
	{
		yaw->give_current = 0.0f;
		return;
	}

	if( gimbal_control.mode == Gimbal_Normal_Mode )
		if( rc_ctrl.rc.ch[2] > 10 || rc_ctrl.rc.ch[2] < -10 )
			yaw->angle_set = yaw->angle_set - (float)rc_ctrl.rc.ch[2]/6600.0f;
	if( gimbal_control.mode == Gimbal_Auto_Aiming )
		yaw->angle_set = vision_rx.yaw_angle;
	yaw->angle_set = (yaw->angle_set > 180.0f) ? 
		(yaw->angle_set - 360.0f) : yaw->angle_set;

	yaw->angle_set = (yaw->angle_set < -180.0f) ? 
		(yaw->angle_set + 360.0f) : yaw->angle_set;

	err = yaw->INS_angle - yaw->angle_set;
	err = (err > 180.0f) ? (err - 360.0f) : err;
	err = (err < -180.0f) ? (err + 360.0f) : err;


	PID_calc( &yaw->INS_angle_pid, err, 0.0f );
	yaw->INS_speed_set = yaw->INS_angle_pid.out;
	PID_calc( &yaw->INS_speed_pid, yaw->INS_speed, yaw->INS_speed_set );
	yaw->give_current = yaw->INS_speed_pid.out;
}

void Pitch_Motor_Control(void)
{
	if (gimbal_control.mode == Gimbal_No_Force)
	{
		pitch->give_current = 0.0f;
		return;
	}

	if( gimbal_control.mode == Gimbal_Normal_Mode )
	{
		if( rc_ctrl.rc.ch[3] > 10 || rc_ctrl.rc.ch[3] < -10 )
			pitch->angle_set = pitch->angle_set + (float)rc_ctrl.rc.ch[3]/6600.0f;
	}
	else if( gimbal_control.mode == Gimbal_Auto_Aiming )
	{
		pitch->angle_set = vision_rx.pitch_angle;
	}
	if (pitch->angle_set<PITCH_DOWN_BOUND)
		pitch->angle_set = PITCH_DOWN_BOUND;
	if (pitch->angle_set>PITCH_UP_BOUND)
		pitch->angle_set = PITCH_UP_BOUND;
		
	PID_calc(&pitch->INS_angle_pid, pitch->INS_angle, pitch->angle_set );
	pitch->INS_speed_set = pitch->INS_angle_pid.out;
	PID_calc( &pitch->INS_speed_pid, pitch->INS_speed, pitch->INS_speed_set );
	pitch->give_current = pitch->INS_speed_pid.out;
}

void Gimbal_Motor_Mode_Set(void)
{
	yaw->last_motor_mode = yaw->motor_mode;
	pitch->last_motor_mode = pitch->motor_mode;

	yaw->motor_mode = Gimbal_Motor_INS;
	pitch->motor_mode = Gimbal_Motor_INS;
}

void Gimbal_Mode_Change_Control_Transit(void)
{
	if( gimbal_control.last_mode == Gimbal_No_Force && gimbal_control.mode == Gimbal_Normal_Mode )
	{
		gimbal_control.init_flag = 1;
		Gimbal_Pos_Init();
		vTaskDelay(1);
		yaw->angle_set=0;
		pitch->angle_set=0;
		gimbal_control.init_flag = 0;
	}
	else if ( gimbal_control.last_mode == Gimbal_Normal_Mode && gimbal_control.mode == Gimbal_Auto_Aiming )
	{

	}
	else if ( gimbal_control.last_mode == Gimbal_Auto_Aiming && gimbal_control.mode == Gimbal_Normal_Mode )
	{
		// TODO: test the chassis follow function
	}
}

void Gimbal_Task(void const * argument)
{

	Gimbal_Motor_Init();
	vTaskDelay(200);

	while(1)
	{
		Gimbal_Motor_Data_Update();
		Gimbal_RC_Mode_Set();
		Gimbal_Motor_Mode_Set();
		Gimbal_Mode_Change_Control_Transit();
		
		Yaw_Motor_Control();
		Pitch_Motor_Control();
		
		CAN_YAW_CMD(1,yaw->give_current);
		CAN_PITCH_CMD(2,pitch->give_current);
		
		vTaskDelay(1);
	}
}
