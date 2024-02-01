#ifndef BSP_CAN_H
#define BSP_CAN_H
#include "struct_typedef.h"

#define CAN_GIMBAL_ALL_ID 0x1FF
#define CAN_SHOOT_ALL_ID 0x200
#define CAN_CHASSIS_ALL_ID 0x200
#define GIMBAL_YAW_CAN hcan2
#define GIMBAL_PITCH_CAN hcan1
#define SHOOT_CAN hcan1
#define CHASSIS_CAN hcan2
#define M9025_CAN hcan1
#define CAP_CAN hcan2

#define CAN_6020_M1_ID 0x205
#define CAN_6020_M2_ID 0x206


#define CAN_FRIC_LEFT_MOTOR_ID 0x201
#define CAN_FRIC_RIGHT_MOTOR_ID 0x202
#define CAN_2006_M1_ID 0x203

//          Front
//      | 4       3 |
//  Left|           |Right
//      | 1       2 |
#define CAN_3508_M1_ID 0x201
#define CAN_3508_M2_ID 0x202
#define CAN_3508_M3_ID 0x203
#define CAN_3508_M4_ID 0x204


#define P_MIN -95.5f    // Radians
#define P_MAX 95.5f
#define V_MIN -45.0f    // Rad/s
#define V_MAX 45.0f
#define KP_MIN 0.0f     // N-m/rad
#define KP_MAX 500.0f
#define KD_MIN 0.0f     // N-m/rad/s
#define KD_MAX 5.0f
#define T_MIN -18.0f
#define T_MAX 18.0f

typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;

typedef struct 
{
    fp32 InputVot;
    fp32 CapVot;
    uint8_t statement;
    fp32 InputCur;
    fp32 OutputCur;
    uint8_t cap_temperature;
    uint8_t cap_percent;
    uint8_t Target_Power;
    uint8_t power_set;
} cap_measure_t;

extern void can_filter_init(void);

void CAN_YAW_CMD(uint8_t id, int16_t current);
void CAN_PITCH_CMD(uint8_t id, int16_t current);
void CAN_Chassis_CMD(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void CAN_Shoot_CMD( int16_t motor1, int16_t motor2, int16_t motor3 );
void CAN_9025_CMD(uint8_t id, int16_t current);
void CAN_CAP_CMD(float power);

extern motor_measure_t motor_measure_chassis[4];
extern motor_measure_t motor_measure_gimbal[2];
extern motor_measure_t motor_measure_shoot[3];
extern cap_measure_t cap_measure;


#endif
