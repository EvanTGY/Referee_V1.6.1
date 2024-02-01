#include "bsp_can.h"
#include "main.h"
#include "Chassis_Task.h"
#include "Gimbal_Task.h"
#include "detect_task.h"
#include "Shoot_Task.h"

#define get_DJI_motor_measure(ptr, data)                           	\
{                                                                   \
	(ptr)->last_ecd = (ptr)->ecd;                                   \
	(ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
	(ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
	(ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
	(ptr)->temperate = (data)[6];                                   \
}


#define get_HT_motor_measure(ptr, data)																			\
{ 																												\
	(ptr)->last_ecd = (ptr)->ecd;																				\
	(ptr)->ecd =uint_to_float((uint16_t)((data)[1] << 8 | (data)[2] ),P_MIN, P_MAX, 16);						\
	(ptr)->speed_rpm = uint_to_float((uint16_t)(data[3]<<4)|(data[4]>>4), V_MIN, V_MAX, 12);					\
	(ptr)->real_torque = uint_to_float((uint16_t)(((data[4] & 0x0f) << 8) | (data)[5]), -18, +18, 12) * 0.7; 	\
}

#define get_9025_motor_measure(ptr, data)                  			\
{                                  									\
	(ptr)->last_ecd = (ptr)->ecd;                  					\
	(ptr)->ecd = (uint16_t)((data)[7] << 8 | (data)[6]);      		\
	(ptr)->speed_rpm = (int16_t)((data)[5] << 8 | (data)[4]);  		\
	(ptr)->given_current = (int16_t)((data)[3] << 8 | (data)[2]);	\
	(ptr)->temperate = (data)[1];                  					\
}

#define get_cap_measure(ptr, data)       	\
{                              				\
	(ptr)->statement = data[0];      		\
	(ptr)->cap_percent = data[1];     		\
	(ptr)->InputVot = (fp32)data[2]/8;   	\
	(ptr)->CapVot = (fp32)data[3]/8;   		\
	(ptr)->InputCur = (fp32)data[4]/16;		\
	(ptr)->OutputCur = (fp32)data[5]/16; 	\
	(ptr)->power_set = data[6];   			\
	(ptr)->cap_temperature = data[7];		\
}

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void can_filter_init(void)
{
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);


    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

uint8_t rx_data[8];
motor_measure_t motor_measure_chassis[4];
motor_measure_t motor_measure_gimbal[2];
motor_measure_t motor_measure_shoot[3];
cap_measure_t cap_measure;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef rx_header;

	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

	if (hcan == &hcan1){
		switch (rx_header.StdId){
			case CAN_FRIC_LEFT_MOTOR_ID:
			case CAN_FRIC_RIGHT_MOTOR_ID:{
				static uint8_t i = 0;
				i = rx_header.StdId - CAN_FRIC_LEFT_MOTOR_ID;
				get_DJI_motor_measure(&motor_measure_shoot[i], rx_data);
				detect_hook(FRIC1_TOE + i);
				break;
			}
			case CAN_2006_M1_ID:{
				get_DJI_motor_measure(&motor_measure_shoot[2], rx_data);
				detect_hook(TRIGGER_MOTOR_TOE);
				break;
			}
			case CAN_6020_M2_ID:{
				get_DJI_motor_measure(&motor_measure_gimbal[1], rx_data);
				detect_hook(PITCH_GIMBAL_MOTOR_TOE);
				break;
			}
			default:{
				break;
			}
		}
	}
	else if (hcan == &hcan2){
		switch (rx_header.StdId){
			case CAN_6020_M1_ID:{
				get_DJI_motor_measure(&motor_measure_gimbal[0], rx_data);
				detect_hook(YAW_GIMBAL_MOTOR_TOE);
				break;
			}
			case CAN_3508_M1_ID:
			case CAN_3508_M2_ID:
			case CAN_3508_M3_ID:
			case CAN_3508_M4_ID: {
				static uint8_t i = 0;
				i = rx_header.StdId - CAN_3508_M1_ID;
				get_DJI_motor_measure(&motor_measure_chassis[i], rx_data);
				detect_hook(CHASSIS_MOTOR1_TOE + i);
				break;
			}
			default:{
				break;
			}
		}
	}
}

void CAN_YAW_CMD(uint8_t id, int16_t current)
{	
	CAN_TxHeaderTypeDef	gimbal_tx_message;
	uint8_t            	gimbal_can_send_data[8];
	uint32_t 			send_mail_box;

	gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
	gimbal_tx_message.IDE = CAN_ID_STD;
	gimbal_tx_message.RTR = CAN_RTR_DATA;
	gimbal_tx_message.DLC = 0x08;

	// gimbal_can_send_data[2*id-2] = current >> 8;
	// gimbal_can_send_data[2*id-1] = current;
	gimbal_can_send_data[0] = current >> 8;
	gimbal_can_send_data[1] = current;

	HAL_CAN_AddTxMessage(&GIMBAL_YAW_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

void CAN_PITCH_CMD(uint8_t id, int16_t current)
{
	CAN_TxHeaderTypeDef gimbal_tx_message;
	uint8_t             gimbal_can_send_data[8];
	uint32_t 			send_mail_box;

	gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
	gimbal_tx_message.IDE = CAN_ID_STD;
	gimbal_tx_message.RTR = CAN_RTR_DATA;
	gimbal_tx_message.DLC = 0x08;

	// gimbal_can_send_data[2*id-2] = current >> 8;
	// gimbal_can_send_data[2*id-1] = current;
	gimbal_can_send_data[2] = current >> 8;
	gimbal_can_send_data[3] = current;

	HAL_CAN_AddTxMessage(&GIMBAL_PITCH_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

void CAN_Chassis_CMD(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
	CAN_TxHeaderTypeDef	chassis_tx_message;
	uint8_t            	chassis_can_send_data[8];
	uint32_t 			send_mail_box;

	chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
	chassis_tx_message.IDE = CAN_ID_STD;
	chassis_tx_message.RTR = CAN_RTR_DATA;
	chassis_tx_message.DLC = 0x08;
	chassis_can_send_data[0] = motor1 >> 8;
	chassis_can_send_data[1] = motor1;
	chassis_can_send_data[2] = motor2 >> 8;
	chassis_can_send_data[3] = motor2;
	chassis_can_send_data[4] = motor3 >> 8;
	chassis_can_send_data[5] = motor3;
	chassis_can_send_data[6] = motor4 >> 8;
	chassis_can_send_data[7] = motor4;

	HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

void CAN_Shoot_CMD( int16_t motor1, int16_t motor2, int16_t motor3 )
{
	CAN_TxHeaderTypeDef	shoot_tx_message;
	uint8_t           	shoot_can_send_data[8];
	uint32_t 			send_mail_box;

	shoot_tx_message.StdId = CAN_SHOOT_ALL_ID;
	shoot_tx_message.IDE = CAN_ID_STD;
	shoot_tx_message.RTR = CAN_RTR_DATA;
	shoot_tx_message.DLC = 0x08;
	shoot_can_send_data[0] = motor1 >> 8;
	shoot_can_send_data[1] = motor1;
	shoot_can_send_data[2] = motor2 >> 8;
	shoot_can_send_data[3] = motor2;
	shoot_can_send_data[4] = motor3 >> 8;
	shoot_can_send_data[5] = motor3;

	HAL_CAN_AddTxMessage(&SHOOT_CAN, &shoot_tx_message, shoot_can_send_data, &send_mail_box);
}

void CAN_9025_CMD(uint8_t id, int16_t current)
{
	CAN_TxHeaderTypeDef chassis_tx_message;
	uint8_t             chassis_can_send_data[8];

	uint32_t canTxMailbox= CAN_TX_MAILBOX0;
	chassis_tx_message.StdId = 0x140+id;
	chassis_tx_message.IDE = CAN_ID_STD;
	chassis_tx_message.RTR = CAN_RTR_DATA;
	chassis_tx_message.DLC = 0x08;
	chassis_can_send_data[0] = 0xA1;
	chassis_can_send_data[1] = 0;
	chassis_can_send_data[2] = 0;
	chassis_can_send_data[3] = 0;
	chassis_can_send_data[4] = current;
	chassis_can_send_data[5] = (current>>8);
	chassis_can_send_data[6] = 0;
	chassis_can_send_data[7] = 0;

	while (HAL_CAN_GetTxMailboxesFreeLevel(&M9025_CAN) == 0); 
	if ((M9025_CAN.Instance->TSR & CAN_TSR_TME0) != RESET) 
	{
		canTxMailbox = CAN_TX_MAILBOX0;
	} 
	else if ((M9025_CAN.Instance->TSR & CAN_TSR_TME1) != RESET) 
	{
		canTxMailbox = CAN_TX_MAILBOX1;
	} 
	else if ((M9025_CAN.Instance->TSR & CAN_TSR_TME2) != RESET) 
	{
		canTxMailbox = CAN_TX_MAILBOX2;
	}
	if(HAL_CAN_AddTxMessage(&M9025_CAN, &chassis_tx_message, chassis_can_send_data, (uint32_t *)&canTxMailbox)==HAL_OK){};
}

void CAN_CAP_CMD(float power)
{
	CAN_TxHeaderTypeDef	cap_tx_measure;
	uint8_t				cap_can_send_data[8];
	uint8_t 			powertx;

	powertx = (uint8_t)power;
	uint32_t send_mail_box;
	cap_tx_measure.StdId = 0x120;
	cap_tx_measure.IDE = CAN_ID_STD;
	cap_tx_measure.RTR = CAN_RTR_DATA;
	cap_tx_measure.DLC = 0x08;
	cap_can_send_data[0] = powertx;
	// cap_can_send_data[0] = (power >> 8);
	// cap_can_send_data[1] = power;
	HAL_CAN_AddTxMessage(&CAP_CAN, &cap_tx_measure, cap_can_send_data, &send_mail_box);
}