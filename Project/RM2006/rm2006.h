#ifndef _RM2006_H
#define _RM2006_H

#include "stm32g4xx.h"
#include "stm32g4xx_hal.h"
#include "fdcan.h"
    
extern uint8_t RM2006_Feedback_Buff[8][7];	 // 电机反馈值(全局变量)
extern uint8_t RM2006_Sendbuff[8];  //CAN1发送的数据

typedef struct
{
	float Target;
	float NowIS;

	float Err;
	float Err_last;
	float Err_sum;
	float I_Start_Err;

	float Kp;
	float Ki;
	float Kd;

	int Max;
	int Min;

	float P_Out;
	float I_Out;
	float D_Out;
	float PID_Out;

	float DeadBand;
	float IntegralLimit;
} M2006_PID;




//从CAN接收电调传回的反馈信息
void RM2006_Get_Feedback(uint32_t std_id, uint8_t *data_p);
//获取电机速度信息
int RM2006_Get_Speed(uint8_t motor_id);
//电流环
void RM2006_Set_I(int target_i, uint8_t motor_id,FDCAN_HandleTypeDef *hfdcan);
//通过速度PID设置速度
void RM2006_Set_Speed(int goal_speed,int ID,FDCAN_HandleTypeDef *hfdcan);
//通过位置PID设置位置
void RM2006_Set_Pos(float pos,int ID,FDCAN_HandleTypeDef *hfdcan);
//RM2006专用发送函数
void RM2006_SendData(FDCAN_HandleTypeDef *hfdcan);

#endif
