#include "door_control.h"

uint8_t close_begin_count = 0;
int8_t closed_flag = 0;
int real_time, time_closed_begin;
volatile uint8_t falling_count = 0;
DOOR_STATE DOOR_ACT = DOOR_NOTHING;

void Realtime(void)
{
	real_time = HAL_GetTick();
}

void Get_Close_begin_time(void)
{	
	while(close_begin_count)
	{
	time_closed_begin = HAL_GetTick();
	close_begin_count--;
	}
}

void Door_Act_Set(void)
{
	if (closed_flag == 1)
	{
		if (falling_count >= 2)
		{
			Get_Close_begin_time();
			if (real_time <= time_closed_begin + 100)
			{
				DOOR_ACT = DOOR_1_ONLY_CLOSE;
			}
			else if (real_time > time_closed_begin + 100)
			{
				DOOR_ACT = DOOR_1AND2_CLOSE;
			}
		}
		else if (falling_count <= 1)
		{
			DOOR_ACT = DOOR_1AND2_OPEN;
		}
	}
	else if (closed_flag == 0)
	{
		DOOR_ACT = DOOR_NOTHING;
	}
	else if (closed_flag == -1)
	{
		DOOR_ACT = DOOR_1AND2_OPEN;
	}
}

void Door_ACT(void)
{
	switch (DOOR_ACT)
	{
	case DOOR_NOTHING:
		RM2006_Set_I(0,1,&hfdcan3);
		RM2006_Set_I(0,2,&hfdcan3);
		break;

	case DOOR_1_ONLY_CLOSE:
		RM2006_Set_Pos(0, 1, &hfdcan3);
		break;

	case DOOR_1AND2_CLOSE:
		RM2006_Set_Pos(0, 1, &hfdcan3);
		RM2006_Set_Pos(0, 2, &hfdcan3);
		break;

	case DOOR_1AND2_OPEN:
		RM2006_Set_Pos(door_target1, 1, &hfdcan3);
		RM2006_Set_Pos(door_target2, 2, &hfdcan3);
		break;
	}
}

//void Close_Status_Send(void)//合并到运球完成发送中,不需要单独发送
//{
//	if (ABS(door_target1 - door.Lenc_total) <= door_deadband && ABS(door_target2 - door.Renc_total) <= door_deadband) //开门
//	{
//		uint8_t tx_data[1] = {1};
//		FDCAN_SendData(&hfdcan3, tx_data, 0x114, 1);
//	}
//	else if (ABS(door.Lenc_total) <= door_deadband && ABS(door.Renc_total) <= door_deadband)  //关门
//	{
//		uint8_t tx_data[1] = {2};
//		FDCAN_SendData(&hfdcan3, tx_data, 0x514, 1);
//	}
//	else
//	{
//		return;
//	}
//}
