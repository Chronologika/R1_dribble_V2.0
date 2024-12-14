#include "elev_control.h"

int8_t elev_lift_flag = 0, dribble_begin_flag = 0, lift_cmplt_count = 0, flag_set_count = 0;
int16_t LIFT_GPIO_SET_COUNT = 0, DESC_GPIO_SET_COUNT = 0;
int lift_cmplt_time;
ELEV_STATE ELEV_ACT = ELEV_NOTHING;

void Elev_State_Set(void)
{
	if (elev_lift_flag == 1)
	{
		ELEV_ACT = ELEV_LIFTING;
	}
	else if (elev_lift_flag == -1)
	{
		ELEV_ACT = ELEV_DESCENDING;
	}
	else if (elev_lift_flag == 0)
	{
		ELEV_ACT = ELEV_NOTHING;
	}
}

void Set_Elev_Pos(void)
{
	switch (ELEV_ACT)
	{
	case ELEV_LIFTING:
		RM2006_Set_Pos(elev_target1, 3, &hfdcan2);
		RM2006_Set_Pos(elev_target2, 4, &hfdcan2);
		if(LIFT_GPIO_SET_COUNT)
		{
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET); //º–«Ú
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET); //Õ£÷πÕ∆«Ú	
		LIFT_GPIO_SET_COUNT--;
		}
		break;

	case ELEV_DESCENDING:
		RM2006_Set_Pos(0, 3, &hfdcan2);
		RM2006_Set_Pos(0, 4, &hfdcan2);
		if(DESC_GPIO_SET_COUNT)
		{
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET); //À…«Ú		
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET); //Õ£÷πÕ∆«Ú	
		DESC_GPIO_SET_COUNT--;
		}
		break;

	case ELEV_NOTHING:
		RM2006_Set_I(0,3,&hfdcan2);
		RM2006_Set_I(0,4,&hfdcan2);
		break;
	}
}

void Elev_State_Send(void)
{
	if (ABS(elev.Lenc_total - elev_target1) <= elev_deadband && ABS(elev.Renc_total - elev_target2) <= elev_deadband) //ºÏ≤‚…œ…˝
	{
		Get_Lift_cmplt_time();
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET); //À…«Ú
		if(real_time >= lift_cmplt_time + 1000)
		{
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET); //Õ∆«Ú
		}
		if(ABS(door.Lenc_total) <= door_deadband && ABS(door.Renc_total) <= door_deadband) //ºÏ≤‚πÿ√≈
		{
//		uint8_t txdata[1] = {1};
//		FDCAN_SendData(&hfdcan1, txdata, 0x602, 1);//‘À«ÚÕÍ≥…–≈œ¢
		}
	}
	else if (ABS(elev.Lenc_total - 0) <= elev_deadband && ABS(elev.Renc_total - 0) <= elev_deadband) //ºÏ≤‚œ¬Ωµ
	{
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET); //À…«Ú
		if(ABS(door_target1 - door.Lenc_total) <= door_deadband && ABS(door_target2 - door.Renc_total) <= door_deadband) //ºÏ≤‚ø™√≈
		{
//		uint8_t txdata[1] = {1};
//		FDCAN_SendData(&hfdcan3, txdata, 0x125, 1);//‘À«Ú∏¥Œª–≈œ¢
		}
	}
}

void Get_Lift_cmplt_time(void)
{
	while (lift_cmplt_count)
	{
		lift_cmplt_time = HAL_GetTick();
		lift_cmplt_count--;
	}
}

void dribble_set(void)
{
	if (dribble_begin_flag == 1)
	{
		while(flag_set_count)
		{
			elev_lift_flag = 1;
			closed_flag = 1;
			lift_cmplt_count = 1;
			close_begin_count = 1;
			DESC_GPIO_SET_COUNT = 500;
			LIFT_GPIO_SET_COUNT = 500;
			falling_count = 0;
			flag_set_count--;
		}
	}
	if (dribble_begin_flag == 0)
	{
		elev_lift_flag = 0;
		closed_flag = 0;
	}
	if (dribble_begin_flag == -1)
	{
		elev_lift_flag = -1;
		closed_flag = -1;
	}
}

void elev_status_judge(void)
{
	if (elev.Lenc_total >= elev_middle && falling_count <= 1)
	{
		uint8_t txdata[1] = {1};
		FDCAN_SendData(&hfdcan1, txdata, 0x600, 1);
	}
	else if (elev.Lenc_total < elev_middle && falling_count <= 1)
	{
		uint8_t txdata[1] = {1};
		FDCAN_SendData(&hfdcan1, txdata, 0x601, 1);
	}
	else if (elev.Lenc_total >= elev_middle && falling_count >= 2)
	{
		uint8_t txdata[1] = {1};
		FDCAN_SendData(&hfdcan1, txdata, 0x602, 1);
	}
	else if (elev.Lenc_total < elev_middle && falling_count >= 2)
	{
		uint8_t txdata[1] = {1};
		FDCAN_SendData(&hfdcan1, txdata, 0x603, 1);
	}
}

