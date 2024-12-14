#include "rm2006.h"
#include "fdcan_bsp.h"
#include "encoder.h"
#include "string.h"
#include "math.h"

#define CLAMP(x, lower, upper) (x >= upper ? upper : (x <= lower ? lower : x))
#define LIMIT(x, limit) (x >= limit ? limit : (x <= -limit ? -limit : x))
#define DEADBAND(x, limit) (x >= limit ? x : (x <= -limit ? x : 0))
#define RM2006_ABS(x) (x >= 0 ? x : -x)

#define SEND_ID_1 0x200
#define SEND_ID_2 0x1ff

const uint8_t RM2006_Reduction_Ratio[8] = {36, 36, 36, 36, 36, 36, 36, 36}; // 电机减速比数组
uint8_t RM2006_Sendbuff[8] = {0};											// 发送数组
uint8_t RM2006_Feedback_Buff[8][7];											// 电机反馈值(全局变量)


M2006_PID M2006_Speed_Pid[8] =
	{
		{.Kp = 8, .Ki = 0.1, .Kd = 7, .Max = 16000, .Min = -16000, .IntegralLimit = 3800, .DeadBand = 50},	// ID = 1
		{.Kp = 8, .Ki = 0.1, .Kd = 7, .Max = 16000, .Min = -16000, .IntegralLimit = 3800, .DeadBand = 50},	// ID = 2
		{.Kp = 10, .Ki = 0.1, .Kd = 7, .Max = 16000, .Min = -16000, .IntegralLimit = 3800, .DeadBand = 5},	// ID = 3
		{.Kp = 10, .Ki = 0.1, .Kd = 7, .Max = 16000, .Min = -16000, .IntegralLimit = 3800, .DeadBand = 5},	// ID = 4
		{.Kp = 8, .Ki = 0.0f, .Kd = 10, .Max = 16000, .Min = -16000, .IntegralLimit = 1000, .DeadBand = 0}, // ID = 5
		{.Kp = 8, .Ki = 0.0f, .Kd = 10, .Max = 16000, .Min = -16000, .IntegralLimit = 1000, .DeadBand = 0}, // ID = 6
		{.Kp = 8, .Ki = 0.0f, .Kd = 10, .Max = 16000, .Min = -16000, .IntegralLimit = 1000, .DeadBand = 0}, // ID = 7
		{.Kp = 8, .Ki = 0.0f, .Kd = 10, .Max = 16000, .Min = -16000, .IntegralLimit = 1000, .DeadBand = 0}, // ID = 8
};

M2006_PID M2006_Pos_Pid[8] =
	{
		{.Kp = 6, .Ki = 0, .Kd = 0.15, .Max = 9600, .Min = -9600, .IntegralLimit = 1000, .I_Start_Err = 50, .DeadBand = 100}, // ID = 1
		{.Kp = 6, .Ki = 0, .Kd = 0.15, .Max = 9600, .Min = -9600, .IntegralLimit = 1000, .I_Start_Err = 50, .DeadBand = 100}, // ID = 2
		{.Kp = 14, .Ki = 0, .Kd = 0.15, .Max = 9600, .Min = -9600, .IntegralLimit = 1000, .I_Start_Err = 50, .DeadBand = 30}, // ID = 3
		{.Kp = 12, .Ki = 0, .Kd = 0.15, .Max = 9600, .Min = -9600, .IntegralLimit = 1000, .I_Start_Err = 50, .DeadBand = 30}, // ID = 4
		{.Kp = 0.3, .Ki = 0, .Kd = 0, .Max = 9600, .Min = -9600, .IntegralLimit = 1000, .I_Start_Err = 10000, .DeadBand = 0},	 // ID = 5
		{.Kp = 0.3, .Ki = 0, .Kd = 0, .Max = 9600, .Min = -9600, .IntegralLimit = 1000, .I_Start_Err = 10000, .DeadBand = 0},	 // ID = 6
		{.Kp = 0.3, .Ki = 0, .Kd = 0, .Max = 9600, .Min = -9600, .IntegralLimit = 1000, .I_Start_Err = 10000, .DeadBand = 0},	 // ID = 7
		{.Kp = 0.3, .Ki = 0, .Kd = 0, .Max = 9600, .Min = -9600, .IntegralLimit = 1000, .I_Start_Err = 10000, .DeadBand = 0},	 // ID = 8

};

void RM2006_Get_Feedback(uint32_t std_id, uint8_t *data_p)
{
	int i = std_id - 0x200;
	memcpy(RM2006_Feedback_Buff[i - 1], data_p, 6);
	return;
}

int RM2006_Get_Speed(uint8_t motor_id)
{
	int speed = 0;
	if (RM2006_Feedback_Buff[motor_id - 1][2] >> 7 == 1) // 转速正负判断
		speed = -(0xffff-((RM2006_Feedback_Buff[motor_id-1][2]<<8)+RM2006_Feedback_Buff[motor_id-1][3]));
	else
		speed = (RM2006_Feedback_Buff[motor_id - 1][2] << 8) + RM2006_Feedback_Buff[motor_id - 1][3];
	return speed;
}

void RM2006_Set_I(int target_i, uint8_t motor_id, FDCAN_HandleTypeDef *hfdcan)
{
	target_i = LIMIT(target_i,10000);
	if (motor_id <= 4)
	{
		RM2006_Sendbuff[2 * motor_id - 2] = target_i >> 8;	   // 电流值高8位
		RM2006_Sendbuff[2 * motor_id - 1] = target_i & 0x00ff; // 电流值低8位
		RM2006_SendData(hfdcan);
	}
}

void RM2006_Set_Speed(int goal_speed, int ID, FDCAN_HandleTypeDef *hfdcan)
{
	uint8_t id = ID - 1;
	M2006_Speed_Pid[id].Target = goal_speed;
	M2006_Speed_Pid[id].NowIS = RM2006_Get_Speed(ID);
	M2006_Speed_Pid[id].Err_last = M2006_Speed_Pid[id].Err;
	M2006_Speed_Pid[id].Err = M2006_Speed_Pid[id].Target - M2006_Speed_Pid[id].NowIS;

	if (RM2006_ABS(M2006_Speed_Pid[id].Err) < M2006_Speed_Pid[id].DeadBand)
	{
		M2006_Speed_Pid[id].Err = 0;
	}

	M2006_Speed_Pid[id].Err_sum += M2006_Speed_Pid[id].Err;
	M2006_Speed_Pid[id].Err_sum = CLAMP(M2006_Speed_Pid[id].Err_sum, -M2006_Speed_Pid[id].IntegralLimit, M2006_Speed_Pid[id].IntegralLimit);

	M2006_Speed_Pid[id].P_Out = M2006_Speed_Pid[id].Kp * M2006_Speed_Pid[id].Err;
	M2006_Speed_Pid[id].I_Out = M2006_Speed_Pid[id].Ki * M2006_Speed_Pid[id].Err_sum;
	M2006_Speed_Pid[id].D_Out = M2006_Speed_Pid[id].Kd * (M2006_Speed_Pid[id].Err - M2006_Speed_Pid[id].Err_last);

	M2006_Speed_Pid[id].PID_Out = M2006_Speed_Pid[id].P_Out + M2006_Speed_Pid[id].I_Out + M2006_Speed_Pid[id].D_Out;
	M2006_Speed_Pid[id].PID_Out = CLAMP(M2006_Speed_Pid[id].PID_Out, M2006_Speed_Pid[id].Min, M2006_Speed_Pid[id].Max);

	RM2006_Set_I(M2006_Speed_Pid[id].PID_Out, ID, hfdcan);
}

void RM2006_Set_Pos(float pos_goal, int ID, FDCAN_HandleTypeDef *hfdcan)
{
	uint8_t id = ID - 1;
	if (hfdcan == &hfdcan3 && ID == 1)
	{
		M2006_Pos_Pid[id].Target = pos_goal;
		M2006_Pos_Pid[id].NowIS = door.Lenc_total;
		M2006_Pos_Pid[id].Err_last = M2006_Pos_Pid[id].Err;
		M2006_Pos_Pid[id].Err = DEADBAND(pos_goal - M2006_Pos_Pid[id].NowIS, M2006_Pos_Pid[id].DeadBand);

		if (fabsf(M2006_Pos_Pid[id].Err) < M2006_Pos_Pid[id].I_Start_Err) // 当误差小于一定程度时开始使用KI进行调节
			M2006_Pos_Pid[id].Err_sum += M2006_Pos_Pid[id].Err;
		else // 远处不使用KI进行调节
			M2006_Pos_Pid[id].Err_sum = 0;

		M2006_Pos_Pid[id].P_Out = M2006_Pos_Pid[id].Kp * M2006_Pos_Pid[id].Err;
		M2006_Pos_Pid[id].I_Out = LIMIT(M2006_Pos_Pid[id].Ki * M2006_Pos_Pid[id].Err_sum, M2006_Pos_Pid[id].IntegralLimit);
		M2006_Pos_Pid[id].D_Out = M2006_Pos_Pid[id].Kd * (M2006_Pos_Pid[id].Err - M2006_Pos_Pid[id].Err_last);

		M2006_Pos_Pid[id].PID_Out = LIMIT(M2006_Pos_Pid[id].P_Out + M2006_Pos_Pid[id].I_Out + M2006_Pos_Pid[id].D_Out, M2006_Pos_Pid[id].Max);

		RM2006_Set_Speed(M2006_Pos_Pid[id].PID_Out, ID, hfdcan);
	}
	else if (hfdcan == &hfdcan3 && ID == 2)
	{
		M2006_Pos_Pid[id].Target = pos_goal;
		M2006_Pos_Pid[id].NowIS = door.Renc_total;
		M2006_Pos_Pid[id].Err_last = M2006_Pos_Pid[id].Err;
		M2006_Pos_Pid[id].Err = DEADBAND(pos_goal - M2006_Pos_Pid[id].NowIS, M2006_Pos_Pid[id].DeadBand);

		if (fabsf(M2006_Pos_Pid[id].Err) < M2006_Pos_Pid[id].I_Start_Err) // 当误差小于一定程度时开始使用KI进行调节
			M2006_Pos_Pid[id].Err_sum += M2006_Pos_Pid[id].Err;
		else // 远处不使用KI进行调节
			M2006_Pos_Pid[id].Err_sum = 0;

		M2006_Pos_Pid[id].P_Out = M2006_Pos_Pid[id].Kp * M2006_Pos_Pid[id].Err;
		M2006_Pos_Pid[id].I_Out = LIMIT(M2006_Pos_Pid[id].Ki * M2006_Pos_Pid[id].Err_sum, M2006_Pos_Pid[id].IntegralLimit);
		M2006_Pos_Pid[id].D_Out = M2006_Pos_Pid[id].Kd * (M2006_Pos_Pid[id].Err - M2006_Pos_Pid[id].Err_last);

		M2006_Pos_Pid[id].PID_Out = LIMIT(M2006_Pos_Pid[id].P_Out + M2006_Pos_Pid[id].I_Out + M2006_Pos_Pid[id].D_Out, M2006_Pos_Pid[id].Max);

		RM2006_Set_Speed(M2006_Pos_Pid[id].PID_Out, ID, hfdcan);
	}
	else if (hfdcan == &hfdcan2 && ID == 3)
	{
		M2006_Pos_Pid[id].Target = pos_goal;
		M2006_Pos_Pid[id].NowIS = elev.Lenc_total;
		M2006_Pos_Pid[id].Err_last = M2006_Pos_Pid[id].Err;
		M2006_Pos_Pid[id].Err = DEADBAND(pos_goal - M2006_Pos_Pid[id].NowIS, M2006_Pos_Pid[id].DeadBand);

		if (fabsf(M2006_Pos_Pid[id].Err) < M2006_Pos_Pid[id].I_Start_Err) // 当误差小于一定程度时开始使用KI进行调节
			M2006_Pos_Pid[id].Err_sum += M2006_Pos_Pid[id].Err;
		else // 远处不使用KI进行调节
			M2006_Pos_Pid[id].Err_sum = 0;

		M2006_Pos_Pid[id].P_Out = M2006_Pos_Pid[id].Kp * M2006_Pos_Pid[id].Err;
		M2006_Pos_Pid[id].I_Out = LIMIT(M2006_Pos_Pid[id].Ki * M2006_Pos_Pid[id].Err_sum, M2006_Pos_Pid[id].IntegralLimit);
		M2006_Pos_Pid[id].D_Out = M2006_Pos_Pid[id].Kd * (M2006_Pos_Pid[id].Err - M2006_Pos_Pid[id].Err_last);

		M2006_Pos_Pid[id].PID_Out = LIMIT(M2006_Pos_Pid[id].P_Out + M2006_Pos_Pid[id].I_Out + M2006_Pos_Pid[id].D_Out, M2006_Pos_Pid[id].Max);

		RM2006_Set_Speed(M2006_Pos_Pid[id].PID_Out, ID, hfdcan);
	}
	else if (hfdcan == &hfdcan2 && ID == 4)
	{
		M2006_Pos_Pid[id].Target = pos_goal;
		M2006_Pos_Pid[id].NowIS = elev.Renc_total;
		M2006_Pos_Pid[id].Err_last = M2006_Pos_Pid[id].Err;
		M2006_Pos_Pid[id].Err = DEADBAND(pos_goal - M2006_Pos_Pid[id].NowIS, M2006_Pos_Pid[id].DeadBand);

		if (fabsf(M2006_Pos_Pid[id].Err) < M2006_Pos_Pid[id].I_Start_Err) // 当误差小于一定程度时开始使用KI进行调节
			M2006_Pos_Pid[id].Err_sum += M2006_Pos_Pid[id].Err;
		else // 远处不使用KI进行调节
			M2006_Pos_Pid[id].Err_sum = 0;

		M2006_Pos_Pid[id].P_Out = M2006_Pos_Pid[id].Kp * M2006_Pos_Pid[id].Err;
		M2006_Pos_Pid[id].I_Out = LIMIT(M2006_Pos_Pid[id].Ki * M2006_Pos_Pid[id].Err_sum, M2006_Pos_Pid[id].IntegralLimit);
		M2006_Pos_Pid[id].D_Out = M2006_Pos_Pid[id].Kd * (M2006_Pos_Pid[id].Err - M2006_Pos_Pid[id].Err_last);

		M2006_Pos_Pid[id].PID_Out = LIMIT(M2006_Pos_Pid[id].P_Out + M2006_Pos_Pid[id].I_Out + M2006_Pos_Pid[id].D_Out, M2006_Pos_Pid[id].Max);

		RM2006_Set_Speed(M2006_Pos_Pid[id].PID_Out, ID, hfdcan);
	}
}

void RM2006_SendData(FDCAN_HandleTypeDef *hfdcan)
{
		FDCAN_SendData(hfdcan, RM2006_Sendbuff, SEND_ID_1, 8);
}
