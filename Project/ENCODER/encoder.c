#include "encoder.h"

uint8_t Enc_Rx[4][8] = {0};
int8_t Enc_Init_count = 2;
Door_Enc door;
Elev_Enc elev;

void Set_ZeroPoint(uint8_t ID)
{
	uint8_t ENC_TX[8] = {0};
	ENC_TX[0] = 0x04;
	ENC_TX[1] = ID;
	ENC_TX[2] = 0x06;
	ENC_TX[3] = 0x00;
	if(ID <= 2)
	{FDCAN_SendData(&hfdcan3, ENC_TX, ID, 4);}
	else
	{FDCAN_SendData(&hfdcan2, ENC_TX, ID, 4);}
}

void Enc_Init(void)
{
	elev.Lenc_total = 0;
	elev.Renc_total = 0;
	elev.Lenc_base = 0;
	elev.Renc_base = 0;
	while(Enc_Init_count)
	{
		Set_ZeroPoint(0x01);
		HAL_Delay(200);
		Set_ZeroPoint(0x02);
		HAL_Delay(200);
		Set_ZeroPoint(0x03);
		HAL_Delay(200);
		Set_ZeroPoint(0x04);
		HAL_Delay(200);
		Enc_Init_count--;
	}		
}

void Enc_Get_Feedback(uint32_t identifier, uint8_t *rxdata)
{
	memcpy(Enc_Rx[identifier - 1],rxdata,7);
}

int Get_Enc_Value(uint8_t ID)
{
	int value = 0;
	value = (Enc_Rx[ID - 1][6] << 24) + (Enc_Rx[ID - 1][5] << 16) + (Enc_Rx[ID - 1][4] << 8) + (Enc_Rx[ID - 1][3]);
	return value;
}

void Elev_Enc_Calulate(void)
{
	elev.Lenc_now = Get_Enc_Value(3);
	elev.Renc_now = Get_Enc_Value(4);

	if (elev.Lenc_now - elev.Lenc_pre > 102400)
		{elev.Lenc_base -= 204800;}
	else if (elev.Lenc_now - elev.Lenc_pre < -102400)
		{elev.Lenc_base += 204800;}
	else
		{elev.Lenc_base = elev.Lenc_base;}
	elev.Lenc_pre = elev.Lenc_now;
	elev.Lenc_total = elev.Lenc_now + elev.Lenc_base; 

	if (elev.Renc_now - elev.Renc_pre > 102400)
		{elev.Renc_base -= 204800;}
	else if (elev.Renc_now - elev.Renc_pre < -102400)
		{elev.Renc_base += 204800;}
	else
		{elev.Renc_base = elev.Renc_base;}
	elev.Renc_pre = elev.Renc_now;
	elev.Renc_total = elev.Renc_now + elev.Renc_base; 
}

void Door_Enc_Calulate(void)
{
	door.Lenc_now = Get_Enc_Value(1);
	door.Renc_now = Get_Enc_Value(2);
	
	if(door.Lenc_now - door.Lenc_pre > 65536)
		{door.Lenc_base -= 131072;}
	else if(door.Lenc_now - door.Lenc_pre <-65536)
		{door.Lenc_base += 131072;}
	else
		{door.Lenc_base = door.Lenc_base;}
	door.Lenc_pre = door.Lenc_now;
	door.Lenc_total = door.Lenc_now + door.Lenc_base;
		
	if(door.Renc_now - door.Renc_pre > 65536)
		{door.Renc_base -= 131072;}
	else if(door.Renc_now - door.Renc_pre <-65536)
		{door.Renc_base += 131072;}
	else
		{door.Renc_base = door.Renc_base;}
	door.Renc_pre = door.Renc_now;
	door.Renc_total = door.Renc_now + door.Renc_base;
}

