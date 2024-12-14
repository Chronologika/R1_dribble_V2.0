#ifndef _ENCDOER_H
#define _ENCDOER_H

#include "fdcan_bsp.h"
#include "string.h"

typedef struct
{
    float Lenc_now;
    float Lenc_pre;
    float Lenc_total;
    float Lenc_base;
    float Renc_now;
    float Renc_pre;
    float Renc_total;
    float Renc_base;
} Elev_Enc;

typedef struct
{
    float Lenc_now;
	  float Lenc_pre;
    float Lenc_total;
    float Lenc_base;
    float Renc_now;
		float Renc_pre;
    float Renc_total;
    float Renc_base;
}Door_Enc;

extern Elev_Enc elev;
extern Door_Enc door;
extern uint8_t Enc_Rx[4][8];

void Set_ZeroPoint(uint8_t ID);
void Enc_Init(void);
void Enc_Get_Feedback(uint32_t identifier, uint8_t *rxdata);
int Get_Enc_Value(uint8_t ID);
void Elev_Enc_Calulate(void);
void Door_Enc_Calulate(void);

#endif
