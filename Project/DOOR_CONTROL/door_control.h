#ifndef _DOOR_CONTROL_H
#define _DOOR_CONTROL_H

#include "main.h"
#include "rm2006.h"
#include "encoder.h"
#include "fdcan_bsp.h"
#include "elev_control.h"

#define ABS(x) ((x) >= 0 ? (x) : -(x))
#define door_target1 -1024
#define door_target2 1024
#define door_deadband 100

typedef enum
{
	DOOR_NOTHING,
	DOOR_1_ONLY_CLOSE,
	DOOR_1AND2_CLOSE,
	DOOR_1AND2_OPEN,
} DOOR_STATE;

extern uint8_t close_begin_count;
extern int8_t closed_flag;
extern volatile uint8_t falling_count;
extern int real_time,time_door_closed;
extern DOOR_STATE DOOR_ACT;

void Realtime(void);
void Get_Close_begin_time(void);
void Door_Act_Set(void);
void Door_ACT(void);

#endif
