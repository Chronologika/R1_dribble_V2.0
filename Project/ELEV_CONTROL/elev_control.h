#ifndef _ELEV_CONTROL_H
#define _ELEV_CONTROL_H

#include "encoder.h"
#include "rm2006.h"
#include "door_control.h"

#define elev_target1 -202500
#define elev_target2 -198500
#define elev_middle -140000
#define elev_deadband 100

typedef enum
{
	ELEV_NOTHING,
	ELEV_LIFTING,
	ELEV_DESCENDING,
}ELEV_STATE;

void Set_Elev_Pos(void);
void Elev_State_Set(void);
void Elev_State_Send(void);
void Get_Lift_cmplt_time(void);
void dribble_set(void);
void elev_status_judge(void);

extern int8_t elev_lift_flag,dribble_begin_flag,lift_cmplt_count,flag_set_count;
extern int16_t LIFT_GPIO_SET_COUNT,DESC_GPIO_SET_COUNT;
extern int8_t Enc_Init_count;
extern int lift_cmplt_time;
extern ELEV_STATE ELEV_ACT;


#endif

