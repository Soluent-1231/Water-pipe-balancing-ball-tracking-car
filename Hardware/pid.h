#ifndef __PID_H
#define __PID_H
#include "ti_msp_dl_config.h"

typedef struct {
	float Target;
	float Actual;
	float Out;
	
	float Kp;
	float Ki;
	float Kd;
	
	float Error0;
	float Error1;
	float ErrorInt;
	
	float OutMax;
	float OutMin;
} PID_t;


typedef enum
{
    TRACK_NORMAL = 0,
    TRACK_RIGHT_ANGLE_R,
    TRACK_RIGHT_ANGLE_L,
		TRACK_BRAKE,
	  TRACK_STOP,
		TRACK_NO_LINE
}TrackState_t;
extern volatile int16_t left_speed;
extern volatile int16_t right_speed;
extern float position;
void PID_Update(PID_t *p);
void Track_PID_Update(void);
void straight_PID_Update(void);
void Cycle_PID_Update(void);
void Balance_PID_Update_one(void);
void Balance_PID_Update_two(void);
void Balance_PID_Update_three(void);
#endif
