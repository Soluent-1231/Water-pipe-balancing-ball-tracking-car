#ifndef __RAMP_H__
#define __RAMP_H__

float Ramp(float current,float target,float step);

/* S型斜坡：起步/停车速度平滑过渡，无速度阶跃 */
void  RampS_Reset(float velocity);
float RampS(float target, float max_accel, float max_jerk);
float RampS_GetAccel(void);

#endif
