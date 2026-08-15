#include "driver_servo.h"


void Servo_Init(void)
{
    Servo_SetAngle(120);
}

// 40°到280°
void Servo_SetAngle(float angle)
{
		float Width=500.0f+ (angle* (2500.0f-500.0f)/270.0f);
		DL_TimerA_setCaptureCompareValue (PWM_1_INST, (uint16_t) (Width+ 0.5f), GPIO_PWM_1_C0_IDX);
}