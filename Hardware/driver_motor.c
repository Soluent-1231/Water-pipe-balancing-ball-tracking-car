#include "ti_msp_dl_config.h"

#define MOTOR_MAX_SPEED   (999) 


void set_left_motor(int16_t speed)
{

    if (speed > MOTOR_MAX_SPEED)  speed = MOTOR_MAX_SPEED;
    if (speed < -MOTOR_MAX_SPEED) speed = -MOTOR_MAX_SPEED;

    if (speed >= 0) 
    {
        DL_GPIO_setPins(GPIO_MOTOR_AIN1_PORT, GPIO_MOTOR_AIN1_PIN);
        DL_GPIO_clearPins(GPIO_MOTOR_AIN2_PORT, GPIO_MOTOR_AIN2_PIN);
        DL_Timer_setCaptureCompareValue(PWM_0_INST, speed, DL_TIMER_CC_0_INDEX);
    } 
    else 
    {
        DL_GPIO_clearPins(GPIO_MOTOR_AIN1_PORT, GPIO_MOTOR_AIN1_PIN);
        DL_GPIO_setPins(GPIO_MOTOR_AIN2_PORT, GPIO_MOTOR_AIN2_PIN);
        DL_Timer_setCaptureCompareValue(PWM_0_INST, -speed, DL_TIMER_CC_0_INDEX);
    }
}


void set_right_motor(int16_t speed)
{
    if (speed > MOTOR_MAX_SPEED)  speed = MOTOR_MAX_SPEED;
    if (speed < -MOTOR_MAX_SPEED) speed = -MOTOR_MAX_SPEED;

    if (speed >= 0) 
    {
        DL_GPIO_setPins(GPIO_MOTOR_BIN1_PORT, GPIO_MOTOR_BIN1_PIN);
        DL_GPIO_clearPins(GPIO_MOTOR_BIN2_PORT, GPIO_MOTOR_BIN2_PIN);
        DL_Timer_setCaptureCompareValue(PWM_0_INST, speed, DL_TIMER_CC_1_INDEX);
    } 
    else 
    {
        DL_GPIO_clearPins(GPIO_MOTOR_BIN1_PORT, GPIO_MOTOR_BIN1_PIN);
        DL_GPIO_setPins(GPIO_MOTOR_BIN2_PORT, GPIO_MOTOR_BIN2_PIN);
        DL_Timer_setCaptureCompareValue(PWM_0_INST, -speed, DL_TIMER_CC_1_INDEX);
    }
}

void set_motor_stop(void)
{
	set_left_motor(0);
	set_right_motor(0);
}

float Base_speed_target = 10;
float Base_speed_now = 0;
void Soft(void)
{
    if(Base_speed_now < Base_speed_target)
    {
        Base_speed_now += 0.25f;

        if(Base_speed_now > Base_speed_target)
        {
            Base_speed_now = Base_speed_target;
        }
    }

    else if(Base_speed_now > Base_speed_target)
    {
        Base_speed_now -= 0.25f;

        if(Base_speed_now < Base_speed_target)
        {
            Base_speed_now = Base_speed_target;
        }
    }
}
