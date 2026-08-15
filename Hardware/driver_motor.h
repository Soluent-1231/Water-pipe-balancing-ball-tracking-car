#ifndef __DRIVER_MOTOR_H__
#define __DRIVER_MOTOR_H__

#include <stdint.h>

void set_left_motor(int16_t speed);
void set_right_motor(int16_t speed);
void set_motor_stop(void);
void Soft(void);

#endif