#ifndef __GREY_H__
#define __GREY_H__
#include <stdint.h>
extern uint8_t gray_sensor_data[8];

void Update_All_Gray_Sensors(void);
float Get_Line_Position(void);

#endif
