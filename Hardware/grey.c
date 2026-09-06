#include "ti_msp_dl_config.h"
#include "grey.h"

uint8_t gray_sensor_data[8] = {0};

static uint8_t Read_Gray_Channel(uint8_t channel) 
{

    uint8_t address = channel - 1; 

    if (address & 0x01) {
        DL_GPIO_setPins(GPIO_IR_GREY_PORT_PORT, GPIO_IR_GREY_PORT_AD0_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_IR_GREY_PORT_PORT, GPIO_IR_GREY_PORT_AD0_PIN);
    }
    if (address & 0x02) {
        DL_GPIO_setPins(GPIO_IR_GREY_PORT_PORT, GPIO_IR_GREY_PORT_AD1_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_IR_GREY_PORT_PORT, GPIO_IR_GREY_PORT_AD1_PIN);
    }
    if (address & 0x04) {
        DL_GPIO_setPins(GPIO_IR_GREY_PORT_PORT, GPIO_IR_GREY_PORT_AD2_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_IR_GREY_PORT_PORT, GPIO_IR_GREY_PORT_AD2_PIN);
    }
    delay_cycles(160);
    if (DL_GPIO_readPins(GPIO_IR_GREY_PORT_PORT, GPIO_IR_GREY_PORT_OUT_PIN) & GPIO_IR_GREY_PORT_OUT_PIN) {
        return 1;
    } else {
        return 0;
    }
}

void Update_All_Gray_Sensors(void) 
{
    for (uint8_t i = 1; i <= 8; i++) 
    {
        gray_sensor_data[i - 1] = Read_Gray_Channel(i);
    }
}

float Get_Line_Position(void)
{
    static const float weight[8] =
    {
        -3.5f,
        -2.5f,
        -1.5f,
        -0.5f,
         0.5f,
         1.5f,
         2.5f,
         3.5f
    };

    float sum = 0;
    float pos = 0;
		static float last_pos = 0;
		
    for(int i=0;i<8;i++)
    {
        if(gray_sensor_data[i])
        {
            pos += weight[i];
            sum += 1;
        }
    }

    if(sum == 0)
    {
        return last_pos;
    }
		
		last_pos = pos/sum;
		
    return last_pos;
}
