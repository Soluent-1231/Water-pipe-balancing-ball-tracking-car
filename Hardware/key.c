#include "ti_msp_dl_config.h"

uint8_t Key_GetNum(void)
{
    // 实时读取当前哪一个按键被按下（低电平有效），查到谁就立刻返回谁
    if ((DL_GPIO_readPins(GPIO_KEY_1_PORT, GPIO_KEY_1_PIN_1_PIN) & GPIO_KEY_1_PIN_1_PIN) == 0)      return 1;
    if ((DL_GPIO_readPins(GPIO_KEY_2_PORT, GPIO_KEY_2_PIN_2_PIN) & GPIO_KEY_2_PIN_2_PIN) == 0)      return 2;
    if ((DL_GPIO_readPins(GPIO_KEY_3_PORT, GPIO_KEY_3_PIN_3_PIN) & GPIO_KEY_3_PIN_3_PIN) == 0)      return 3;
    if ((DL_GPIO_readPins(GPIO_KEY_4_PORT, GPIO_KEY_4_PIN_4_PIN) & GPIO_KEY_4_PIN_4_PIN) == 0)      return 4;

	
		if ((DL_GPIO_readPins(GPIO_KEY_5_PORT, GPIO_KEY_5_PIN_5_PIN) & GPIO_KEY_5_PIN_5_PIN) == 0)      return 5;
		if ((DL_GPIO_readPins(GPIO_KEY_6_PORT, GPIO_KEY_6_PIN_6_PIN) & GPIO_KEY_6_PIN_6_PIN) == 0)      return 6;
		return 0;
}

