#include "driver_TM1637.h"

void TM1637_CLK_H(void)
{
    DL_GPIO_setPins(TM1637_CLK_PORT_PORT, TM1637_CLK_PORT_TM1637_CLK_PIN_PIN);
}


void TM1637_CLK_L(void)
{
    DL_GPIO_clearPins(TM1637_CLK_PORT_PORT, TM1637_CLK_PORT_TM1637_CLK_PIN_PIN);
}


void TM1637_DIO_H(void)
{
    DL_GPIO_setPins(TM1637_DIO_PORT_PORT, TM1637_DIO_PORT_TM1637_DIO_PIN_PIN);
}


void TM1637_DIO_L(void)
{
    DL_GPIO_clearPins(TM1637_DIO_PORT_PORT, TM1637_DIO_PORT_TM1637_DIO_PIN_PIN);
}

void TM1637_Delay(void)
{
    delay_cycles(100);
}

void TM1637_Start(void)
{
    TM1637_DIO_H();
    TM1637_CLK_H();

    TM1637_Delay();

    TM1637_DIO_L();

    TM1637_Delay();

    TM1637_CLK_L();
}

void TM1637_Stop(void)
{
    TM1637_CLK_L();

    TM1637_Delay();

    TM1637_DIO_L();

    TM1637_Delay();

    TM1637_CLK_H();

    TM1637_Delay();

    TM1637_DIO_H();
}

void TM1637_WriteByte(uint8_t data)
{
    for(int i=0;i<8;i++)
    {
        TM1637_CLK_L();

        if(data & 0x01)
            TM1637_DIO_H();
        else
            TM1637_DIO_L();


        TM1637_Delay();


        TM1637_CLK_H();

        TM1637_Delay();


        data >>= 1;
    }


    // ACK
    TM1637_CLK_L();
    TM1637_Delay();

    TM1637_CLK_H();
    TM1637_Delay();

    TM1637_CLK_L();
}

void TM1637_Init(void)
{
    TM1637_CLK_H();
    TM1637_DIO_H();
}

const uint8_t TM1637_SEG_TABLE[] =
{
    0x3F, //0
    0x06, //1
    0x5B, //2
    0x4F, //3
    0x66, //4
    0x6D, //5
    0x7D, //6
    0x07, //7
    0x7F, //8
    0x6F  //9
};
void TM1637_Display(uint16_t num)
{
    uint8_t display_buf[4];

    display_buf[0] = TM1637_SEG_TABLE[num / 1000 % 10];
    display_buf[1] = TM1637_SEG_TABLE[num / 100 % 10];
    display_buf[2] = TM1637_SEG_TABLE[num / 10 % 10];
    display_buf[3] = TM1637_SEG_TABLE[num % 10];

    TM1637_Start();

    TM1637_WriteByte(0x40);

    TM1637_Stop();

    TM1637_Start();

    TM1637_WriteByte(0xC0);


    for(uint8_t i = 0; i < 4; i++)
    {
        TM1637_WriteByte(display_buf[i]);
    }


    TM1637_Stop();

    TM1637_Start();

    TM1637_WriteByte(0x88 | 7);

    TM1637_Stop();
}