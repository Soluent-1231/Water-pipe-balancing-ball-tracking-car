#include "k230.h"
#include "atk_ms901m_uart.h"
#include "ti_msp_dl_config.h"
#include <stdio.h>
#include <string.h>


char g_rx_buffer[32]; // 接收缓冲区
volatile uint8_t g_rx_index = 0;  // 缓冲区当前索引
volatile uint8_t g_line_received = 0; // 一行数据接收完成标志

// 解析出来的最终数据，供 PID 使用
volatile int g_ball_first_position = 0;
volatile int g_ball_position = 0;
volatile int g_ball_speed = 0;
volatile uint32_t uart_irq_count = 0;

void UART_1_INST_IRQHandler(void)
{
    switch (DL_UART_getPendingInterrupt(UART1))
    {
			uint8_t rx_data;
        case DL_UART_IIDX_RX:
        {
            char rcv_char;

            rcv_char = DL_UART_receiveData(UART1);
						
//						printf("%s\r\n",rcv_char);
					
            if(g_line_received == 0)
            {
                if(rcv_char == '\n' || rcv_char == '\r')
                {
                    if(g_rx_index > 0)
                    {
                        g_rx_buffer[g_rx_index] = '\0';
                        g_line_received = 1;
                    }
                }
                else if(g_rx_index < sizeof(g_rx_buffer)-1)
                {
                    g_rx_buffer[g_rx_index++] = rcv_char;
                }
                else
                {
                    g_rx_index = 0;
                }
            }

           
            break;
        }


        default:
            break;
    }
}