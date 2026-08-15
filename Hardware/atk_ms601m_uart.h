#ifndef __ATK_MS601M_UART_H
#define __ATK_MS601M_UART_H

#include "ti_msp_dl_config.h"  /* 关键：包含TI配置，确保能识别 SysConfig 中断和引脚宏 */

/* 1. 定义软件 FIFO 缓冲区大小（根据你的需求调整大小） */
#define ATK_MS601M_UART_RX_FIFO_BUF_SIZE    256  

/* 2. 核心对接：将代码里的宏映射到你在 SysConfig 里配置的串口名字 */
/* 注意：如果你在 SysConfig 里给 UART 命名为其他名字（例如 UART_0），请将下面的 ATK_MS601M 替换为 UART_0 */
#define ATK_MS601M_UART_INST               UART_1_INST
#define ATK_MS601M_UART_INST_IRQHandler    UART_1_INST_IRQHandler

/* 函数声明，供外部文件（如 atk_ms601m.c）调用 */
void atk_ms601m_uart_init(uint32_t baudrate);
void atk_ms601m_uart_send(uint8_t *dat, uint8_t len);
uint16_t atk_ms601m_uart_rx_fifo_read(uint8_t *dat, uint16_t len);
uint8_t atk_ms601m_uart_rx_fifo_write(uint8_t *dat, uint16_t len);
void atk_ms601m_rx_fifo_flush(void);

#endif /* __ATK_MS601M_UART_H */