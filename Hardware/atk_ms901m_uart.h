#ifndef __ATK_MS901M_UART_H
#define __ATK_MS901M_UART_H

#include <stdint.h>

/* 根据需要调整 FIFO 缓冲区大小 */
#define ATK_MS901M_UART_RX_FIFO_BUF_SIZE    256

/* 函数声明 */
void atk_ms901m_uart_init(uint32_t baudrate);
void atk_ms901m_uart_send(uint8_t *dat, uint8_t len);
uint16_t atk_ms901m_uart_rx_fifo_read(uint8_t *dat, uint16_t len);
uint8_t atk_ms901m_uart_rx_fifo_write(uint8_t *dat, uint16_t len);
void atk_ms901m_rx_fifo_flush(void);

#endif /* __ATK_MS901M_UART_H */