#include "ti_msp_dl_config.h"
#include "atk_ms901m.h"
#include "atk_ms901m_uart.h"

/* 
 * 注意：请在 SysConfig 图形配置工具中添加一个 UART 外设，并将其命名为 "ATK_MS901M_UART"
 * 这样工具会自动生成诸如 ATK_MS901M_UART_INST 等宏定义。
 */

static struct
{
    uint8_t buf[ATK_MS901M_UART_RX_FIFO_BUF_SIZE];  /* 缓冲 */
    uint16_t size;                                  /* 缓冲大小 */
    volatile uint16_t reader;                       /* 读指针（中断修改，加volatile） */
    volatile uint16_t writer;                       /* 写指针（中断修改，加volatile） */
} g_uart_rx_fifo;                                   /* UART接收FIFO */

/**
 * @brief        ATK-MS901M UART接收FIFO写入数据
 */
uint8_t atk_ms901m_uart_rx_fifo_write(uint8_t *dat, uint16_t len)
{
    uint16_t i;
    
    for (i = 0; i < len; i++)
    {
        g_uart_rx_fifo.buf[g_uart_rx_fifo.writer] = dat[i];
        g_uart_rx_fifo.writer = (g_uart_rx_fifo.writer + 1) % g_uart_rx_fifo.size;
    }
    
    return 0;
}

/**
 * @brief        ATK-MS901M UART接收FIFO读取数据
 */
uint16_t atk_ms901m_uart_rx_fifo_read(uint8_t *dat, uint16_t len)
{
    uint16_t fifo_usage;
    uint16_t i;
    
    /* 获取FIFO已使用大小 */
    if (g_uart_rx_fifo.writer >= g_uart_rx_fifo.reader)
    {
        fifo_usage = g_uart_rx_fifo.writer - g_uart_rx_fifo.reader;
    }
    else
    {
        fifo_usage = g_uart_rx_fifo.size - g_uart_rx_fifo.reader + g_uart_rx_fifo.writer;
    }
    
    /* FIFO数据量不足 */
    if (len > fifo_usage)
    {
        len = fifo_usage;
    }
    
    /* 从FIFO读取数据 */
    for (i = 0; i < len; i++)
    {
        dat[i] = g_uart_rx_fifo.buf[g_uart_rx_fifo.reader];
        g_uart_rx_fifo.reader = (g_uart_rx_fifo.reader + 1) % g_uart_rx_fifo.size;
    }
    
    return len;
}

/**
 * @brief        ATK-MS901M UART接收FIFO清空
 */
void atk_ms901m_rx_fifo_flush(void)
{
    g_uart_rx_fifo.writer = g_uart_rx_fifo.reader;
}

/**
 * @brief        ATK-MS901M UART发送数据
 */
void atk_ms901m_uart_send(uint8_t *dat, uint8_t len)
{
    for (uint8_t i = 0; i < len; i++)
    {
        /* 阻塞等待发送缓冲区有空位，然后发送单字节 */
        DL_UART_transmitDataBlocking(UART_1_INST, dat[i]);
    }
}

/**
 * @brief        ATK-MS901M UART初始化
 */
void atk_ms901m_uart_init(uint32_t baudrate)
{
    /* 
     * 针对 TI MSPM0 的规范说明：
     * 引脚 GPIO、时钟源和基本参数（8位数据、1个停止位、无校验）建议直接在 SysConfig 界面中配好。
     * 如果需要在代码中动态调整波特率，可使用下方这行代码；如果 SysConfig 中已固定好波特率，此行可省略。
     */
    // DL_UART_setBaudRate(ATK_MS901M_UART_INST, CLOCK_FREQ, baudrate); 

    /* 初始化软件 FIFO 结构体 */
    g_uart_rx_fifo.size = ATK_MS901M_UART_RX_FIFO_BUF_SIZE;
    g_uart_rx_fifo.reader = 0;
    g_uart_rx_fifo.writer = 0;

    /* 清除可能存在的接收中断标志并使能接收中断 */
    DL_UART_clearInterruptStatus(UART_1_INST, DL_UART_INTERRUPT_RX);
    DL_UART_enableInterrupt(UART_1_INST, DL_UART_INTERRUPT_RX);
    
    /* 使能该 UART 的 NVIC 中断向量 */
    NVIC_EnableIRQ(UART_1_INST_INT_IRQN);
}

/**
 * @brief        ATK-MS901M UART中断服务函数
 * @note         根据 TI MSPM0 规范，中断函数名需与 SysConfig 自动生成的宏保持一致
 */
void UART_0_INST_IRQHandler(void) // <-- 核心修改：改为硬件认的函数名
{
    uint8_t rx_data;

    /* 检查是否为接收中断 */
    switch (DL_UART_getPendingInterrupt(UART_0_INST))
    {
        case DL_UART_IIDX_RX:
            /* 读取接收到的单字节数据 */
            rx_data = DL_UART_receiveData(UART_0_INST);
            /* 存入软件 FIFO */
            atk_ms901m_uart_rx_fifo_write(&rx_data, 1);
            break;
            
        default:
            break;
    }
}