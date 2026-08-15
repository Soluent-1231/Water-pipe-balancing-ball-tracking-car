#include "ti_msp_dl_config.h"  /* MSPM0 配置头文件，包含 DriverLib 和 SysConfig 生成的宏 */
#include "atk_ms601m.h"
#include "atk_ms601m_uart.h"
/* 假定你在 SysConfig 中将配置的 UART 命名为 ATK_MS601M_UART 
 * SysConfig 会自动在 ti_msp_dl_config.h 中生成相应宏，如：
 * ATK_MS601M_UART_INST
 * ATK_MS601M_UART_INST_IRQHandler
 */

static struct
{
    uint8_t buf[ATK_MS601M_UART_RX_FIFO_BUF_SIZE];  /* 缓冲区 */
    uint16_t size;                                  /* 缓冲区大小 */
    uint16_t reader;                                /* 读指针 */
    uint16_t writer;                                /* 写指针 */
} g_uart_rx_fifo;                                   /* UART接收FIFO */

/**
 * @brief       ATK-MS601M UART接收FIFO写入数据
 * @param       dat: 待写入数据
 *              len: 待写入数据的长度
 * @retval      0: 函数执行成功
 *              1: FIFO剩余空间不足
 */
uint8_t atk_ms601m_uart_rx_fifo_write(uint8_t *dat, uint16_t len)
{
    uint16_t i;
    
    /* 将数据写入FIFO，并更新FIFO写指针 */
    for (i = 0; i < len; i++)
    {
        g_uart_rx_fifo.buf[g_uart_rx_fifo.writer] = dat[i];
        g_uart_rx_fifo.writer = (g_uart_rx_fifo.writer + 1) % g_uart_rx_fifo.size;
    }
    
    return 0;
}

/**
 * @brief       ATK-MS601M UART接收FIFO读取数据
 * @param       dat: 读取数据存放位置
 *              len: 欲读取数据的长度
 * @retval      0: FIFO中无数据
 *              其它值: 实际读取的数据长度
 */
uint16_t atk_ms601m_uart_rx_fifo_read(uint8_t *dat, uint16_t len)
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
    
    /* 从FIFO读取数据，并更新FIFO读指针 */
    for (i = 0; i < len; i++)
    {
        dat[i] = g_uart_rx_fifo.buf[g_uart_rx_fifo.reader];
        g_uart_rx_fifo.reader = (g_uart_rx_fifo.reader + 1) % g_uart_rx_fifo.size;
    }
    
    return len;
}

/**
 * @brief       ATK-MS601M UART接收FIFO清空
 */
void atk_ms601m_rx_fifo_flush(void)
{
    g_uart_rx_fifo.writer = g_uart_rx_fifo.reader;
}

/**
 * @brief       ATK-MS601M UART发送数据
 * @param       dat: 待发送的数据
 *              len: 待发送数据的长度
 */
void atk_ms601m_uart_send(uint8_t *dat, uint8_t len)
{
    for (uint8_t i = 0; i < len; i++)
    {
        /* 阻塞等待发送FIFO有空闲空间，然后发送单字节 */
        DL_UART_Main_transmitDataBlocking(ATK_MS601M_UART_INST, dat[i]);
    }
}

/**
 * @brief       ATK-MS601M UART软件FIFO初始化
 * @note        波特率等硬件参数建议直接在 TI SysConfig 图形界面中配置
 */
void atk_ms601m_uart_init(uint32_t baudrate)
{
    /* MSPM0 推荐在 SysConfig 中配置好对应的 UART（包含波特率、中断等），
     * 并在 main 函数开头调用 SYSCFG_DL_init() 进行系统初始化。
     * 这里仅对软件实现的接收环形队列（FIFO）进行初始化。
     */
    g_uart_rx_fifo.size   = ATK_MS601M_UART_RX_FIFO_BUF_SIZE;
    g_uart_rx_fifo.reader = 0;
    g_uart_rx_fifo.writer = 0;

    /* 如果你需要开启接收中断（确保已经在中断控制器里开启了对应的NVIC接口）*/
    DL_UART_Main_enableInterrupt(ATK_MS601M_UART_INST, DL_UART_MAIN_INTERRUPT_RX);
}

/**
 * @brief       ATK-MS601M UART 中断服务函数
 * @note        函数名需要与 startup 或者是 SysConfig 生成的保持一致
 */
//void ATK_MS601M_UART_INST_IRQHandler(void)
//{
//    /* 读取当前产生的中断类型（返回的是 IIDX 枚举） */
//    switch (DL_UART_Main_getPendingInterrupt(ATK_MS601M_UART_INST))
//    {
//        /* 1. 将原先的 DL_UART_MAIN_INTERRUPT_RX 替换为包含 IIDX 的枚举 */
//        case DL_UART_IIDX_RX:
//        {
//            /* 接收中断：读取一字节数据并存入软件 FIFO */
//            uint8_t tmp = DL_UART_Main_receiveData(ATK_MS601M_UART_INST);
//            atk_ms601m_uart_rx_fifo_write(&tmp, 1);
//            break;
//        }
//        
//        /* 2. 将原先的 DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR 替换为对应的 IIDX 枚举 */
//        case DL_UART_IIDX_OVERRUN_ERROR:
//        {
//            /* 溢出错误处理 */
//            break;
//        }
//        
//        default:
//            break;
//    }
//}