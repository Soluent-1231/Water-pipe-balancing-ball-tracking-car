#ifndef __ATK_MS901M_H
#define __ATK_MS901M_H

#include <stdint.h>
#include <stddef.h>

#define delay_us(us)   delay_cycles((us) * 32)
/* 错误码定义 */
#define ATK_MS901M_EOK                          0       /* 成功 */
#define ATK_MS901M_ERROR                        1       /* 通用错误 */
#define ATK_MS901M_EINVAL                       2       /* 参数无效 */
#define ATK_MS901M_ETIMEOUT                     3       /* 超时 */

/* 缓存大小限制 */
#define ATK_MS901M_FRAME_DAT_MAX_SIZE          32

/* 帧ID类型 */
#define ATK_MS901M_FRAME_ID_TYPE_UPLOAD        0x00
#define ATK_MS901M_FRAME_ID_TYPE_ACK           0x01

/* 主动上传帧的 ID 定义 */
#define ATK_MS901M_FRAME_ID_ATTITUDE           0x01    /* 姿态角 */
#define ATK_MS901M_FRAME_ID_QUAT               0x02    /* 四元数 */
#define ATK_MS901M_FRAME_ID_GYRO_ACCE          0x03    /* 陀螺仪与加速度计 */
#define ATK_MS901M_FRAME_ID_MAG                0x04    /* 磁力计与温度 */
#define ATK_MS901M_FRAME_ID_BARO               0x05    /* 气压计 */
#define ATK_MS901M_FRAME_ID_PORT               0x06    /* 端口数据 */

/* 寄存器 ID 定义（用于读写操作） */
#define ATK_MS901M_FRAME_ID_REG_GYROFSR        0x10    /* 陀螺仪满量程 */
#define ATK_MS901M_FRAME_ID_REG_ACCFSR         0x11    /* 加速度计满量程 */
#define ATK_MS901M_FRAME_ID_REG_LEDOFF         0x12    /* LED开关状态 */
#define ATK_MS901M_FRAME_ID_REG_D0MODE         0x20    /* D0端口模式 */
#define ATK_MS901M_FRAME_ID_REG_D1MODE         0x21    /* D1端口模式 */
#define ATK_MS901M_FRAME_ID_REG_D2MODE         0x22    /* D2端口模式 */
#define ATK_MS901M_FRAME_ID_REG_D3MODE         0x23    /* D3端口模式 */
#define ATK_MS901M_FRAME_ID_REG_D1PULSE        0x25    /* D1高电平脉宽 */
#define ATK_MS901M_FRAME_ID_REG_D3PULSE        0x27    /* D3高电平脉宽 */
#define ATK_MS901M_FRAME_ID_REG_D1PERIOD       0x29    /* D1脉冲周期 */
#define ATK_MS901M_FRAME_ID_REG_D3PERIOD       0x2B    /* D3脉冲周期 */

/* 端口定义 */
typedef enum {
    ATK_MS901M_PORT_D0 = 0,
    ATK_MS901M_PORT_D1,
    ATK_MS901M_PORT_D2,
    ATK_MS901M_PORT_D3
} atk_ms901m_port_t;

/* 端口模式定义 */
typedef enum {
    ATK_MS901M_PORT_MODE_INPUT_ANALOG = 0,
    ATK_MS901M_PORT_MODE_INPUT_DIGITAL,
    ATK_MS901M_PORT_MODE_OUTPUT_DIGITAL_HIGH,
    ATK_MS901M_PORT_MODE_OUTPUT_DIGITAL_LOW,
    ATK_MS901M_PORT_MODE_OUTPUT_PWM
} atk_ms901m_port_mode_t;

/* LED状态定义 */
typedef enum {
    ATK_MS901M_LED_ON = 0,
    ATK_MS901M_LED_OFF = 1
} atk_ms901m_led_state_t;

/* 数据结构体定义 */
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} atk_ms901m_raw_data_t;

typedef struct {
    float roll;
    float pitch;
    float yaw;
} atk_ms901m_attitude_data_t;

typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
} atk_ms901m_quaternion_data_t;

typedef struct {
    atk_ms901m_raw_data_t raw;
    float x;
    float y;
    float z;
} atk_ms901m_gyro_data_t;

typedef struct {
    atk_ms901m_raw_data_t raw;
    float x;
    float y;
    float z;
} atk_ms901m_accelerometer_data_t;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
    float temperature;
} atk_ms901m_magnetometer_data_t;

typedef struct {
    int32_t pressure;
    int32_t altitude;
    float temperature;
} atk_ms901m_barometer_data_t;

typedef struct {
    uint16_t d0;
    uint16_t d1;
    uint16_t d2;
    uint16_t d3;
} atk_ms901m_port_data_t;

/* 外部函数声明 */
uint8_t atk_ms901m_init(uint32_t baudrate);
uint8_t atk_ms901m_read_reg_by_id(uint8_t id, uint8_t *dat, uint32_t timeout);
uint8_t atk_ms901m_write_reg_by_id(uint8_t id, uint8_t len, uint8_t *dat);
uint8_t atk_ms901m_get_attitude(atk_ms901m_attitude_data_t *attitude_dat, uint32_t timeout);
uint8_t atk_ms901m_get_quaternion(atk_ms901m_quaternion_data_t *quaternion_dat, uint32_t timeout);
uint8_t atk_ms901m_get_gyro_accelerometer(atk_ms901m_gyro_data_t *gyro_dat, atk_ms901m_accelerometer_data_t *accelerometer_dat, uint32_t timeout);
uint8_t atk_ms901m_get_magnetometer(atk_ms901m_magnetometer_data_t *magnetometer_dat, uint32_t timeout);
uint8_t atk_ms901m_get_barometer(atk_ms901m_barometer_data_t *barometer_dat, uint32_t timeout);
uint8_t atk_ms901m_get_port(atk_ms901m_port_data_t *port_dat, uint32_t timeout);
uint8_t atk_ms901m_get_led_state(atk_ms901m_led_state_t *state, uint32_t timeout);
uint8_t atk_ms901m_set_led_state(atk_ms901m_led_state_t state, uint32_t timeout);
uint8_t atk_ms901m_get_port_mode(atk_ms901m_port_t port, atk_ms901m_port_mode_t *mode, uint32_t timeout);
uint8_t atk_ms901m_set_port_mode(atk_ms901m_port_t port, atk_ms901m_port_mode_t mode, uint32_t timeout);
uint8_t atk_ms901m_get_port_pwm_pulse(atk_ms901m_port_t port, uint16_t *pulse, uint32_t timeout);
uint8_t atk_ms901m_set_port_pwm_pulse(atk_ms901m_port_t port, uint16_t pulse, uint32_t timeout);
uint8_t atk_ms901m_get_port_pwm_period(atk_ms901m_port_t port, uint16_t *period, uint32_t timeout);
uint8_t atk_ms901m_set_port_pwm_period(atk_ms901m_port_t port, uint16_t period, uint32_t timeout);

#endif /* __ATK_MS901M_H */