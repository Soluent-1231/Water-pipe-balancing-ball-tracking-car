/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"
#include "pid.h"
#include "driver_motor.h"
#include "driver_servo.h"
#include "k230.h"
#include "grey.h"
#include "ramp.h"
#include "key.h"
#include "driver_TM1637.h"
#include "atk_ms601m.h"
#include "atk_ms601m_uart.h"
#include "atk_ms901m.h"
#include "atk_ms901m_uart.h"

#include <stdio.h>

// ===== 调试参数 =====
extern volatile uint32_t uart_irq_count;
// ===== 硬件参数 =====
#define AB_RUN_SPEED    12.0f
#define BB_RUN_SPEED    10.0f

// ===== S型斜坡参数（单位：每 20ms 控制周期）=====
#define S_RAMP_MAX_ACCEL    0.08f   // 最大加速度(速度/周期²)，越小起步越柔
#define S_RAMP_MAX_JERK     0.016f  // 最大加加速度(速度/周期³)，越小冲击越小

//编码器
volatile int32_t left_pulse_count  = 0;
volatile int32_t right_pulse_count = 0;
volatile int16_t left_speed = 0;
volatile int16_t right_speed = 0;

//电机
extern float Base_speed_target;
extern float Base_speed_now;

//舵机
uint8_t servo_flag = 0;

//灰度传感器
extern uint8_t gray_sensor_data[8];
float position = 0;

//数码管
volatile uint8_t display_flag=0; 

//时间
uint16_t car_running_time=0;
static uint16_t car_running_time_stopping=0;
uint8_t car_running_time_flag=0;

//按键
uint8_t KeyNum = 0;

//串口
extern char g_rx_buffer[32];
extern volatile uint8_t g_line_received;
volatile int16_t ball_x = 0;
volatile int16_t ball_s = 0;
extern volatile int g_ball_first_position;
extern volatile int g_ball_position;
extern volatile int g_ball_speed;
extern volatile uint8_t g_rx_index;
volatile uint8_t g_stop_flag;

//IMU
float accel_x = 0.0f, accel_y = 0.0f, accel_z = 0.0f;
float current_yaw = 0.0f;
atk_ms901m_accelerometer_data_t ms901m_accel;
atk_ms901m_gyro_data_t ms901m_gyro;
atk_ms901m_attitude_data_t ms901m_attitude;
float accel_x_filter;
float accel_y_filter;
float target_yaw;

// ===== PID代码部分参数 =====
TrackState_t TrackState = TRACK_NORMAL;

volatile uint8_t control_pid_flag_for_track = 0;
PID_t PID_Left;
PID_t PID_Right;
PID_t PID_Track;
PID_t PID_Yaw;

volatile uint8_t control_pid_flag_for_balance = 0;
PID_t PID_K230;
PID_t PID_Servo;

volatile uint8_t control_pid_flag_for_balance_and_run = 0;
PID_t PID_K230_run;
PID_t PID_Servo_run;

uint8_t Base_speed = 0;
int16_t CENTER = 0;

//FLAG
uint8_t center_flag = 0;
uint8_t start_flag = 0;
uint8_t stop_flagg = 0;
uint8_t yaw_init_flag=0;

//题目
typedef enum
{
    NONE,
    PROBLEM_ONE,
    PROBLEM_TWO,
    PROBLEM_THREE,
		PROBLEM_FOURTH,
		FIFTH,
		SIXTH
}ProblemState;
ProblemState problem_state = NONE;

int main(void)
{
    SYSCFG_DL_init();
	  
	  NVIC_ClearPendingIRQ(GPIO_ENCODER_Encoder_L_A_INT_IRQN);
		NVIC_ClearPendingIRQ(GPIO_ENCODER_Encoder_R_A_INT_IRQN);
		NVIC_EnableIRQ(GPIO_ENCODER_Encoder_L_A_INT_IRQN);
    NVIC_EnableIRQ(GPIO_ENCODER_Encoder_R_A_INT_IRQN);
	  
		NVIC_ClearPendingIRQ(TIMER_TICK_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_TICK_INST_INT_IRQN); 
	
		NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
		NVIC_EnableIRQ(UART_1_INST_INT_IRQN);
	
		NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
		NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
    
    DL_TimerA_startCounter(TIMER_TICK_INST);
	
		
		
		// ===== 初始化函数 =====
		TM1637_Init();
//    Servo_SetAngle(120);

	  // ===== PID参数 =====
		PID_Left.Kp=55 ,PID_Left.Kd=1 ,PID_Left.Ki=0.5 ,PID_Left.OutMax=999 ,PID_Left.OutMin=-999; 
		PID_Right.Kp=55,PID_Right.Kd=1,PID_Right.Ki=0.5 ,PID_Right.OutMax=999 ,PID_Right.OutMin=-999; 
		PID_Track.Kp=2.5,PID_Track.Kd=5,PID_Track.Ki=0 ,PID_Track.OutMax = 5 ,PID_Track.OutMin = -5;
		PID_Yaw.Kp = 5.0,PID_Yaw.Ki = 0,PID_Yaw.Kd = 1,PID_Yaw.OutMax = 5,PID_Yaw.OutMin = -5;
		
		PID_K230.Kp=1,PID_K230.Ki=0.05,PID_K230.Kd=0.2,PID_K230.OutMax=150,PID_K230.OutMin=-150;
		PID_Servo.Kp=1,PID_Servo.Ki=0.03,PID_Servo.Kd=0.1,PID_Servo.OutMax=100,PID_Servo.OutMin=-100;
	
		PID_K230_run.Kp=0.4,PID_K230_run.Ki=0,PID_K230_run.Kd=0.5,PID_K230_run.OutMax=80,PID_K230_run.OutMin=-80;
		PID_Servo_run.Kp=0.7,PID_Servo_run.Ki=0,PID_Servo_run.Kd=1.2,PID_Servo_run.OutMax=60,PID_Servo_run.OutMin=-60;
		
    while (1) {	
			
//		printf("%d\r\n",left_pulse_count);
			
		//舵机初始化位置
		if(!servo_flag)
		{
			Servo_SetAngle(120);
			servo_flag = 1;
		}
			
    // 灰度开启			
		Update_All_Gray_Sensors();
		position = Get_Line_Position();

		// 接收串口数据
		if (g_line_received == 1)
			{
					int temp_pos = 0;
					int temp_spd = 0;
					if (sscanf(g_rx_buffer, "%d,%d", &temp_pos, &temp_spd) == 2)
					{		
							g_ball_position = temp_pos;
							g_ball_speed = temp_spd;

							ball_x = g_ball_position;
							ball_s = g_ball_speed;
//						printf("%d,%d\r\n",ball_x,ball_s);
					}
					g_rx_index = 0;
					g_line_received = 0;
					
			}		
		if (atk_ms901m_get_gyro_accelerometer(&ms901m_gyro, &ms901m_accel, 10) == ATK_MS901M_EOK)
		{
				accel_x = ms901m_accel.x; 
				accel_y = ms901m_accel.y; 
				accel_x_filter = accel_x_filter * 0.7 + accel_x * 0.3;
				accel_y_filter = accel_y_filter * 0.7 + accel_y * 0.3;
//					printf("AccX:%d\r\n", (int)(accel_x * 100));
		}
		if (atk_ms901m_get_attitude(&ms901m_attitude, 10) == ATK_MS901M_EOK) 
		{
				current_yaw = ms901m_attitude.yaw; 
				if(!yaw_init_flag)
				{
						target_yaw=current_yaw;
						yaw_init_flag=1;
//					printf("%d\r\n",(int)target_yaw);
				}
		}
			
		
		
		// 选题
		KeyNum = Key_GetNum();
		if(KeyNum == 1)
		{
				problem_state = PROBLEM_ONE;
				car_running_time = 0;
				car_running_time_flag = 0;
				display_flag = 1;
				TrackState = TRACK_NORMAL;
		}
		if(KeyNum == 2)
		{
				problem_state = PROBLEM_TWO;
				center_flag = 0;
				car_running_time = 0;
				car_running_time_flag = 0;
				TrackState = TRACK_NORMAL;
		}
		if(KeyNum == 3)
		{
				problem_state = PROBLEM_THREE;
				car_running_time = 0;
				car_running_time_flag = 0;
				start_flag = 0;
				TrackState = TRACK_NORMAL;
		}
		if(KeyNum == 4)
		{
				problem_state = PROBLEM_FOURTH;
				car_running_time = 0;
				car_running_time_flag = 0;
				start_flag = 0;
				TrackState = TRACK_NORMAL;
		}
		
		//蒙
		if(KeyNum == 5)
		{
				problem_state = FIFTH;
				car_running_time = 0;
				car_running_time_flag = 0;
				start_flag = 0;
				TrackState = TRACK_NORMAL;
		}
		if(KeyNum == 6)
		{
				problem_state = SIXTH;
				car_running_time = 0;
				car_running_time_flag = 0;
				start_flag = 0;
				TrackState = TRACK_NORMAL;
		}
		
		
		// 每题逻辑
		switch(problem_state)
		{
		case PROBLEM_ONE:
		{
			Base_speed = 17;
			//PID函数
			if(control_pid_flag_for_track)
			{
					control_pid_flag_for_track = 0;
					Track_PID_Update();
				
				//数码管
				if(TrackState == TRACK_NORMAL)
				{
						TM1637_Display(car_running_time/50);
						car_running_time_stopping = car_running_time;
				}
				else if(TrackState == TRACK_STOP)
				{
						TM1637_Display(car_running_time_stopping/50);
						car_running_time_flag = 1;
				}
			}
							break;
		}
		
		case PROBLEM_TWO:
		{	
			if(control_pid_flag_for_balance)
			{		
				control_pid_flag_for_balance = 0;
				if(center_flag == 0)
				{
					center_flag = 1;
					CENTER = 100;
				}		
				if(center_flag == 1 && car_running_time >=20)
				{
					center_flag = 2;
					CENTER = -160;	
				}
				if(center_flag == 2)
				{
					CENTER = -160;	
				}
				
					Balance_PID_Update_one();
			}
							break;
		}
			
		case PROBLEM_THREE:
		{		
				if(!start_flag)
				{
						Base_speed_now=0;
						RampS_Reset(0.0f);
						Base_speed=0;
						start_flag=1;
						stop_flagg=0;
				}

				//PID函数
				if(control_pid_flag_for_track)
				{
						control_pid_flag_for_track = 0;

						if(stop_flagg == 2)
						{
								set_motor_stop();
						}
						else if(stop_flagg == 1)
						{
								Base_speed_target = 0;
								Base_speed_now = RampS(Base_speed_target, S_RAMP_MAX_ACCEL, S_RAMP_MAX_JERK);
								Base_speed = Base_speed_now;

								PID_Left.ErrorInt  = 0;
								PID_Right.ErrorInt = 0;
								PID_Track.ErrorInt = 0;

								if(Base_speed_now <= 0.1f)
								{
										set_motor_stop();
										stop_flagg = 2;
										car_running_time_flag = 1;
										TrackState = TRACK_STOP;
								}
								else
								{
										Track_PID_Update();
								}
						}
						else
						{
								Base_speed_target = AB_RUN_SPEED;
								Base_speed_now = RampS(Base_speed_target, S_RAMP_MAX_ACCEL, S_RAMP_MAX_JERK);
								Base_speed = Base_speed_now;

								Track_PID_Update();

								if(car_running_time >= 400)
								{
										stop_flagg = 1;
								}
						}
				}
				
				
				if(control_pid_flag_for_balance_and_run)
				{
						control_pid_flag_for_balance_and_run = 0;
						CENTER = -130;
						Balance_PID_Update_two();
				}
						break;
		}
		
		case PROBLEM_FOURTH:
		{		
				if(!start_flag)
				{
						Base_speed_now=0;
						RampS_Reset(0.0f);
						Base_speed=0;
						start_flag=1;
						stop_flagg=0;
				}

				//PID函数
				if(control_pid_flag_for_track)
				{
						control_pid_flag_for_track = 0;

						if(stop_flagg == 2)
						{
								set_motor_stop();
						}
						else if(stop_flagg == 1)
						{
								Base_speed_target = 0;
								Base_speed_now = RampS(Base_speed_target, S_RAMP_MAX_ACCEL, S_RAMP_MAX_JERK);
								Base_speed = Base_speed_now;

								PID_Left.ErrorInt  = 0;
								PID_Right.ErrorInt = 0;
								PID_Track.ErrorInt = 0;

								if(Base_speed_now <= 0.1f)
								{
										set_motor_stop();
										stop_flagg = 2;
										car_running_time_flag = 1;
										TrackState = TRACK_STOP;
								}
								else
								{
										Cycle_PID_Update();
								}
						}
						else
						{
								Base_speed_target = BB_RUN_SPEED;
								Base_speed_now = RampS(Base_speed_target, S_RAMP_MAX_ACCEL, S_RAMP_MAX_JERK);
								Base_speed = Base_speed_now;

								Cycle_PID_Update();

								if(car_running_time >= 1700)
								{
//										stop_flagg = 1;
								}
						}
				}
				
				
				if(control_pid_flag_for_balance_and_run)
				{
						control_pid_flag_for_balance_and_run = 0;
						CENTER = -110;
						Balance_PID_Update_three();
				}
						break;
		}
		
		
		//蒙
		case FIFTH:
		{		
				if(!start_flag)
				{
						Base_speed_now=0;
						RampS_Reset(0.0f);
						Base_speed=0;
						start_flag=1;
						stop_flagg=0;
				}

				//PID函数
				if(control_pid_flag_for_track)
				{
						control_pid_flag_for_track = 0;

						if(stop_flagg == 2)
						{
								set_motor_stop();
						}
						else if(stop_flagg == 1)
						{
								Base_speed_target = 0;
								Base_speed_now = RampS(Base_speed_target, S_RAMP_MAX_ACCEL, S_RAMP_MAX_JERK);
								Base_speed = Base_speed_now;

								PID_Left.ErrorInt  = 0;
								PID_Right.ErrorInt = 0;
								PID_Track.ErrorInt = 0;

								if(Base_speed_now <= 0.1f)
								{
										set_motor_stop();
										stop_flagg = 2;
										car_running_time_flag = 1;
										TrackState = TRACK_STOP;
								}
								else
								{
										Cycle_PID_Update();
								}
						}
						else
						{
								Base_speed_target = BB_RUN_SPEED;
								Base_speed_now = RampS(Base_speed_target, S_RAMP_MAX_ACCEL, S_RAMP_MAX_JERK);
								Base_speed = Base_speed_now;

								Cycle_PID_Update();

								if(car_running_time >= 1700)
								{
//										stop_flagg = 1;
								}
						}
				}
				
				
				if(control_pid_flag_for_balance_and_run)
				{
						control_pid_flag_for_balance_and_run = 0;
						CENTER = -150;
						Balance_PID_Update_three();
				}
						break;
		}
		case SIXTH:
		{		
				if(!start_flag)
				{
						Base_speed_now=0;
						RampS_Reset(0.0f);
						Base_speed=0;
						start_flag=1;
						stop_flagg=0;
				}

				//PID函数
				if(control_pid_flag_for_track)
				{
						control_pid_flag_for_track = 0;

						if(stop_flagg == 2)
						{
								set_motor_stop();
						}
						else if(stop_flagg == 1)
						{
								Base_speed_target = 0;
								Base_speed_now = RampS(Base_speed_target, S_RAMP_MAX_ACCEL, S_RAMP_MAX_JERK);
								Base_speed = Base_speed_now;

								PID_Left.ErrorInt  = 0;
								PID_Right.ErrorInt = 0;
								PID_Track.ErrorInt = 0;

								if(Base_speed_now <= 0.1f)
								{
										set_motor_stop();
										stop_flagg = 2;
										car_running_time_flag = 1;
										TrackState = TRACK_STOP;
								}
								else
								{
										Cycle_PID_Update();
								}
						}
						else
						{
								Base_speed_target = BB_RUN_SPEED;
								Base_speed_now = RampS(Base_speed_target, S_RAMP_MAX_ACCEL, S_RAMP_MAX_JERK);
								Base_speed = Base_speed_now;

								Cycle_PID_Update();

								if(car_running_time >= 1700)
								{
//										stop_flagg = 1;
								}
						}
				}
				
				
				if(control_pid_flag_for_balance_and_run)
				{
						control_pid_flag_for_balance_and_run = 0;
						CENTER = -60;
						Balance_PID_Update_three();
				}
						break;
		}
		default:
		{
				break;
		}
		}
}
}

// ===== 中断部分 =====
void TIMER_TICK_INST_IRQHandler(void)
{
    switch (DL_TimerA_getPendingInterrupt(TIMER_TICK_INST)) 
		{
        case DL_TIMERA_IIDX_ZERO: 
        {
						static float filtered_right_speed = 0.0f;
						static float filtered_left_speed = 0.0f;
					
				// 计算速度
            static int32_t last_left_pulse = 0;
            static int32_t last_right_pulse = 0;
            
            int32_t cur_left_pulse = left_pulse_count;
            int32_t cur_right_pulse = right_pulse_count;
            
            int32_t raw_left_speed = cur_left_pulse - last_left_pulse;
            int32_t raw_right_speed = cur_right_pulse - last_right_pulse;
						
						//滤波
					  filtered_left_speed = (filtered_left_speed * 0.7f) + ((float)raw_left_speed * 0.3f);
						filtered_right_speed = (filtered_right_speed * 0.7f) + ((float)raw_right_speed * 0.3f);
            
					  left_speed = (int16_t)filtered_left_speed;
					  right_speed = (int16_t)filtered_right_speed;
					
            last_left_pulse = cur_left_pulse;
            last_right_pulse = cur_right_pulse;
					
					
					control_pid_flag_for_track = 1;// 开启PID
					control_pid_flag_for_balance = 1;
					
					if (g_line_received == 1)
					{
						control_pid_flag_for_balance_and_run = 1;
					}
					
//          if(display_flag)// 开启数码管
//					{
//							car_running_time++;
//					}
					if(!car_running_time_flag)
					{
						car_running_time++;
					}
					
					
					
					
            break;
        }
        default:
            break;
    }
}



// ===== 编码器部分 =====
//volatile int32_t left_pulse_count  = 0;
//volatile int32_t right_pulse_count = 0;
//volatile int16_t left_speed = 0;
//volatile int16_t right_speed = 0;
void GROUP1_IRQHandler(void)
{
    // ===== 左轮 =====
    uint32_t left_status =
        DL_GPIO_getEnabledInterruptStatus(
            GPIO_ENCODER_Encoder_L_A_PORT,
            GPIO_ENCODER_Encoder_L_A_Encoder_L_A_PIN);

    if(left_status & GPIO_ENCODER_Encoder_L_A_Encoder_L_A_PIN)
    {
        bool A =
            DL_GPIO_readPins(
                GPIO_ENCODER_Encoder_L_A_PORT,
                GPIO_ENCODER_Encoder_L_A_Encoder_L_A_PIN);

        bool B =
            DL_GPIO_readPins(
                GPIO_ENCODER_Encoder_L_B_PORT,
                GPIO_ENCODER_Encoder_L_B_Encoder_L_B_PIN);

        if(A ^ B)
            left_pulse_count++;
        else
            left_pulse_count--;

        DL_GPIO_clearInterruptStatus(
            GPIO_ENCODER_Encoder_L_A_PORT,
            GPIO_ENCODER_Encoder_L_A_Encoder_L_A_PIN);
    }

    // ===== 右轮 =====
    uint32_t right_status =
        DL_GPIO_getEnabledInterruptStatus(
            GPIO_ENCODER_Encoder_R_A_PORT,
            GPIO_ENCODER_Encoder_R_A_Encoder_R_A_PIN);

    if(right_status & GPIO_ENCODER_Encoder_R_A_Encoder_R_A_PIN)
    {
        bool A =
            DL_GPIO_readPins(
                GPIO_ENCODER_Encoder_R_A_PORT,
                GPIO_ENCODER_Encoder_R_A_Encoder_R_A_PIN);

        bool B =
            DL_GPIO_readPins(
                GPIO_ENCODER_Encoder_R_B_PORT,
                GPIO_ENCODER_Encoder_R_B_Encoder_R_B_PIN);

        if(A ^ B)
            right_pulse_count++;
        else
            right_pulse_count--;

        DL_GPIO_clearInterruptStatus(
            GPIO_ENCODER_Encoder_R_A_PORT,
            GPIO_ENCODER_Encoder_R_A_Encoder_R_A_PIN);
    }
}



// ===== 串口重定向 =====（重要误删）
__asm(".global __use_no_semihosting\n\t");
FILE __stdout;
void _sys_exit(int x)
{
    (void)x;
}
void _ttywrch(int ch)
{
    (void)ch;
}
int fputc(int ch, FILE *f)
{
    while (DL_UART_isTXFIFOFull(UART_2_INST));
    DL_UART_transmitData(UART_2_INST, ch);
    return ch;
}
