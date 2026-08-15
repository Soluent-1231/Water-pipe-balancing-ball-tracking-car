#include "pid.h"
#include "driver_motor.h"
#include "grey.h"
#include "ramp.h"
#include "driver_TM1637.h"
#include "driver_Servo.h"

#define IntMax 100

// 指令加速度前馈增益：车加速/减速时提前倾斜管道，抵消小球惯性
#define K_FF_ACC  80.0f

void PID_Update(PID_t *p)
{
	p->Error1 = p->Error0;
	p->Error0 = p->Target - p->Actual;
	
	if (p->Ki != 0)
	{
		p->ErrorInt += p->Error0;
		
		if(p->ErrorInt > IntMax)
			p->ErrorInt = IntMax;

		if(p->ErrorInt < -IntMax)
			p->ErrorInt = -IntMax;
	}
	else
	{
		p->ErrorInt = 0;
	}
	
	p->Out = p->Kp * p->Error0
		   + p->Ki * p->ErrorInt
		   + p->Kd * (p->Error0 - p->Error1);
	
	if (p->Out > p->OutMax) {p->Out = p->OutMax;}
	if (p->Out < p->OutMin) {p->Out = p->OutMin;}
}




#define BASE_PWM    0
#define LOST_LINE_SPEED 8

float last_turn=0;
int8_t direction = 0;
uint8_t brake_count=0;
int16_t left_pwm;
int16_t right_pwm;
float turn_left_target = 0;
float turn_right_target = 0;
float turn_left_speed = 0;
float turn_right_speed = 0;
uint8_t stop_flag = 0;
uint8_t stop_count = 0;
uint8_t last_stop_state = 0;
extern uint16_t car_running_time;

extern PID_t PID_Left;
extern PID_t PID_Right;
extern PID_t PID_Track;
extern PID_t PID_Yaw;
extern PID_t PID_K230;
extern PID_t PID_Servo;
extern PID_t PID_K230_run;
extern PID_t PID_Servo_run;

extern TrackState_t TrackState;
extern uint8_t Base_speed;
void Track_PID_Update(void)
{
		bool right_angle_R = (!gray_sensor_data[0] && !gray_sensor_data[1] && gray_sensor_data[5] && gray_sensor_data[6] && gray_sensor_data[7]);
		bool right_angle_L = (!gray_sensor_data[6] && !gray_sensor_data[7] && gray_sensor_data[0] && gray_sensor_data[1] && gray_sensor_data[2]);
		bool no_line =!gray_sensor_data[0] && !gray_sensor_data[1] && !gray_sensor_data[2] && !gray_sensor_data[3] && !gray_sensor_data[4] && !gray_sensor_data[5] && !gray_sensor_data[6] && !gray_sensor_data[7];
		bool stop = gray_sensor_data[3] && gray_sensor_data[4] && gray_sensor_data[2] && gray_sensor_data[5];
		
		if(TrackState == TRACK_NORMAL)
		{		
				if(stop && last_stop_state == 0)
				{
						stop_count++;
						last_stop_state = 1;
				}

				if(!stop)
				{
						last_stop_state = 0;
				}


				if(stop_count >= 2)
				{
						TrackState = TRACK_STOP;

						PID_Left.ErrorInt = 0;
						PID_Right.ErrorInt = 0;
						PID_Track.ErrorInt = 0;
				}
				else if(right_angle_R)
				{
						set_left_motor(0);
						set_right_motor(0);
					
						direction  = -1;
						
						turn_left_speed = 4.0f;
						turn_right_speed = -1.0f;
					
						TrackState = TRACK_BRAKE;

						PID_Left.ErrorInt  = 0;
						PID_Right.ErrorInt = 0;
						PID_Track.ErrorInt = 0;
					
				}
				else if(right_angle_L)
				{		
						set_left_motor(0);
						set_right_motor(0);
					
						direction  = 1;
						
						turn_right_speed = 4.0f;
						turn_left_speed = -1.0f;
					
						TrackState = TRACK_BRAKE;

						PID_Left.ErrorInt  = 0;
						PID_Right.ErrorInt = 0;
						PID_Track.ErrorInt = 0;
					
				}
				else if(no_line)
				{
						TrackState = TRACK_NO_LINE;
					
						PID_Left.ErrorInt  = 0;
						PID_Right.ErrorInt = 0;
						PID_Track.ErrorInt = 0;
					
				}
		}
		switch(TrackState)
		{
			case TRACK_NORMAL:
			{
					PID_Track.Target = 0;
					PID_Track.Actual = position;

					PID_Update(&PID_Track);

					float turn = PID_Track.Out;
					
					if(fabs(turn)<0.5f){turn=0;}
					if(fabs(turn)>0.5f){last_turn=turn;if(last_turn > 2)last_turn = 2;if(last_turn < -2)last_turn = -2;}
					
					
					PID_Left.Target  = Base_speed - turn;
					PID_Right.Target = Base_speed + turn;

					PID_Left.Actual  = left_speed;
					PID_Right.Actual = right_speed;

					PID_Update(&PID_Left);
					PID_Update(&PID_Right);

					left_pwm  = BASE_PWM + PID_Left.Out;
					right_pwm = BASE_PWM + PID_Right.Out;
					
					if(left_pwm >=999)left_pwm=999;if(left_pwm <=-999)left_pwm=-999;
					if(right_pwm >=999)right_pwm=999;if(right_pwm <=-999)right_pwm=-999;
				
					set_left_motor(left_pwm);
					set_right_motor(right_pwm);

					break;
			}
//			case TRACK_RIGHT_ANGLE_R:
//			{
//					turn_left_target = 6.5;
//					turn_right_target = -4.5;
//					
//					turn_left_speed =Ramp(turn_left_speed,turn_left_target,0.6f);
//					turn_right_speed =Ramp(turn_right_speed,turn_right_target,0.8f);
//				
//					PID_Left.Target  = turn_left_speed;
//					PID_Right.Target = turn_right_speed;

//					PID_Left.Actual  = left_speed;
//					PID_Right.Actual = right_speed;
//				
//					PID_Update(&PID_Left);
//					PID_Update(&PID_Right);

//					set_left_motor(BASE_PWM + PID_Left.Out);
//					set_right_motor(BASE_PWM + PID_Right.Out);

//					if(gray_sensor_data[3] && gray_sensor_data[4])
//					{
//							TrackState = TRACK_NORMAL;
//						
//							turn_left_speed = 0;
//							turn_right_speed = 0;

//							PID_Left.ErrorInt=0;
//							PID_Right.ErrorInt=0;
//					}
//					break;
//			}
//			case TRACK_RIGHT_ANGLE_L:
//			{
//					turn_left_target = -4.5;
//					turn_right_target = 6.5;
//					
//					turn_left_speed =Ramp(turn_left_speed,turn_left_target,0.8f);
//					turn_right_speed =Ramp(turn_right_speed,turn_right_target,0.6f);
//				
//					PID_Left.Target  = turn_left_speed;
//					PID_Right.Target = turn_right_speed;

//					PID_Left.Actual  = left_speed;
//					PID_Right.Actual = right_speed;
//				
//					PID_Update(&PID_Left);
//					PID_Update(&PID_Right);

//					set_left_motor(BASE_PWM + PID_Left.Out);
//					set_right_motor(BASE_PWM + PID_Right.Out);

//					if(gray_sensor_data[3] && gray_sensor_data[4])
//					{
//							TrackState = TRACK_NORMAL;
//						
//							turn_left_speed = 0;
//							turn_right_speed = 0;

//							PID_Left.ErrorInt=0;
//							PID_Right.ErrorInt=0;
//					}
//					break;
//			}
//			case TRACK_BRAKE:
//			{
//					set_left_motor(-20);
//					set_right_motor(-20);

//					brake_count++;

//					if(brake_count>=3)
//					{		
//							PID_Left.ErrorInt  = 0;
//							PID_Right.ErrorInt = 0;
//							PID_Track.ErrorInt = 0;
//					
//							if(direction==-1)
//									TrackState=TRACK_RIGHT_ANGLE_R;

//							else
//									TrackState=TRACK_RIGHT_ANGLE_L;

//							brake_count=0;
//					}
//					
//					break;
//			}
			case TRACK_NO_LINE:
			{
					PID_Left.Target = LOST_LINE_SPEED + last_turn * 0.5f;
					PID_Right.Target = LOST_LINE_SPEED - last_turn * 0.5f;

					PID_Left.Actual = left_speed;
					PID_Right.Actual = right_speed;

					PID_Update(&PID_Left);
					PID_Update(&PID_Right);

					left_pwm = BASE_PWM + PID_Left.Out;
					right_pwm = BASE_PWM + PID_Right.Out;

					set_left_motor(left_pwm);
					set_right_motor(right_pwm);
					
					if(!no_line)
					{
							TrackState = TRACK_NORMAL;

							PID_Left.ErrorInt  = 0;
							PID_Right.ErrorInt = 0;
							PID_Track.ErrorInt = 0;
					}
				
					break;
			}
			case TRACK_STOP:
			{
					set_left_motor(-80);
					set_right_motor(-80);

					static uint16_t stop_count=0;

					stop_count++;

					if(stop_count>10)
					{
							set_left_motor(0);
							set_right_motor(0);
					}
					
					PID_Left.ErrorInt  = 0;
					PID_Right.ErrorInt = 0;
					PID_Track.ErrorInt = 0;
					
					break;
			}
			default:
			{
					TrackState = TRACK_NORMAL;
					break;
			}
		}
}


extern TrackState_t TrackState;
extern uint8_t Base_speed;
void Cycle_PID_Update(void)
{		
		switch(TrackState)
		{
			case TRACK_NORMAL:
			{
					PID_Track.Target = 0;
					PID_Track.Actual = position;

					PID_Update(&PID_Track);

					float turn = PID_Track.Out;
					
					if(fabs(turn)<0.5f){turn=0;}
					if(fabs(turn)>0.5f){last_turn=turn;if(last_turn > 2)last_turn = 2;if(last_turn < -2)last_turn = -2;}
					
					
					PID_Left.Target  = Base_speed - turn;
					PID_Right.Target = Base_speed + turn;

					PID_Left.Actual  = left_speed;
					PID_Right.Actual = right_speed;

					PID_Update(&PID_Left);
					PID_Update(&PID_Right);

					left_pwm  = BASE_PWM + PID_Left.Out;
					right_pwm = BASE_PWM + PID_Right.Out;
					
					if(left_pwm >=999)left_pwm=999;if(left_pwm <=-999)left_pwm=-999;
					if(right_pwm >=999)right_pwm=999;if(right_pwm <=-999)right_pwm=-999;
				
					set_left_motor(left_pwm);
					set_right_motor(right_pwm);

					break;
			}
			default:
			{
					TrackState = TRACK_NORMAL;
					break;
			}
		}
}



extern float target_yaw;
extern float current_yaw;
void straight_PID_Update(void)
{
	float yaw_error = 0;
	float yaw_output = 0;

    //================速度环================

    PID_Left.Target  = Base_speed;
    PID_Right.Target = Base_speed;


    PID_Left.Actual  = left_speed;
    PID_Right.Actual = right_speed;


    PID_Update(&PID_Left);
    PID_Update(&PID_Right);


    left_pwm  = PID_Left.Out;
    right_pwm = PID_Right.Out;



    //================Yaw航向环================

    PID_Yaw.Target = target_yaw;
    PID_Yaw.Actual = current_yaw;


    PID_Update(&PID_Yaw);


    yaw_output = PID_Yaw.Out;



    //================方向修正================

    //车头偏右，需要左轮加速，右轮减速
    left_pwm  += yaw_output;
    right_pwm -= yaw_output;



    //限幅

    if(left_pwm > 999)
        left_pwm=999;

    if(left_pwm < -999)
        left_pwm=-999;


    if(right_pwm >999)
        right_pwm=999;

    if(right_pwm <-999)
        right_pwm=-999;



    set_left_motor(left_pwm);
    set_right_motor(right_pwm);

}

//+5CENTER:240 -5CENTER:-140
#define center 0
extern int16_t CENTER;
extern volatile int16_t ball_x;
extern volatile int16_t ball_s;    
float target_out;
float servo_angle;
void Balance_PID_Update_one(void)
{
    // ==========位置环==========
    
    PID_K230.Target = CENTER;

    PID_K230.Actual = ball_x;

    PID_Update(&PID_K230);

    target_out = PID_K230.Out;



    //  ==========速度环==========

    PID_Servo.Target = target_out;

    PID_Servo.Actual = ball_s;

    PID_Update(&PID_Servo);

    servo_angle = PID_Servo.Out;

    if(servo_angle > 100){servo_angle = 100;}
    if(servo_angle < -100){servo_angle = -100;}

    Servo_SetAngle(90 - servo_angle);
}



extern float accel_x_filter;
extern float accel_y_filter;
void Balance_PID_Update_two(void)
{
		float K_ACC=0.05;
		float K_ACC_Y = 0.15f;
		float servo_angle;
    float acc_comp;
    // ==========位置环==========
    
    PID_K230_run.Target = CENTER;

    PID_K230_run.Actual = ball_x;

    PID_Update(&PID_K230_run);

    target_out = PID_K230_run.Out;



    //  ==========速度环==========

    PID_Servo_run.Target = target_out;

    PID_Servo_run.Actual = ball_s;

    PID_Update(&PID_Servo_run);

    servo_angle = PID_Servo_run.Out;


		acc_comp = K_ACC * (accel_x_filter+8);
    servo_angle += acc_comp;
		acc_comp = K_ACC_Y * accel_y_filter;
    servo_angle += acc_comp;

    // 指令加速度前馈：车加速/减速时提前倾斜管道，抵消小球惯性
    servo_angle += K_FF_ACC * RampS_GetAccel();


    if(servo_angle > 80){servo_angle = 80;}
    if(servo_angle < -80){servo_angle = -80;}

    Servo_SetAngle(90 - servo_angle);
}


extern float accel_x_filter;
extern float accel_y_filter;
extern uint8_t stop_flagg;

void Balance_PID_Update_three(void)
{
    float K_ACC_X = 0.04f;
    float K_ACC_Y = 0.15f;

    float servo_angle;
    float acc_x_comp;
    float acc_y_comp;

    float servo_limit = 80.0f;


    //==========位置环==========

    PID_K230_run.Target = CENTER;

    PID_K230_run.Actual = ball_x;

    PID_Update(&PID_K230_run);


    target_out = PID_K230_run.Out;


    //弯道/停车限制目标速度
    if(stop_flagg == 1)
    {
        target_out *= 0.3f;       //停车阶段降低球运动目标
    }


    //目标速度限幅
    if(target_out > 80)
    {
        target_out = 80;
    }

    if(target_out < -80)
    {
        target_out = -80;
    }



    //==========速度环==========

    PID_Servo_run.Target = target_out;

    PID_Servo_run.Actual = ball_s;

    PID_Update(&PID_Servo_run);


    servo_angle = PID_Servo_run.Out;



    //==========加速度前馈==========

    acc_x_comp = K_ACC_X * accel_x_filter;

    acc_y_comp = K_ACC_Y * accel_y_filter;


    //限制补偿量，避免突然打舵
    if(acc_x_comp > 10)
        acc_x_comp = 10;

    if(acc_x_comp < -10)
        acc_x_comp = -10;


    if(acc_y_comp > 10)
        acc_y_comp = 10;

    if(acc_y_comp < -10)
        acc_y_comp = -10;


    servo_angle += acc_x_comp;
    servo_angle += acc_y_comp;

    // 指令加速度前馈：车加速/减速时提前倾斜管道，抵消小球惯性
    servo_angle += K_FF_ACC * RampS_GetAccel();



    //==========停车降低舵机动作==========

    if(stop_flagg == 1)
    {
        servo_limit = 40.0f;
    }
    else
    {
        servo_limit = 80.0f;
    }



    if(servo_angle > servo_limit)
    {
        servo_angle = servo_limit;
    }

    if(servo_angle < -servo_limit)
    {
        servo_angle = -servo_limit;
    }



    Servo_SetAngle(90 - servo_angle);
}
