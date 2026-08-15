#include "ti_msp_dl_config.h"

float Ramp(float current,float target,float step)
{
    if(current < target)
    {
        current += step;
        if(current > target)
            current = target;
    }
    else if(current > target)
    {
        current -= step;
        if(current < target)
            current = target;
    }

    return current;
}

/* ===== S型斜坡 =====
 * 原来的 Ramp 每周期固定跳 step，相当于速度"阶跃"，起步那一下会踢到小球。
 * S型斜坡改成：
 *   1. 由剩余速度差算出期望加速度（限幅 max_accel）
 *   2. 加速度再按 max_jerk 平滑逼近（加速度也不突变）
 *   3. 速度 = 速度 + 加速度
 * 这样起步/停车速度都是圆滑过渡，没有瞬时冲击。
 */
#define S_RAMP_KP     1.0f   // 速度差→加速度的比例系数，越大收敛越快
#define S_RAMP_SNAP   0.02f  // 离目标多近时直接锁死到目标（保证停车能到 0）

static float s_ramp_velocity = 0.0f;   // 当前速度
static float s_ramp_accel   = 0.0f;    // 当前加速度

void RampS_Reset(float velocity)
{
    s_ramp_velocity = velocity;
    s_ramp_accel   = 0.0f;
}

float RampS(float target, float max_accel, float max_jerk)
{
    float dv;
    float a_target;
    float da;

    dv = target - s_ramp_velocity;

    /* 期望加速度 = 剩余速度差 * 比例，接近目标时自然减小 */
    a_target = dv * S_RAMP_KP;
    if (a_target >  max_accel) a_target =  max_accel;
    if (a_target < -max_accel) a_target = -max_accel;

    /* 加速度按 jerk 限幅，平滑逼近 a_target */
    da = a_target - s_ramp_accel;
    if (da >  max_jerk) da =  max_jerk;
    if (da < -max_jerk) da = -max_jerk;
    s_ramp_accel += da;

    /* 速度积分 */
    s_ramp_velocity += s_ramp_accel;

    /* 到位：离目标很近时直接锁死，保证精确到达（尤其停车到 0） */
    if ((target - s_ramp_velocity) <  S_RAMP_SNAP &&
        (target - s_ramp_velocity) > -S_RAMP_SNAP)
    {
        s_ramp_velocity = target;
        s_ramp_accel   = 0.0f;
    }

    return s_ramp_velocity;
}

/* 返回当前 S 型斜坡的指令加速度，供小球平衡做前馈 */
float RampS_GetAccel(void)
{
    return s_ramp_accel;
}
