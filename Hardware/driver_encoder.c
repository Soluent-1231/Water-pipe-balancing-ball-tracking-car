#include "ti_msp_dl_config.h"
#include <stdio.h>
//// ===== 编码器部分 =====
//volatile int32_t left_pulse_count  = 0;
//volatile int32_t right_pulse_count = 0;

//void GROUP1_IRQHandler(void)
//{
//    // ===== 左轮 =====
//    uint32_t left_status =
//        DL_GPIO_getEnabledInterruptStatus(
//            GPIO_ENCODER_Encoder_L_A_PORT,
//            GPIO_ENCODER_Encoder_L_A_Encoder_L_A_PIN);

//    if(left_status & GPIO_ENCODER_Encoder_L_A_Encoder_L_A_PIN)
//    {
//        bool A =
//            DL_GPIO_readPins(
//                GPIO_ENCODER_Encoder_L_A_PORT,
//                GPIO_ENCODER_Encoder_L_A_Encoder_L_A_PIN);

//        bool B =
//            DL_GPIO_readPins(
//                GPIO_ENCODER_Encoder_L_B_PORT,
//                GPIO_ENCODER_Encoder_L_B_Encoder_L_B_PIN);

//        if(A ^ B)
//            left_pulse_count++;
//        else
//            left_pulse_count--;

//        DL_GPIO_clearInterruptStatus(
//            GPIO_ENCODER_Encoder_L_A_PORT,
//            GPIO_ENCODER_Encoder_L_A_Encoder_L_A_PIN);
//    }

//    // ===== 右轮 =====
//    uint32_t right_status =
//        DL_GPIO_getEnabledInterruptStatus(
//            GPIO_ENCODER_Encoder_R_A_PORT,
//            GPIO_ENCODER_Encoder_R_A_Encoder_R_A_PIN);

//    if(right_status & GPIO_ENCODER_Encoder_R_A_Encoder_R_A_PIN)
//    {
//        bool A =
//            DL_GPIO_readPins(
//                GPIO_ENCODER_Encoder_R_A_PORT,
//                GPIO_ENCODER_Encoder_R_A_Encoder_R_A_PIN);

//        bool B =
//            DL_GPIO_readPins(
//                GPIO_ENCODER_Encoder_R_B_PORT,
//                GPIO_ENCODER_Encoder_R_B_Encoder_R_B_PIN);

//        if(A ^ B)
//            right_pulse_count++;
//        else
//            right_pulse_count--;

//        DL_GPIO_clearInterruptStatus(
//            GPIO_ENCODER_Encoder_R_A_PORT,
//            GPIO_ENCODER_Encoder_R_A_Encoder_R_A_PIN);
//    }
//}
