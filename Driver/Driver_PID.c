//
// Created by 73932 on 2024/10/9.
//

#include "Driver_PID.h"

#include <stdint.h>

#include "Bsp_CAN.h"
#include "main.h"

volatile int16_t OutputVoltage_6020=0;

volatile int16_t TargetSpeed_6020=0;

float Speed_Kp_6020=450.0f;
float Speed_Ki_6020=10.0f;
float Speed_Kd_6020=0;

int16_t SpdError_6020 = 0;	                //速度误差
int32_t Spd_Integral_6020 = 0;  		    //速度积分
int16_t SpeedErrorDiff_6020=0;				//速度微分
int16_t Spd_PreviousError_6020 = 0; 		//速度上一次的误差

void Speed_generate_6020(void)
{
    if(RCMode==1)
    {
        TargetSpeed_6020=(-0.15f)*RC_Ctl.rc.ch2;
    }
    else
    {
        TargetSpeed_6020=0;
    }
}

void Speed_PID_6020(void)
{
    Speed_generate_6020();

    int32_t OutputVoltage_6020_tmp;

    SpdError_6020 = TargetSpeed_6020 - GM6020_speed;			            //计算速度误差
    Spd_Integral_6020 += SpdError_6020;										//计算积分
    SpeedErrorDiff_6020 = SpdError_6020 - Spd_PreviousError_6020;			//计算微分

    if (Spd_Integral_6020 > 1800)										    //限制积分范围
    {
        Spd_Integral_6020 = 1800;
    }
    else if(Spd_Integral_6020 < -1800)
    {
        Spd_Integral_6020 = -1800;
    }

    //PID计算输出
    OutputVoltage_6020_tmp = (int32_t)((int32_t)(Speed_Kp_6020 * SpdError_6020) + (int32_t)(Speed_Ki_6020 * Spd_Integral_6020) + Speed_Kd_6020 * SpeedErrorDiff_6020);

    if (OutputVoltage_6020_tmp > 25000)										    //限制输出电流值
    {
        OutputVoltage_6020_tmp = 25000;
    }
    if(OutputVoltage_6020_tmp < -25000)
    {
        OutputVoltage_6020_tmp = -25000;
    }

    OutputVoltage_6020 = (int16_t)OutputVoltage_6020_tmp;

    Spd_PreviousError_6020 = SpdError_6020;									//更新先前误差
};