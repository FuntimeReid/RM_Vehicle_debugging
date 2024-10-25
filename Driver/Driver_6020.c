//
// Created by 73932 on 2024/10/9.
//

#include "Driver_6020.h"

#include <stdint.h>

#include "Bsp_CAN.h"
#include "ins_task.h"
#include "main.h"

volatile int16_t OutputVoltage_6020=0;

volatile float TargetPosition_6020=0;
volatile float TargetSpeed_6020=0;

float Speed_Kp_6020=6500.0f;
float Speed_Ki_6020=4.0f;
float Speed_Kd_6020=0;

float Position_Kp_6020=0.14f;
float Position_Ki_6020=0;
float Position_Kd_6020=0;

float SpdError_6020 = 0;	                //速度误差
float Spd_Integral_6020 = 0;  		        //速度积分
float SpeedErrorDiff_6020 = 0;			    //速度微分
float Spd_PreviousError_6020 = 0; 		    //速度上一次的误差

float PosError_6020 = 0;	                //位置误差
float Pos_Integral_6020 = 0;  		        //位置积分
float PosErrorDiff_6020 = 0;			    //位置微分
float Pos_PreviousError_6020 = 0; 		    //位置上一次的误差

void Speed_generate_6020(void)
{
    if(RCMode==1)
    {
        TargetSpeed_6020=(-0.1f)*RC_Ctl.rc.ch0;
    }
    else
    {
        TargetSpeed_6020=0;
    }
}

void Speed_PID_6020(void)
{
    //Speed_generate_6020();
    int32_t OutputVoltage_6020_tmp;

    SpdError_6020 = TargetSpeed_6020 - ActualGyro;			                //计算速度误差
    Spd_Integral_6020 += SpdError_6020;										//计算积分
    SpeedErrorDiff_6020 = SpdError_6020 - Spd_PreviousError_6020;			//计算微分

    if (Spd_Integral_6020 > 250.0f)										    //限制积分范围
    {
        Spd_Integral_6020 = 250.0f;
    }
    else if(Spd_Integral_6020 < -250.0f)
    {
        Spd_Integral_6020 = -250.0f;
    }

    // if(abs(TargetSpeed_6020)<=10)
    // {
    //     //Speed_Ki_6020=1.1f;
    // }
    // else if(abs(TargetSpeed_6020)>=90)
    // {
    //     //Speed_Ki_6020=3.0f;
    // }
    // else if((abs(TargetSpeed_6020)>10)&&(abs(TargetSpeed_6020)<90))
    // {
    //     //Speed_Ki_6020=1.1f+(TargetSpeed_6020-10)/80.f*1.9f;
    // }

    //PID计算输出
    OutputVoltage_6020_tmp = (int32_t)(Speed_Kp_6020 * SpdError_6020 + Speed_Ki_6020 * Spd_Integral_6020 + Speed_Kd_6020 * SpeedErrorDiff_6020);

    if (OutputVoltage_6020_tmp > 30000)										    //限制输出电流值
    {
        OutputVoltage_6020_tmp = 30000;
    }
    if(OutputVoltage_6020_tmp < -30000)
    {
        OutputVoltage_6020_tmp = -30000;
    }

    OutputVoltage_6020 = -OutputVoltage_6020_tmp;

    Spd_PreviousError_6020 = SpdError_6020;									    //更新先前误差
};

void PositionPID_6020(void)
{
    PosError_6020 = TargetPosition_6020 - ActualIns;			                //计算位置误差
    Pos_Integral_6020 += PosError_6020;								            //计算积分
    PosErrorDiff_6020 = PosError_6020 - Pos_PreviousError_6020;			        //计算微分

    if (Pos_Integral_6020 > 1800.0f)										    //限制积分范围
    {
        Pos_Integral_6020 = 1800.0f;
    }
    else if(Pos_Integral_6020 < -1800.0f)
    {
        Pos_Integral_6020 = -1800.0f;
    }

    //PID计算输出
    TargetSpeed_6020 = Position_Kp_6020 * PosError_6020 + Position_Ki_6020 * Pos_Integral_6020 + Position_Kd_6020 * PosErrorDiff_6020;

    if (TargetSpeed_6020 > 20.0f)										        //限制输出速度
    {
        TargetSpeed_6020 = 20.0f;
    }
    if(TargetSpeed_6020 < -20.0f)
    {
        TargetSpeed_6020 = -20.0f;
    }

    Pos_PreviousError_6020 = PosError_6020;									    //更新先前误差
}