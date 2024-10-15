//
// Created by 73932 on 2024/10/15.
//

#include "Driver_M2006.h"

#include <stdint.h>

#include "Bsp_CAN.h"

int16_t M2006_TargetSpeed=0;
int16_t OutputCur_2006=0;

float Speed_Kp_2006=10.0f;
float Speed_Ki_2006=0.3f;
float Speed_Kd_2006=0;

int16_t SpdError_2006 = 0;	                //速度误差
int32_t Spd_Integral_2006 = 0;  		    //速度积分
int16_t SpeedErrorDiff_2006=0;				//速度微分
int16_t Spd_PreviousError_2006 = 0; 		//速度上一次的误差

float Pos_Kp_2006=0.12f;
float Pos_Ki_2006=0;
float Pos_Kd_2006=0;

int32_t PosError_2006 = 0;	                //位置误差
int32_t Pos_Integral_2006 = 0;  		    //位置积分
int32_t PosErrorDiff_2006 = 0;				//位置微分
int32_t Pos_PreviousError_2006 = 0; 		//位置上一次的误差

int32_t TargetPosition = 0; 		        //目标位置
int32_t TargetCircle = 0; 		            //目标圈数

void M2006_Position_PID(void)
{
    static uint16_t PreviousPosition=4096; 					//上一次电机机械位置, 初始化为半圈处

    TargetPosition += (int32_t)(8192 * TargetCircle);		//将目标圈数折合成目标位置
    TargetCircle = 0;

    if((M2006_position < 2048) && (PreviousPosition > 6144))//过零点检测
    {														//按3000rpm,每秒50转,在250Hz下的每转能分配5Hz,换言之两次检测间ActualPosition差值不会超过8192/5
        TargetPosition -= 8192;
    }
    else if((M2006_position > 6144) && (PreviousPosition < 2048))
    {
        TargetPosition += 8192;
    }

    PosError_2006= TargetPosition - (int32_t)(M2006_position);	//位置误差
    Pos_Integral_2006 += PosError_2006;							//位置积分
    PosErrorDiff_2006 = PosError_2006 - Pos_PreviousError_2006;	//位置微分

    if(Pos_Integral_2006 > 12000)							    //限制积分范围
    {
        Pos_Integral_2006 = 12000;
    }
    else if(Pos_Integral_2006 < -12000)
    {
        Pos_Integral_2006 = -12000;
    }

    int32_t Speed_From_Pos = Pos_Kp_2006 * PosError_2006 + Pos_Ki_2006 * Pos_Integral_2006 + Pos_Kd_2006 * PosErrorDiff_2006;	//PID计算目标速度

    if(Speed_From_Pos > 3600)				                    //限制目标速度范围
    {
        Speed_From_Pos = 3600;
    }
    else if(Speed_From_Pos < -3600)
    {
        Speed_From_Pos = -3600;
    }

    M2006_TargetSpeed = (int16_t)(Speed_From_Pos);
    Pos_PreviousError_2006 = PosError_2006;
    PreviousPosition = M2006_position;
};

void M2006_Speed_PID(void)
{
    int32_t OutputCur_2006_tmp;

    SpdError_2006 = M2006_TargetSpeed - M2006_speed;			            //计算速度误差
    Spd_Integral_2006 += SpdError_2006;										//计算积分
    SpeedErrorDiff_2006 = SpdError_2006 - Spd_PreviousError_2006;			//计算微分

    if (Spd_Integral_2006 > 10800)										    //限制积分范围
    {
        Spd_Integral_2006 = 10800;
    }
    else if(Spd_Integral_2006 < -10800)
    {
        Spd_Integral_2006 = -10800;
    }

    //PID计算输出
    OutputCur_2006_tmp = (int32_t)((int32_t)(Speed_Kp_2006 * SpdError_2006) + (int32_t)(Speed_Ki_2006 * Spd_Integral_2006) + Speed_Kd_2006 * SpeedErrorDiff_2006);

    if (OutputCur_2006_tmp > 10000)										    //限制输出电流值
    {
        OutputCur_2006_tmp = 10000;
    }
    if(OutputCur_2006_tmp < -10000)
    {
        OutputCur_2006_tmp = -10000;
    }

    OutputCur_2006 = (int16_t)OutputCur_2006_tmp;

    Spd_PreviousError_2006 = SpdError_2006;									//更新先前误差
}