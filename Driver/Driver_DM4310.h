//
// Created by 73932 on 2024/10/14.
//

#ifndef DRIVER_DM4310_H
#define DRIVER_DM4310_H
#include <stdint.h>
#define P_MAX 3.14
#define P_MIN -3.14
#define T_MAX 10
#define T_MIN -10
#define V_MAX 30
#define V_MIN -30
#define KP_MAX 500
#define KP_MIN 0
#define KD_MAX 5
#define KD_MIN 0

#endif //DRIVER_DM4310_H

float uint_to_float(int x_int, float x_min, float x_max, int bits);
int float_to_uint(float x, float x_min, float x_max, int bits);
void ctrl_motor(float _pos, float _vel,float _KP, float _KD, float _torq);
void ctrl_motor_Init(void);
void ctrl_motor_Clear(void);
void ctrl_motor_Exit(void);