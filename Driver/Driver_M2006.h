//
// Created by 73932 on 2024/10/15.
//

#ifndef DRIVER_M2006_H
#define DRIVER_M2006_H
#include <stdint.h>

#endif //DRIVER_M2006_H

extern int16_t OutputCur_2006;

extern int32_t TargetPosition; 		        //目标位置
extern int32_t TargetCircle; 		        //目标圈数

void M2006_Position_PID(void);
void M2006_Speed_PID(void);