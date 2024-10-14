//
// Created by 73932 on 2024/10/13.
//

#ifndef BSP_CAN_H
#define BSP_CAN_H

#include <stdint.h>
#define CHASSIS_CAN hcan1
#define STM32_CAN hcan2


#endif //BSP_CAN_H

void can_filter_init(void);

typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,

    CAN_YAW_MOTOR_ID = 0x205,
    CAN_PIT_MOTOR_ID = 0x206,
    CAN_TRIGGER_MOTOR_ID = 0x207,
    CAN_GIMBAL_ALL_ID = 0x1FF,

    CAN_STM32_ID = 0x210,

} can_msg_id_e;

extern volatile int16_t GM6020_position;
extern volatile int16_t GM6020_speed;

void CAN_cmd_m3508(void);
void CAN_cmd_6020(void);