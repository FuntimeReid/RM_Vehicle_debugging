//
// Created by 73932 on 2024/10/14.
//

#include "Driver_DM4310.h"

#include "main.h"

static CAN_TxHeaderTypeDef  DM4310_tx_message;
static uint8_t              DM4310_can_send_data[8];

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    // converts unsigned int to float, given range and number of bits
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

int float_to_uint(float x, float x_min, float x_max, int bits)
{
    // Converts a float to an unsigned int, given range and number of bits
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}

void ctrl_motor(float _pos, float _vel,float _KP, float _KD, float _torq)
{
    uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
    pos_tmp = float_to_uint(_pos, P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(_vel, V_MIN, V_MAX, 12);
    kp_tmp = float_to_uint(_KP, KP_MIN, KP_MAX, 12);
    kd_tmp = float_to_uint(_KD, KD_MIN, KD_MAX, 12);
    tor_tmp = float_to_uint(_torq, T_MIN, T_MAX, 12);

    uint32_t send_mail_box;
    DM4310_tx_message.StdId = 0x01;
    DM4310_tx_message.IDE = CAN_ID_STD;
    DM4310_tx_message.RTR = CAN_RTR_DATA;
    DM4310_tx_message.DLC = 0x08;
    DM4310_can_send_data[0] = (pos_tmp >> 8);
    DM4310_can_send_data[1] = pos_tmp;
    DM4310_can_send_data[2] = (vel_tmp >> 4);
    DM4310_can_send_data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
    DM4310_can_send_data[4] = kp_tmp;
    DM4310_can_send_data[5] = (kd_tmp >> 4);
    DM4310_can_send_data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
    DM4310_can_send_data[7] = tor_tmp;

    HAL_CAN_AddTxMessage(&hcan1, &DM4310_tx_message, DM4310_can_send_data, &send_mail_box);
}

void ctrl_motor_Init(void)
{
    uint32_t send_mail_box;
    DM4310_tx_message.StdId = 0x01;
    DM4310_tx_message.IDE = CAN_ID_STD;
    DM4310_tx_message.RTR = CAN_RTR_DATA;
    DM4310_tx_message.DLC = 0x08;
    DM4310_can_send_data[0] = 0xFF;
    DM4310_can_send_data[1] = 0xFF;
    DM4310_can_send_data[2] = 0xFF;
    DM4310_can_send_data[3] = 0xFF;
    DM4310_can_send_data[4] = 0xFF;
    DM4310_can_send_data[5] = 0xFF;
    DM4310_can_send_data[6] = 0xFF;
    DM4310_can_send_data[7] = 0xFC;

    HAL_CAN_AddTxMessage(&hcan1, &DM4310_tx_message, DM4310_can_send_data, &send_mail_box);
}

void ctrl_motor_Clear(void)
{
    uint32_t send_mail_box;
    DM4310_tx_message.StdId = 0x01;
    DM4310_tx_message.IDE = CAN_ID_STD;
    DM4310_tx_message.RTR = CAN_RTR_DATA;
    DM4310_tx_message.DLC = 0x08;
    DM4310_can_send_data[0] = 0xFF;
    DM4310_can_send_data[1] = 0xFF;
    DM4310_can_send_data[2] = 0xFF;
    DM4310_can_send_data[3] = 0xFF;
    DM4310_can_send_data[4] = 0xFF;
    DM4310_can_send_data[5] = 0xFF;
    DM4310_can_send_data[6] = 0xFF;
    DM4310_can_send_data[7] = 0xFB;

    HAL_CAN_AddTxMessage(&hcan1, &DM4310_tx_message, DM4310_can_send_data, &send_mail_box);
}

void ctrl_motor_Exit(void)
{
    uint32_t send_mail_box;
    DM4310_tx_message.StdId = 0x01;
    DM4310_tx_message.IDE = CAN_ID_STD;
    DM4310_tx_message.RTR = CAN_RTR_DATA;
    DM4310_tx_message.DLC = 0x08;
    DM4310_can_send_data[0] = 0xFF;
    DM4310_can_send_data[1] = 0xFF;
    DM4310_can_send_data[2] = 0xFF;
    DM4310_can_send_data[3] = 0xFF;
    DM4310_can_send_data[4] = 0xFF;
    DM4310_can_send_data[5] = 0xFF;
    DM4310_can_send_data[6] = 0xFF;
    DM4310_can_send_data[7] = 0xFD;

    HAL_CAN_AddTxMessage(&hcan1, &DM4310_tx_message, DM4310_can_send_data, &send_mail_box);
}