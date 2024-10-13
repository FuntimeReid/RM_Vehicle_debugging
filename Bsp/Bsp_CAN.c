//
// Created by 73932 on 2024/10/13.
//

#include "Bsp_CAN.h"

#include <sys/types.h>

#include "main.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void can_filter_init(void)
{
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

static CAN_TxHeaderTypeDef  stm32_tx_message;
static uint8_t              stm32_can_send_data[8];

/**
  * @brief          hal库CAN回调函数,接收电机数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
    if (hcan == &hcan1)
    {
        if(rx_header.StdId==0x206)
        {

        }
    }
    if (hcan == &hcan2)
    {
        if(rx_header.StdId==0x206)
        {

        }
    }
}

void CAN_cmd_m3508(void)
{
    uint32_t send_mail_box;
    stm32_tx_message.StdId = CAN_STM32_ID;
    stm32_tx_message.IDE = CAN_ID_STD;
    stm32_tx_message.RTR = CAN_RTR_DATA;
    stm32_tx_message.DLC = 0x08;

    uint16_t ch0_tmp=(uint16_t)(RC_Ctl.rc.ch0+1024);
    uint16_t ch1_tmp=(uint16_t)(RC_Ctl.rc.ch1+1024);
    uint16_t ch2_tmp=(uint16_t)(RC_Ctl.rc.ch2+1024);
    uint16_t ch3_tmp=(uint16_t)(RC_Ctl.rc.ch3+1024);

    // 将 ch0, ch1, ch2, ch3, s1, s2, RCMode 打包进 stm32_can_send_data
    uint64_t packed_data = 0;

    // 将各个变量打包成 64 位数据
    packed_data |= ((uint64_t)ch0_tmp & 0x7FF) << 0;            // ch0 占用 11 位，从第 0 位开始
    packed_data |= ((uint64_t)ch1_tmp & 0x7FF) << 11;           // ch1 占用 11 位，从第 11 位开始
    packed_data |= ((uint64_t)ch2_tmp & 0x7FF) << 22;           // ch2 占用 11 位，从第 22 位开始
    packed_data |= ((uint64_t)ch3_tmp & 0x7FF) << 33;           // ch3 占用 11 位，从第 33 位开始
    packed_data |= ((uint64_t)RC_Ctl.rc.s1 & 0x3) << 44;        // s1 占用 2 位，从第 44 位开始
    packed_data |= ((uint64_t)RC_Ctl.rc.s2 & 0x3) << 46;        // s2 占用 2 位，从第 46 位开始
    packed_data |= ((uint64_t)RCMode & 0x1) << 48;              // RCMode 占用 1 位，从第 48 位开始

    // 将 64 位数据分配到 8 个字节的数组中
    for (int i = 0; i < 8; i++)
    {
        stm32_can_send_data[i] = (packed_data >> (i * 8)) & 0xFF;
    }

    HAL_CAN_AddTxMessage(&hcan2, &stm32_tx_message, stm32_can_send_data, &send_mail_box);
}