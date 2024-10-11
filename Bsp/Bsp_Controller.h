//
// Created by 73932 on 2024/10/9.
//

#ifndef BSP_CONTROLLER_H
#define BSP_CONTROLLER_H

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_it.h"
#include "main.h"

#endif //BSP_CONTROLLER_H

static int uart_receive_dma_no_it(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size);
void dbus_uart_init(void);
void rc_callback_handler(RC_Ctl_t *rc, uint8_t *buff);
uint16_t dma_current_data_counter(DMA_Stream_TypeDef *dma_stream);
static void uart_rx_idle_callback(UART_HandleTypeDef* huart);
void uart_receive_handler(UART_HandleTypeDef *huart);