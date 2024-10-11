//
// Created by 73932 on 2024/10/10.
//

#ifndef DRIVER_BMI088_H
#define DRIVER_BMI088_H

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_spi.h"
#include "main.h"

#endif //DRIVER_BMI088_H

#define BMI088_ACCEL_3G_SEN 0.0008974358974f
#define BMI088_GYRO_2000_SEN 0.00106526443603169529841533860381f

extern volatile float accelerometer[3];
extern volatile float gyro[3];
extern uint8_t pTxData;
extern uint8_t pRxData;

void Read_Accelerometer(void);
void Read_Gyroscope(void);
