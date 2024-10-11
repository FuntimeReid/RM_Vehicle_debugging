//
// Created by 73932 on 2024/10/10.
//

#include "Driver_BMI088.h"

float BMI088_ACCEL_SEN = BMI088_ACCEL_3G_SEN;
float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;

uint8_t pTxData;
uint8_t pRxData;
uint8_t buf[8]={0,0,0,0,0,0,0,0};
volatile float accelerometer[3];
volatile float gyro[3];

void Read_Accelerometer(void)
{
    uint8_t i=0;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    pTxData = (0x12 | 0x80);
    HAL_SPI_Transmit(&hspi1, &pTxData, 1, 1000);
    while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX);
    HAL_SPI_Receive(&hspi1, &pRxData, 1, 1000);
    while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_RX);

    for (i = 0; i < 6; i++)
    {
        HAL_SPI_Receive(&hspi1, &pRxData, 1, 1000);
        while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_RX);
        buf[i] = pRxData;
    }
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

    accelerometer[0] = ((int16_t)((buf[1]) << 8) | buf[0]) * BMI088_ACCEL_SEN;
    accelerometer[1] = ((int16_t)((buf[3]) << 8) | buf[2]) * BMI088_ACCEL_SEN;
    accelerometer[2] = ((int16_t)((buf[5]) << 8) | buf[4]) * BMI088_ACCEL_SEN;
}

void Read_Gyroscope(void)
{
    uint8_t i=0;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    pTxData = (0x00 | 0x80);
    HAL_SPI_Transmit(&hspi1, &pTxData, 1, 1000);
    while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX);

    for (i = 0; i < 8; i++)
    {
        HAL_SPI_Receive(&hspi1, &pRxData, 1, 1000);
        while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_RX);
        buf[i] = pRxData;
    }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

    if(buf[0] == 0x0F)    // 检查是否是陀螺仪的数据
    {
        gyro[0] = ((int16_t)((buf[3]) << 8) | buf[2]) * BMI088_GYRO_SEN;
        gyro[1] = ((int16_t)((buf[5]) << 8) | buf[4]) * BMI088_GYRO_SEN;
        gyro[2] = ((int16_t)((buf[7]) << 8) | buf[6]) * BMI088_GYRO_SEN;
    }
}
