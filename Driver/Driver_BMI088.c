//
// Created by 73932 on 2024/10/10.
//

#include "Driver_BMI088.h"
#include <math.h>

#define BMI088_ACCEL_3G_SEN 0.0008974358974f    //这个数字我也不知道哪来的
#define BMI088_GYRO_2000_SEN 0.00106526443603169529841533860381f    //这个数字我也不知道哪来的
float BMI088_ACCEL_SEN = BMI088_ACCEL_3G_SEN;
float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;

float quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};
float INS_angle[3] = {0.0f, 0.0f, 0.0f};

uint8_t pTxData;
uint8_t pRxData;

float accelerometer[3];
float gyro[3];
uint8_t buf[8]={0,0,0,0,0,0,0,0};


acc_raw_data_t BMI_acc;
gyro_raw_data_t BMI_gyro;

bmi088_error_e BMI088_INIT(void) {
    bmi088_error_e error = NO_ERROR;

    BMI088_CONF_INIT();

    error |= VerifyAccChipID();
    error |= VerifyGyroChipID();
    if (1) {  // 将来改成变量控制自检
        error |= VerifyAccSelfTest();
    }
    if (1) {  // 将来改成变量控制自检
        error |= VerifyGyroSelfTest();
    }
    return error;
}

void WriteDataToAcc(uint8_t addr, uint8_t data) {
    HAL_GPIO_WritePin(BMI088_ACC_GPIOx, BMI088_ACC_GPIOp, GPIO_PIN_RESET);
    uint8_t pTxData = (addr & BMI088_SPI_WRITE_CODE);
    HAL_SPI_Transmit(&BMI088_SPI, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_TX)
        ;
    pTxData = data;
    HAL_SPI_Transmit(&BMI088_SPI, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_TX)
        ;
    HAL_Delay(1);
    HAL_GPIO_WritePin(BMI088_ACC_GPIOx, BMI088_ACC_GPIOp, GPIO_PIN_SET);
}

void WriteDataToGyro(uint8_t addr, uint8_t data) {
    HAL_GPIO_WritePin(BMI088_GYRO_GPIOx, BMI088_GYRO_GPIOp, GPIO_PIN_RESET);
    uint8_t pTxData = (addr & BMI088_SPI_WRITE_CODE);
    HAL_SPI_Transmit(&BMI088_SPI, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_TX)
        ;
    pTxData = data;
    HAL_SPI_Transmit(&BMI088_SPI, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_TX)
        ;
    HAL_Delay(1);
    HAL_GPIO_WritePin(BMI088_GYRO_GPIOx, BMI088_GYRO_GPIOp, GPIO_PIN_SET);
}

void ReadSingleDataFromAcc(uint8_t addr, uint8_t *data) {
    HAL_GPIO_WritePin(BMI088_ACC_GPIOx, BMI088_ACC_GPIOp, GPIO_PIN_RESET);
    uint8_t pTxData = (addr | BMI088_SPI_READ_CODE);
    HAL_SPI_Transmit(&BMI088_SPI, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_TX)
        ;
    HAL_SPI_Receive(&BMI088_SPI, data, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_RX)
        ;
    HAL_SPI_Receive(&BMI088_SPI, data, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_RX)
        ;
    HAL_GPIO_WritePin(BMI088_ACC_GPIOx, BMI088_ACC_GPIOp, GPIO_PIN_SET);
}

void ReadSingleDataFromGyro(uint8_t addr, uint8_t *data) {
    HAL_GPIO_WritePin(BMI088_GYRO_GPIOx, BMI088_GYRO_GPIOp, GPIO_PIN_RESET);
    uint8_t pTxData = (addr | BMI088_SPI_READ_CODE);
    HAL_SPI_Transmit(&BMI088_SPI, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_TX)
        ;
    HAL_SPI_Receive(&BMI088_SPI, data, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_RX)
        ;
    HAL_GPIO_WritePin(BMI088_GYRO_GPIOx, BMI088_GYRO_GPIOp, GPIO_PIN_SET);
}

void ReadMultiDataFromAcc(uint8_t addr, uint8_t len, uint8_t *data) {
    HAL_GPIO_WritePin(BMI088_ACC_GPIOx, BMI088_ACC_GPIOp, GPIO_PIN_RESET);
    uint8_t pTxData = (addr | BMI088_SPI_READ_CODE);
    uint8_t pRxData;
    HAL_SPI_Transmit(&BMI088_SPI, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_TX)
        ;
    HAL_SPI_Receive(&BMI088_SPI, &pRxData, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_RX)
        ;
    for (int i = 0; i < len; i++) {
        HAL_SPI_Receive(&BMI088_SPI, &pRxData, 1, 1000);
        while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_RX)
            ;
        data[i] = pRxData;
    }
    HAL_GPIO_WritePin(BMI088_ACC_GPIOx, BMI088_ACC_GPIOp, GPIO_PIN_SET);
}

void ReadMultiDataFromGyro(uint8_t addr, uint8_t len, uint8_t *data) {
    HAL_GPIO_WritePin(BMI088_GYRO_GPIOx, BMI088_GYRO_GPIOp, GPIO_PIN_RESET);
    uint8_t pTxData = (addr | BMI088_SPI_READ_CODE);
    uint8_t pRxData;
    HAL_SPI_Transmit(&BMI088_SPI, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_TX)
        ;
    for (int i = 0; i < len; i++) {
        HAL_SPI_Receive(&BMI088_SPI, &pRxData, 1, 1000);
        while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_RX)
            ;
        data[i] = pRxData;
    }
    HAL_GPIO_WritePin(BMI088_GYRO_GPIOx, BMI088_GYRO_GPIOp, GPIO_PIN_SET);
}

void BMI088_CONF_INIT(void) {
    // 加速度计初始化
    // 先软重启，清空所有寄存器
    WriteDataToAcc(ACC_SOFTRESET_ADDR, ACC_SOFTRESET_VAL);
    HAL_Delay(50);
    // 打开加速度计电源
    WriteDataToAcc(ACC_PWR_CTRL_ADDR, ACC_PWR_CTRL_ON);
    // 加速度计变成正常模式
    WriteDataToAcc(ACC_PWR_CONF_ADDR, ACC_PWR_CONF_ACT);

    // 陀螺仪初始化
    // 先软重启，清空所有寄存器
    WriteDataToGyro(GYRO_SOFTRESET_ADDR, GYRO_SOFTRESET_VAL);
    HAL_Delay(50);
    // 陀螺仪变成正常模式
    WriteDataToGyro(GYRO_LPM1_ADDR, GYRO_LPM1_NOR);

    // 加速度计配置写入
    // 写入范围，+-3g的测量范围
    WriteDataToAcc(ACC_RANGE_ADDR, ACC_RANGE_3G);
    // 写入配置，正常带宽，1600hz输出频率
    WriteDataToAcc(ACC_CONF_ADDR,
                   (ACC_CONF_RESERVED << 7) | (ACC_CONF_BWP_NORM << 6) | (ACC_CONF_ODR_1600_Hz));

    // 陀螺仪配置写入
    // 写入范围，+-500°/s的测量范围
    WriteDataToGyro(GYRO_RANGE_ADDR, GYRO_RANGE_500_DEG_S);
    // 写入带宽，2000Hz输出频率，532Hz滤波器带宽
    WriteDataToGyro(GYRO_BANDWIDTH_ADDR, GYRO_ODR_2000Hz_BANDWIDTH_532Hz);
}

bmi088_error_e VerifyAccChipID(void) {
    uint8_t chip_id;
    ReadSingleDataFromAcc(ACC_CHIP_ID_ADDR, &chip_id);
    if (chip_id != ACC_CHIP_ID_VAL) {
        return ACC_CHIP_ID_ERR;
    }
    return NO_ERROR;
}

bmi088_error_e VerifyGyroChipID(void) {
    uint8_t chip_id;
    ReadSingleDataFromGyro(GYRO_CHIP_ID_ADDR, &chip_id);
    if (chip_id != GYRO_CHIP_ID_VAL) {
        return GYRO_CHIP_ID_ERR;
    }
    return NO_ERROR;
}

bmi088_error_e VerifyAccSelfTest(void) {
    acc_raw_data_t pos_data, neg_data;
    WriteDataToAcc(ACC_RANGE_ADDR, ACC_RANGE_24G);
    WriteDataToAcc(ACC_CONF_ADDR, 0xA7);
    HAL_Delay(10);
    WriteDataToAcc(ACC_SELF_TEST_ADDR, ACC_SELF_TEST_POS);
    HAL_Delay(100);
    ReadAccData(&pos_data);
    WriteDataToAcc(ACC_SELF_TEST_ADDR, ACC_SELF_TEST_NEG);
    HAL_Delay(100);
    ReadAccData(&neg_data);
    WriteDataToAcc(ACC_SELF_TEST_ADDR, ACC_SELF_TEST_OFF);
    HAL_Delay(100);
    if ((fabs(pos_data.x - neg_data.x) > 0.1f) || (fabs(pos_data.y - neg_data.y) > 0.1f) || (fabs(pos_data.z - neg_data.z) > 0.1f)) {
        return ACC_DATA_ERR;
    }
    WriteDataToAcc(ACC_SOFTRESET_ADDR, ACC_SOFTRESET_VAL);
    WriteDataToAcc(ACC_PWR_CTRL_ADDR, ACC_PWR_CTRL_ON);
    WriteDataToAcc(ACC_PWR_CONF_ADDR, ACC_PWR_CONF_ACT);
    WriteDataToAcc(ACC_CONF_ADDR,
                   (ACC_CONF_RESERVED << 7) | (ACC_CONF_BWP_NORM << 6) | (ACC_CONF_ODR_1600_Hz));
    WriteDataToAcc(ACC_RANGE_ADDR, ACC_RANGE_3G);
    return NO_ERROR;
}

bmi088_error_e VerifyGyroSelfTest(void) {
    WriteDataToGyro(GYRO_SELF_TEST_ADDR, GYRO_SELF_TEST_ON);
    uint8_t bist_rdy = 0x00, bist_fail;
    while (bist_rdy == 0) {
        ReadSingleDataFromGyro(GYRO_SELF_TEST_ADDR, &bist_rdy);
        bist_rdy = (bist_rdy & 0x02) >> 1;
    }
    ReadSingleDataFromGyro(GYRO_SELF_TEST_ADDR, &bist_fail);
    bist_fail = (bist_fail & 0x04) >> 2;
    if (bist_fail == 0) {
        return NO_ERROR;
    } else {
        return GYRO_DATA_ERR;
    }
}

void ReadAccData(acc_raw_data_t *data) {
    uint8_t buf[ACC_XYZ_LEN], range;
    int16_t acc[3];
    ReadSingleDataFromAcc(ACC_RANGE_ADDR, &range);
    ReadMultiDataFromAcc(ACC_X_LSB_ADDR, ACC_XYZ_LEN, buf);
    acc[0] = ((int16_t)buf[1] << 8) + (int16_t)buf[0];
    acc[1] = ((int16_t)buf[3] << 8) + (int16_t)buf[2];
    acc[2] = ((int16_t)buf[5] << 8) + (int16_t)buf[4];
    data->x = (float)acc[0] * BMI088_ACCEL_3G_SEN;
    data->y = (float)acc[1] * BMI088_ACCEL_3G_SEN;
    data->z = (float)acc[2] * BMI088_ACCEL_3G_SEN;
}

void ReadGyroData(gyro_raw_data_t *data) {
    uint8_t buf[GYRO_XYZ_LEN], range;
    int16_t gyro[3];
    float unit;
    ReadSingleDataFromGyro(GYRO_RANGE_ADDR, &range);
    switch (range) {
        case 0x00:
            unit = 16.384;
            break;
        case 0x01:
            unit = 32.768;
            break;
        case 0x02:
            unit = 65.536;
            break;
        case 0x03:
            unit = 131.072;
            break;
        case 0x04:
            unit = 262.144;
            break;
        default:
            unit = 16.384;
            break;
    }
    ReadMultiDataFromGyro(GYRO_RATE_X_LSB_ADDR, GYRO_XYZ_LEN, buf);
    gyro[0] = ((int16_t)buf[1] << 8) + (int16_t)buf[0];
    gyro[1] = ((int16_t)buf[3] << 8) + (int16_t)buf[2];
    gyro[2] = ((int16_t)buf[5] << 8) + (int16_t)buf[4];
    data->roll = (float)gyro[0] / unit * DEG2SEC;
    data->pitch = (float)gyro[1] / unit * DEG2SEC;
    data->yaw = (float)gyro[2] / unit * DEG2SEC;
}

void ReadAccSensorTime(float *time) {
    uint8_t buf[SENSORTIME_LEN];
    ReadMultiDataFromAcc(SENSORTIME_0_ADDR, SENSORTIME_LEN, buf);
    *time = buf[0] * SENSORTIME_0_UNIT + buf[1] * SENSORTIME_1_UNIT + buf[2] * SENSORTIME_2_UNIT;
}

void ReadAccTemperature(float *temp) {
    uint8_t buf[TEMP_LEN];
    ReadMultiDataFromAcc(TEMP_MSB_ADDR, TEMP_LEN, buf);
    uint16_t temp_uint11 = (buf[0] << 3) + (buf[1] >> 5);
    int16_t temp_int11;
    if (temp_uint11 > 1023) {
        temp_int11 = (int16_t)temp_uint11 - 2048;
    } else {
        temp_int11 = (int16_t)temp_uint11;
    }
    *temp = temp_int11 * TEMP_UNIT + TEMP_BIAS;
}

void BMI_Init(void)
{
    //这9行代码，向地址0x7E处写入0xB6值，加速度计软件复位，使加速度计各个寄存器恢复为默认值
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);    //PA4置0，片选加速度计
    pTxData = (0x7E & 0x7F);    //Bit #0和Bit #1-7，Bit #0是0，表示写
    HAL_SPI_Transmit(&hspi1, &pTxData, 1, 1000);
    while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_TX);    //等待SPI发送完成
    pTxData = 0xB6;    //Bit #8-15
    HAL_SPI_Transmit(&hspi1, &pTxData, 1, 1000);
    while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_TX);    //等待SPI发送完成
    HAL_Delay(1);    //延时1ms
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);    //PA4置1，取消片选加速度计

    //加速度计复位后默认是暂停模式，这9行代码，向地址0x7D处写入0x04值，使加速度计进入正常模式
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);    //PA4置0，片选加速度计
    pTxData = (0x7D & 0x7F);    //Bit #0和Bit #1-7，Bit #0是0，表示写
    HAL_SPI_Transmit(&hspi1, &pTxData, 1, 1000);
    while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_TX);    //等待SPI发送完成
    pTxData = 0x04;    //Bit #8-15
    HAL_SPI_Transmit(&hspi1, &pTxData, 1, 1000);
    while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_TX);    //等待SPI发送完成
    HAL_Delay(1);    //延时1ms
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);    //PA4置1，取消片选加速度计

    //这9行代码，向地址0x14处写入0xB6值，陀螺仪软件复位，使陀螺仪各个寄存器恢复为默认值
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);    //PB0置0，片选陀螺仪
    pTxData = (0x14 & 0x7F);    //Bit #0和Bit #1-7，Bit #0是0，表示写
    HAL_SPI_Transmit(&hspi1, &pTxData, 1, 1000);
    while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_TX);    //等待SPI发送完成
    pTxData = 0xB6;    //Bit #8-15
    HAL_SPI_Transmit(&hspi1, &pTxData, 1, 1000);
    while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_TX);    //等待SPI发送完成
    HAL_Delay(30);    //延时30ms
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);    //PB0置1，取消片选陀螺仪
}

void BMI_Get(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);    //PA4置0，片选加速度计
    pTxData = (0x12 | 0x80);    //Bit #0和Bit #1-7，Bit #0是1，表示读
    HAL_SPI_Transmit(&hspi1, &pTxData, 1, 1000);
    while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_TX);    //等待SPI发送完成
    HAL_SPI_Receive(&hspi1, &pRxData, 1, 1000);    //Bit #8-15，无效值
    while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_RX);    //等待SPI接收完成
    int16_t i = 0;
    while (i < 6)
    {
        HAL_SPI_Receive(&hspi1, &pRxData, 1, 1000);    //Bit #16-23，寄存器0x12的值，然后是寄存器0x13、0x14、0x15、0x16、0x17的值
    	while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_RX);    //等待SPI接收完成
    	buf[i] = pRxData;
        i++;
    }
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);    //PA4置1，取消片选加速度计
    accelerometer[0] = ((int16_t)((buf[1]) << 8) | buf[0]) * BMI088_ACCEL_SEN;
    accelerometer[1] = ((int16_t)((buf[3]) << 8) | buf[2]) * BMI088_ACCEL_SEN;
    accelerometer[2] = ((int16_t)((buf[5]) << 8) | buf[4]) * BMI088_ACCEL_SEN;

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);    //PB0置0，片选陀螺仪
    pTxData = (0x00 | 0x80);    //Bit #0和Bit #1-7，Bit #0是1，表示读
    HAL_SPI_Transmit(&hspi1, &pTxData, 1, 1000);
    while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_TX);    //等待SPI发送完成
    i = 0;
    while (i < 8)
    {
    	HAL_SPI_Receive(&hspi1, &pRxData, 1, 1000);    //Bit #8-15，寄存器0x00的值，然后是寄存器0x01、0x02、0x03、0x04、0x05、0x06、0x07的值
    	while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_RX);    //等待SPI接收完成
    	buf[i] = pRxData;
    	i++;
    }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);    //PB0置1，取消片选陀螺仪
    if(buf[0] == 0x0F)	//buf[0]储存GYRO_CHIP_ID，应该为0x0F，判断我们读取到的是不是陀螺仪的值。
    {
    	gyro[0] = ((int16_t)((buf[3]) << 8) | buf[2]) * BMI088_GYRO_SEN;
    	gyro[1] = ((int16_t)((buf[5]) << 8) | buf[4]) * BMI088_GYRO_SEN;
    	gyro[2] = ((int16_t)((buf[7]) << 8) | buf[6]) * BMI088_GYRO_SEN;
    }
}

void get_angle(float q[4], float *yaw, float *pitch, float *roll)
{
    *yaw = atan2f(2.0f*(q[0]*q[3]+q[1]*q[2]), 2.0f*(q[0]*q[0]+q[1]*q[1])-1.0f);
    *pitch = asinf(-2.0f*(q[1]*q[3]-q[0]*q[2]));
    *roll = atan2f(2.0f*(q[0]*q[1]+q[2]*q[3]),2.0f*(q[0]*q[0]+q[3]*q[3])-1.0f);
}