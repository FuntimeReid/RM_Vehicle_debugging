/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
// ReSharper disable CppParameterMayBeConstPtrOrRef
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Bsp_CAN.h"
#include "Bsp_Controller.h"
#include "Driver_DM4310.h"
#include "Driver_M2006.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void HAL_UART_IDLE_Callback(UART_HandleTypeDef *huart);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

static int32_t tim1_cnt=0;
static int16_t RCMode_cnt_tmp=0;
int16_t DMMode_Init=0;
int16_t DMMode_RC=0;
int32_t DM_cnt=0;
float DM_TargerPosition=1.5;
float DM_TargerPosition_tmp=1.5;
uint8_t pitch_status=0;
uint8_t M2006_status=1;
uint16_t M2006_status_cnt=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim10;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim2;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */

  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line3 interrupt.
  */
void EXTI3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI3_IRQn 0 */

  /* USER CODE END EXTI3_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(DRDY_IST8310_Pin);
  /* USER CODE BEGIN EXTI3_IRQn 1 */

  /* USER CODE END EXTI3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line4 interrupt.
  */
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */

  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(INT1_ACCEL_Pin);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

  /* USER CODE END DMA1_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles CAN1 RX0 interrupts.
  */
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */

  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */

  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(INT1_GYRO_Pin);
  HAL_GPIO_EXTI_IRQHandler(HALL_SENSOR_PIN1_Pin);
  HAL_GPIO_EXTI_IRQHandler(HALL_SENSOR_PIN2_Pin);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
  */
void TIM1_UP_TIM10_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  HAL_TIM_IRQHandler(&htim10);
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  uart_receive_handler(&huart3);//调用之前定义的函数，传入DBUS串口的地�???????，以处理接收事件

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles CAN2 RX0 interrupts.
  */
void CAN2_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN2_RX0_IRQn 0 */

  /* USER CODE END CAN2_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan2);
  /* USER CODE BEGIN CAN2_RX0_IRQn 1 */

  /* USER CODE END CAN2_RX0_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  if (htim->Instance == TIM1)
  {
    //TIM1频率为1000Hz
    tim1_cnt++;

    //关控状态判断 每0.7s判断一次
    //开控时RCMode=1 关控时RCMode=0
    if(tim1_cnt%70==1)
    {
      if(RCMode_cnt==RCMode_cnt_tmp)
      {
        RCMode=0;
        RCMode_cnt=0;
      }
      else
      {
        RCMode=1;
      }
      RCMode_cnt_tmp=RCMode_cnt;
    }

    //500hz向底盘发送遥控器数据
    if(tim1_cnt % 2 == 1)
    {
      CAN_cmd_m3508();
    }

    //100Hz发送pitch轴电机控制数据
    if(tim1_cnt % 10 == 1)
    {
      //pitch轴电机初始化
      if(DM_cnt<300)
      {
        DM_TargerPosition=1.5;
        ctrl_motor(DM_TargerPosition,0,8,0.3,0);
        DM_cnt++;
      }
      if(DM_cnt==300)
      {
        DM_TargerPosition=pitch_position+6.28f;
        DMMode_Init=1;
        DM_cnt++;
      }

      //设计关控保护重启逻辑 重启后重新初始化并读取当前位置作为TargetPosition
      if(RCMode==0)
      {
        DMMode_RC=0;
        DM_TargerPosition_tmp=pitch_cnt;
      }
      if(RCMode==1&&DMMode_Init==1&&DMMode_RC==0)
      {
        ctrl_motor_Init();
        if(pitch_cnt!=DM_TargerPosition_tmp)
        {
          DM_TargerPosition=pitch_position+6.283f;
          DMMode_RC=1;
        }
      }

      if(RCMode==1&&DMMode_Init==1&&DMMode_RC==1)
      {
        //电池外面是昊京的的车TargetPosition介于-0.74~0.2,软控范围限制在-0.71~0.15,平放初始化大概在-0.3
        //电池外面是Tower的车TargerPosition介于1.02~2.09,实际控制限制在1.07~2.03,平放初始化大概在1.5
        DM_TargerPosition += 0.000015*RC_Ctl.rc.ch1;
        if (DM_TargerPosition > 2.03)
        {
          DM_TargerPosition = 2.03;
        }
        if (DM_TargerPosition < 1.07)
        {
          DM_TargerPosition = 1.07;
        }
        ctrl_motor(DM_TargerPosition,0,50,0.5,0);
      }
    }

    //200hz检测pitch轴DM电机状态 关控时使其失能
    if(tim1_cnt % 5 == 1)
    {
      if(RCMode==1)
      {
        if(pitch_status==0)
        {
          ctrl_motor_Init();
          pitch_status=1;
        }
      }
      if(RCMode==0)
      {
        if(pitch_status==1)
        {
          ctrl_motor_Exit();
          pitch_status=0;
        }
      }
    }

    //500hz控制yaw轴电机
    if(tim1_cnt % 2 == 1)
    {
      if(RCMode==1)
      {
        CAN_cmd_6020();
      }
    }

    //发送拨弹M2006电机控制数据(速度PID1000Hz,位置PID250Hz)
    if(RCMode==1)
    {
      //堵转保护
      if((RC_Ctl.rc.s2==3)&&(M2006_status==0))   //回转后等待70ms再恢复拨弹模式
      {
        M2006_status_cnt++;
        if(M2006_status_cnt>70)
        {
          M2006_status=1;
          M2006_status_cnt=0;
        }
      }

      //每0.3s转一格
      if((tim1_cnt%300==1)&&(RC_Ctl.rc.s2==3)&&(M2006_status==1))
      {
        TargetCircle+=4;
      }

      if(tim1_cnt % 4 == 1)
      {
        M2006_Position_PID();
      }
      CAN_cmd_2006();

      if((M2006_torque>8000)&&(M2006_status==1))    //如果电机扭矩过大,判断进入堵转状状态,回转一格
      {
        TargetCircle-=4;
        M2006_status=0;
      }
    }
    }
    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM2)
    {
      HAL_IncTick();
    }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/* USER CODE END 1 */
