/*
 * @Author: rogue-wave zhangjingjie@zju.edu.cn
 * @Date: 2025-11-22 21:11:35
 * @LastEditors: rogue-wave zhangjingjie@zju.edu.cn
 * @LastEditTime: 2025-11-23 21:00:37
 * @FilePath: \yuntai_quanxiang\Tasks\main_task.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/**
*******************************************************************************
 * @file      :main_task.cpp
* @brief     :
* @history   :
*  Version     Date            Author          Note
*  V0.9.0      yyyy-mm-dd      <author>        1. <note>
*******************************************************************************
* @attention :
*******************************************************************************
*  Copyright (c) 2024 Hello World Team，Zhejiang University.
*  All Rights Reserved.
*******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "main_task.hpp"
#include "system_user.hpp"

#include "DT7.hpp"
#include "HW_can.hpp"
#include "dm4310_drv.hpp"
#include "iwdg.h"
#include "math.h"
#include "imu_task.hpp"
#include "pid.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
float euler_angles_raw[3] = {0.0f, 0.0f, 0.0f};
int state_imu = 0;
int state_pitch_dm = 0;
Joint_Motor_t motor_pitch;
pid::Pid pid_pitch_pos(10.0,0.1,0.7,15.0,-15.0);
pid::Pid pid_pitch_vel(1.6,0.1,0,6.5,-6.5);
extern float gyro_data[3];
extern float euler_angles[3];
extern float pos_pitch;
extern float vel_pitch;
float imu_calc(float now_angle,float raw_angle);
uint32_t tick = 0;

namespace remote_control = hello_world::devices::remote_control;
static const uint8_t kRxBufLen = remote_control::kRcRxDataLen;
static uint8_t rx_buf[kRxBufLen];
remote_control::DT7 *rc_ptr;

void RobotInit(void) { 
  rc_ptr = new remote_control::DT7(); 
  ImuInit();
}

void MainInit(void) {
  RobotInit();

  // 开启CAN1和CAN2
  CanFilter_Init(&hcan1);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  CanFilter_Init(&hcan2);
  HAL_CAN_Start(&hcan2);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);

  // 开启遥控器接收
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_buf, kRxBufLen);

  // 开启定时器
  HAL_TIM_Base_Start_IT(&htim6);
}
//euler[0]yaw,euler[1]roll,euler[2]pitch
//-0.0048
//逆时针加，顺时针减
//gyro_data[2]yaw,gyro_data[0]pitch,gyro_data[1]roll
//rc_rh有0.01的偏差，极限一样，注意
volatile float a = -2.8f;
float purpose_vel = 0.0f;
void MainTask(void) {
  tick++;
  ImuUpdate();
  if(tick<6000)
  {
    if(state_pitch_dm == 1)
    {
      mit_ctrl(&hcan2,0x02,0,0,0,0,0.0f);
      state_pitch_dm = 0;
    }
    return;
  }
  if(state_pitch_dm == 0)
  {
    enable_motor_mode(&hcan2,0x02,MIT_MODE);
    state_pitch_dm = 1;
  }
  pid_pitch_pos.set_error(a - pos_pitch);
  purpose_vel = pid_pitch_pos.calc();
  pid_pitch_vel.set_error(purpose_vel - vel_pitch);
  float output_temp =pid_pitch_vel.calc();
  //float output_temp = 0.0f;
  mit_ctrl(&hcan2,0x02,0,0,0,0,output_temp-0.9f*cosf(pos_pitch+2.9));
  if(tick%1000==0)
  {
    euler_angles[0]+=0.0077;
  }
  if(state_imu == 0)
  {
    for(int i=0;i<3;i++)
    {
      euler_angles_raw[i] = euler_angles[i];
    }
    state_imu = 1;
  }
  int16_t rc_rv_temp = (rc_ptr->rc_rv())*1000.0f;
  int16_t rc_rh_temp = (rc_ptr->rc_rh()-0.01)*1000.0f;
  int16_t temp1 = (rc_ptr->rc_lv())*1000.0f;
  int16_t temp2 = (rc_ptr->rc_lh())*1000.0f;
  uint8_t kong[8]={0,0,0,0,0,0,0,0};
  kong[0] = (uint8_t)(temp1>>8);
  kong[1] = (uint8_t)(temp1);
  kong[2] = (uint8_t)(temp2>>8);
  kong[3] = (uint8_t)(temp2);
  int16_t imu_send = (int16_t)(imu_calc(euler_angles[0],euler_angles_raw[0])*1000);
  kong[4] = (uint8_t)(imu_send>>8);
  kong[5] = (uint8_t)(imu_send);
  int16_t vel_imu_yaw = (int16_t)(-gyro_data[2]*1000);
  kong[6] = (uint8_t)(vel_imu_yaw>>8);
  kong[7] = (uint8_t)(vel_imu_yaw);
  uint8_t kong1[8]={0,0,0,0,0,0,0,0};
  kong1[0] = (uint8_t)(rc_rv_temp>>8);
  kong1[1] = (uint8_t)(rc_rv_temp);
  kong1[2] = (uint8_t)(rc_rh_temp>>8);
  kong1[3] = (uint8_t)(rc_rh_temp);
  CAN_Send_Msg(&hcan1,kong,0x1FF,8);
  CAN_Send_Msg(&hcan1,kong1,0x0FE,8);
 }

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

  if (htim == &htim6) {
    MainTask();
  }
}
uint8_t rx_data = 0;
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  if (huart == &huart3) {
    if (Size == remote_control::kRcRxDataLen) {
      // TODO:在这里刷新看门狗
      HAL_IWDG_Refresh(&hiwdg);
      rc_ptr->decode(rx_buf);
    }

    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_buf, kRxBufLen);
  }
}
float imu_calc(float now_angle,float raw_angle)
{
  if(now_angle - raw_angle <-3.14)
  {
    return now_angle + 6.2832 - raw_angle;
  }
  else if(now_angle - raw_angle > 3.1416)
  {
    return now_angle - 6.2832 - raw_angle;
  }
  return now_angle - raw_angle;
}