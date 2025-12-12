/**
 *******************************************************************************
 * @file      :pid.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1。<note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2025 Hello World Team,Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

#include "pid.hpp"
namespace pid {
float Pid::calc(void)
{
    integral_ += error_;
    derivative_ = error_ - prev_error_;
    float output = kp * error_ + ki * integral_ + kd * derivative_;
    if(ki != 0)
    {
        if(integral_ > 1.0)
        {
            integral_ = 0.5;
        }
        if(integral_ < -1.0)
        {
            integral_ = -0.5;
        }
    }
    if(max_ != 0)
    {
        if(output > max_)
            output = max_;
    }
    if(min_ != 0)
    {
        if(output < min_)
            output = min_;
    }
    return output;
    prev_error_ = error_;
}
}