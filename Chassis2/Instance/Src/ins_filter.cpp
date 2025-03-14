/**
 *******************************************************************************
 * @file      :ins_filter.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ins_filter.hpp"
/* Private constants ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
hw_filter::Ramp unique_ramp_cmd_vx = hw_filter::Ramp(3.6f, 0.001f, 0.0f, 1);
hw_filter::Ramp unique_ramp_cmd_vy = hw_filter::Ramp(3.6f, 0.001f, 0.0f, 1);
hw_filter::Ramp unique_ramp_wheel1_speed = hw_filter::Ramp(16.0f, 0.001f, 0.0f, 1);
hw_filter::Ramp unique_ramp_wheel2_speed = hw_filter::Ramp(16.0f, 0.001f, 0.0f, 1);
hw_filter::Ramp unique_ramp_wheel3_speed = hw_filter::Ramp(16.0f, 0.001f, 0.0f, 1);
hw_filter::Ramp unique_ramp_wheel4_speed = hw_filter::Ramp(16.0f, 0.001f, 0.0f, 1);
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
hw_filter::Ramp *GetRampCmdVx(void) { return &unique_ramp_cmd_vx; };
hw_filter::Ramp *GetRampCmdVy(void) { return &unique_ramp_cmd_vy; };
hw_filter::Ramp *GetRampWheel1Speed(void) { return &unique_ramp_wheel1_speed; };
hw_filter::Ramp *GetRampWheel2Speed(void) { return &unique_ramp_wheel2_speed; };
hw_filter::Ramp *GetRampWheel3Speed(void) { return &unique_ramp_wheel3_speed; };
hw_filter::Ramp *GetRampWheel4Speed(void) { return &unique_ramp_wheel4_speed; };

/* Private function definitions ----------------------------------------------*/
