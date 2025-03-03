/**
 *******************************************************************************
 * @file      :ins_pid.cpp
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
#include "ins_pid.hpp"

#include "motor.hpp"

/* Private constants ---------------------------------------------------------*/
const float kMaxPidOutWheel = 20.0f; ///< 3508电流控制的最大输出
const float kMaxPidOutFollowOmega = 40.0f;

const hw_pid::OutLimit kOutLimitWheel =
    hw_pid::OutLimit(true, -kMaxPidOutWheel, kMaxPidOutWheel);
const hw_pid::OutLimit kOutLimitFollowOmega =
    hw_pid::OutLimit(true, -kMaxPidOutFollowOmega, kMaxPidOutFollowOmega);

const hw_pid::MultiNodesPid::ParamsList kPidWheelParams = {
    {
        .auto_reset = true, ///< 是否自动清零
        .kp = 3000.0f * (20.0f / 16384.0f),
        .ki = 0.0f,
        .kd = 0.0f,
        // .inte_anti_windup = hw_pid::InteAntiWindup(true, -10000.0, 10000.0),
        .out_limit = kOutLimitWheel,
    },
};

const hw_pid::MultiNodesPid::ParamsList kPidFollowOmegaParams = {
    {
        .auto_reset = true,
        .kp = 7.5f, // 7.5
        .ki = 0.0f, // 0.005
        .kd = 270.0f,
        .setpoint_ramping = hw_pid::SetpointRamping(true, -0.1, 0.1, 0.1),
        .period_sub = hw_pid::PeriodSub(true, 2.0 * PI), // 双向跟随，半圈过零
        .inte_changing_rate = hw_pid::InteChangingRate(true, 0.01f, 0.5f),
        .diff_filter = hw_pid::DiffFilter(true, -0.1f, 0.1f, 0.5f),
        .out_limit = kOutLimitFollowOmega,
    },
};

const hw_pid::MultiNodesPid::Type kPidTypeCascade =
    hw_pid::MultiNodesPid::Type::kCascade;
/* Private macro -------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
// 轮电机PID
hw_pid::MultiNodesPid unique_pid_wheel_left_front(kPidTypeCascade,
                                                  kOutLimitWheel,
                                                  kPidWheelParams);
hw_pid::MultiNodesPid unique_pid_wheel_left_rear(kPidTypeCascade,
                                                 kOutLimitWheel,
                                                 kPidWheelParams);
hw_pid::MultiNodesPid unique_pid_wheel_right_rear(kPidTypeCascade,
                                                  kOutLimitWheel,
                                                  kPidWheelParams);
hw_pid::MultiNodesPid unique_pid_wheel_right_front(kPidTypeCascade,
                                                   kOutLimitWheel,
                                                   kPidWheelParams);

// 自旋PID
hw_pid::MultiNodesPid unique_pid_follow_omega(kPidTypeCascade,
                                              kOutLimitFollowOmega,
                                              kPidFollowOmegaParams);

/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
hw_pid::MultiNodesPid *GetPidWheelLeftFront() {
  return &unique_pid_wheel_left_front;
};
hw_pid::MultiNodesPid *GetPidWheelLeftRear() {
  return &unique_pid_wheel_left_rear;
};
hw_pid::MultiNodesPid *GetPidWheelRightRear() {
  return &unique_pid_wheel_right_rear;
};
hw_pid::MultiNodesPid *GetPidWheelRightFront() {
  return &unique_pid_wheel_right_front;
};

hw_pid::MultiNodesPid *GetPidFollowOmega() { return &unique_pid_follow_omega; };
/* Private function definitions ----------------------------------------------*/
