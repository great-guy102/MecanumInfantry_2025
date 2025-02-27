/**
 *******************************************************************************
 * @file      :ins_motor.cpp
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
#include "ins_motor.hpp"
/* Private constants ---------------------------------------------------------*/
// TODO：整车移植
// 轮电机通用参数配置
const hw_motor::OptionalParams kMotorWheelParams = {
    .input_type = hw_motor::InputType::kCurr,
    .angle_range = hw_motor::AngleRange::kNegPiToPosPi,
    /** 是否移除电机自带的减速器 */
    .remove_build_in_reducer = true,
    /** 电机输出端实际角度与规定角度的差值 */
    .angle_offset = 0,
    /** 电机外置减速器的减速比（额外） */
    .ex_redu_rat = 14,
    .max_raw_input_lim = 16384.0f, // 报文输入限制
    .max_curr_input_lim = 20.0f,   // 电流输入限制为20.0A
};

const hw_motor::OptionalParams kMotorYawParams = {
    .input_type = hw_motor::InputType::kTorq,
    .angle_range = hw_motor::AngleRange::kNegPiToPosPi,
    .dir = hw_motor::kDirFwd,
    /** 是否移除电机自带的减速器 */
    .remove_build_in_reducer = false,
    /** 电机输出端实际角度与规定角度的差值 */
    .angle_offset = 1.34805f,
    /** 电机外置减速器的减速比（额外） */
    // .ex_redu_rat = 14,   //TODO ：和硬件了解
};

// TODO: 这里的 MotorId 需要按照实际情况修改

/* Private macro -------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
// 轮电机单例
static bool is_motor_wheel_left_front_inited = false;
hw_motor::M3508 unique_motor_wheel_left_front;

static bool is_motor_wheel_left_rear_inited = false;
hw_motor::M3508 unique_motor_wheel_left_rear;

static bool is_motor_wheel_right_rear_inited = false;
hw_motor::M3508 unique_motor_wheel_right_rear;

static bool is_motor_wheel_right_front_inited = false;
hw_motor::M3508 unique_motor_wheel_right_front;

// yaw电机单例
static bool is_motor_yaw_inited = false;
hw_motor::DM_J4310 unique_motor_yaw;

hw_motor::Motor *GetMotorWheelLeftFront() {
  if (!is_motor_wheel_left_front_inited) {
    hw_motor::OptionalParams kMotorWheelLeftFrontParams = kMotorWheelParams;
    kMotorWheelLeftFrontParams.dir = hw_motor::kDirFwd;
    unique_motor_wheel_left_front = hw_motor::M3508(0x01, kMotorWheelLeftFrontParams);
    is_motor_wheel_left_front_inited = true;
  }
  return &unique_motor_wheel_left_front;
};

hw_motor::Motor *GetMotorWheelLeftRear() {
  if (!is_motor_wheel_left_rear_inited) {
    hw_motor::OptionalParams kMotorWheelLeftRearParams = kMotorWheelParams;
    kMotorWheelLeftRearParams.dir = hw_motor::kDirFwd;
    unique_motor_wheel_left_rear = hw_motor::M3508(0x02, kMotorWheelLeftRearParams);
    is_motor_wheel_left_rear_inited = true;
  }
  return &unique_motor_wheel_left_rear;
};

hw_motor::Motor *GetMotorWheelRightRear() {
  if (!is_motor_wheel_right_rear_inited) {
    hw_motor::OptionalParams kMotorWheelRightRearParams = kMotorWheelParams;
    kMotorWheelRightRearParams.dir = hw_motor::kDirRev;
    unique_motor_wheel_right_rear = hw_motor::M3508(0x03, kMotorWheelRightRearParams);
    is_motor_wheel_right_rear_inited = true;
  }
  return &unique_motor_wheel_right_rear;
};

hw_motor::Motor *GetMotorWheelRightFront() {
  if (!is_motor_wheel_right_front_inited) {
    hw_motor::OptionalParams kMotorWheelRightFrontParams = kMotorWheelParams;
    kMotorWheelRightFrontParams.dir = hw_motor::kDirRev;
    unique_motor_wheel_right_front = hw_motor::M3508(0x04, kMotorWheelRightFrontParams);
    is_motor_wheel_right_front_inited = true;
  }
  return &unique_motor_wheel_right_front;
};

hw_motor::Motor *GetMotorYaw() {
  if (!is_motor_yaw_inited) {
    unique_motor_yaw = hw_motor::DM_J4310(0x02, kMotorYawParams);
    is_motor_yaw_inited = true;
  }
  return &unique_motor_yaw;
};

/* Private function definitions ----------------------------------------------*/