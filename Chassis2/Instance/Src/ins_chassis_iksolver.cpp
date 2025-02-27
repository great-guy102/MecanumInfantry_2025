/**
 *******************************************************************************
 * @file      :ins_chassis_iksolver.cpp
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
#include "ins_chassis_iksolver.hpp"
/* Private constants ---------------------------------------------------------*/

//TODO：整车移植
const float kWheelRadius = 76 * 0.001; ///< 轮子半径 [m]
const float kWheelBase = 0.357;        ///< 左右轮距 [m]
const float kWheelTrack = 0.384;       ///< 前后轮距 [m]
const float kWheelGamma = PI / 4;      ///< Swidish 轮的 gamma 角

const hw_chassis_iksolver::PosVec kCenterPos =
    hw_chassis_iksolver::PosVec(0, 0);
const hw_chassis_iksolver::WheelParams kIkSolverParams = {
    .theta_vel_fdb = 0,
    .radius = kWheelRadius,
};
/* Private macro -------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

hw_chassis_iksolver::ChassisIkSolver unique_chassis_iksolver =
    hw_chassis_iksolver::ChassisIkSolver(kCenterPos);
bool is_chassis_iksolver_init = false;
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

static void InitChassisIkSolver(void) {
  hw_chassis_iksolver::WheelParams ik_solver_params[4] = {
      kIkSolverParams,
      kIkSolverParams,
      kIkSolverParams,
      kIkSolverParams,
  };
  if (unique_chassis_iksolver.size() == 0) {
    // 左前轮
    ik_solver_params[0].wheel_pos =
        hw_chassis_iksolver::PosVec(kWheelTrack / 2, kWheelBase / 2);
    ik_solver_params[0].gamma = -kWheelGamma;
    // 左后轮
    ik_solver_params[1].wheel_pos =
        hw_chassis_iksolver::PosVec(-kWheelTrack / 2, kWheelBase / 2);
    ik_solver_params[1].gamma = kWheelGamma;
    // 右后轮
    ik_solver_params[2].wheel_pos =
        hw_chassis_iksolver::PosVec(-kWheelTrack / 2, -kWheelBase / 2);
    ik_solver_params[2].gamma = -kWheelGamma;
    // 右前轮
    ik_solver_params[3].wheel_pos =
        hw_chassis_iksolver::PosVec(kWheelTrack / 2, -kWheelBase / 2);
    ik_solver_params[3].gamma = kWheelGamma;
    
    for (int i = 0; i < 4; i++) {
      unique_chassis_iksolver.append(
          hw_chassis_iksolver::WheelType::kSwedish,
          ik_solver_params[i]);
    }
  }
}

/* Exported function definitions ---------------------------------------------*/
hw_chassis_iksolver::ChassisIkSolver *GetChassisIkSolver(void) {
  if (!is_chassis_iksolver_init) {
    is_chassis_iksolver_init = true;
    InitChassisIkSolver();
  }
  return &unique_chassis_iksolver;
};
/* Private function definitions ----------------------------------------------*/