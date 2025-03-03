/**
 *******************************************************************************
 * @file      :ins_chassis_fksolver.cpp
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
#include "ins_chassis_fksolver.hpp"
/* Private constants ---------------------------------------------------------*/

// TODO：整车移植
const float kWheelRadius = 76 * 0.001; ///< 轮子半径 [m]
const float kWheelBase = 0.357;        ///< 左右轮距 [m]
const float kWheelTrack = 0.384;       ///< 前后轮距 [m]

/* Private macro -------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

rbt_chassis_fksolver::ChassisFkSolver unique_chassis_fksolver =
    rbt_chassis_fksolver::ChassisFkSolver(kWheelTrack, kWheelBase,
                                          kWheelRadius);
bool is_chassis_fksolver_init = false;
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Exported function definitions ---------------------------------------------*/
rbt_chassis_fksolver::ChassisFkSolver *GetChassisFkSolver(void) {
  return &unique_chassis_fksolver;
};
/* Private function definitions ----------------------------------------------*/