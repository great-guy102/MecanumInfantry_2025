/**
 *******************************************************************************
 * @file      :ins_chassis_fksolver.hpp
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef INSTANCE_INS_CHASSIS_FKSOLVER_HPP_
#define INSTANCE_INS_CHASSIS_FKSOLVER_HPP_

/* Includes ------------------------------------------------------------------*/
#include "chassis_fksolver.hpp"

namespace rbt_chassis_fksolver = robot::chassis_fk_solver;
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
rbt_chassis_fksolver::ChassisFkSolver *GetChassisFkSolver(void);

#endif /* INSTANCE_INS_CHASSIS_FKSOLVER_HPP_ */