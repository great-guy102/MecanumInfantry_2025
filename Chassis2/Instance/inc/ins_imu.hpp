/**
 *******************************************************************************
 * @file      : ins_imu.hpp
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
#ifndef INSTANCE_INS_IMU_HPP_
#define INSTANCE_INS_IMU_HPP_

/* Includes ------------------------------------------------------------------*/
#include "imu.hpp"

namespace hw_imu = hello_world::imu;
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

hw_imu::Imu *GetImu(void);

#endif /* INSTANCE_INS_IMU_HPP_ */
