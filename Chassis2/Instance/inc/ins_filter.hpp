/**
 *******************************************************************************
 * @file      :ins_filter.hpp
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
#ifndef INSTANCE_INS_FILTER_HPP_
#define INSTANCE_INS_FILTER_HPP_

/* Includes ------------------------------------------------------------------*/
#include "base.hpp"
#include "ramp.hpp"

namespace hw_filter = hello_world::filter;
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
hw_filter::Ramp *GetRampCmdVx(void);
hw_filter::Ramp *GetRampCmdVy(void);
hw_filter::Ramp *GetRampWheel1Speed(void);
hw_filter::Ramp *GetRampWheel2Speed(void);
hw_filter::Ramp *GetRampWheel3Speed(void);
hw_filter::Ramp *GetRampWheel4Speed(void);
#endif /* INSTANCE_INS_FILTER_HPP_ */
