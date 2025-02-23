/**
*******************************************************************************
 * @file      :power_limiter.cpp
* @brief     : 底盘功率限制
* @history   :
*  Version     Date            Author          Note
*  V0.9.0      2024-08-28      YangZhou        1.Create this file
*  V1.0.0      2025-01-16      Jinletian       1.重构代码
*******************************************************************************
* @attention :
*******************************************************************************
*  Copyright (c) 2025 Hello World Team,Zhejiang University.
*  All Rights Reserved.
*******************************************************************************
*/
/* Includes ------------------------------------------------------------------*/
#include "power_limiter.hpp"

#include "arm_math.h"
#include "base.hpp"

namespace hello_world
{
namespace power_limiter
{
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

HW_OPTIMIZE_O2_START

#pragma region PowerModel

PowerModel::PowerModel(const Params &params) : params_(params)
{
  HW_ASSERT(params.k1 >= 0, "k1 must be not less than 0");
  HW_ASSERT(params.k2 >= 0, "k2 must be not less than 0");
  HW_ASSERT(params.k3 >= 0, "k3 must be not less than 0");
  HW_ASSERT(params.kp >= 0, "kp must be greater than 0");
}

void PowerModel::init(const Params &params)
{
  HW_ASSERT(params.k1 >= 0, "k1 must be not less than 0");
  HW_ASSERT(params.k2 >= 0, "k2 must be not less than 0");
  HW_ASSERT(params.k3 >= 0, "k3 must be not less than 0");
  HW_ASSERT(params.kp >= 0, "kp must be not less than 0");

  params_ = params;
}

void PowerModel::setParams(const Params &params)
{
  HW_ASSERT(params.k1 >= 0, "k1 must be not less than 0");
  HW_ASSERT(params.k2 >= 0, "k2 must be not less than 0");
  HW_ASSERT(params.k3 >= 0, "k3 must be not less than 0");
  HW_ASSERT(params.kp >= 0, "kp must be not less than 0");

  params_ = params;
}

void PowerModel::setParams(float k1, float k2, float k3)
{
  HW_ASSERT(k1 >= 0, "k1 must be not less than 0");
  HW_ASSERT(k2 >= 0, "k2 must be not less than 0");
  HW_ASSERT(k3 >= 0, "k3 must be not less than 0");

  params_.k1 = k1;
  params_.k2 = k2;
  params_.k3 = k3;
}

float PowerModel::update(float spd_ref, float spd_fdb, float i_ffd, float i_fdb)
{
  status_.spd_ref = spd_ref;
  status_.spd_fdb = spd_fdb;
  status_.i_ffd = i_ffd;
  status_.i_fdb = i_fdb;

  /* 计算模型功率 */
  status_.p_model = calcPower(i_fdb, spd_fdb);

  /* 计算预测电流（限制前）*/
  status_.i_predict = calcCurrent(spd_ref, spd_fdb, i_ffd);

  /* 计算电机预测功率（限制前）*/
  status_.p_predict = calcPower(status_.i_predict, spd_fdb);

  return status_.p_predict;
}

float PowerModel::update(float spd_fdb, float i_input, float i_fdb)
{
  status_.spd_ref = 0;
  status_.spd_fdb = spd_fdb;
  status_.i_ffd = 0;
  status_.i_fdb = i_fdb;

  /* 计算模型功率 */
  status_.p_model = calcPower(i_fdb, spd_fdb);

  /* 计算预测电流（限制前）*/
  status_.i_predict = calcCurrent(0, 0, i_input);

  /* 计算电机预测功率（限制前）*/
  status_.p_predict = calcPower(status_.i_predict, spd_fdb);

  return status_.p_predict;
}

float PowerModel::limitBySpd(float k_limit)
{
  results_.k_limit = k_limit;

  /* 计算限制后的期望速度*/
  results_.spd_limited = results_.k_limit * status_.spd_ref;

  /* 计算限制后的电流*/
  results_.i_limited =
      calcCurrent(results_.spd_limited, status_.spd_fdb, status_.i_ffd);

  /* 计算限制后的功率*/
  results_.p_limited =
      calcPower(results_.i_limited, status_.spd_fdb);

  return results_.p_limited;
}

float PowerModel::limitByCurrent(float k_limit)
{
  results_.k_limit = k_limit;
  results_.spd_limited = 0;

  /* 计算限制后的电流*/
  results_.i_limited = results_.k_limit * status_.i_predict;
  results_.i_limited = calcCurrent(0, 0, results_.i_limited);

  /* 计算限制后的功率*/
  results_.p_limited =
      calcPower(results_.i_limited, status_.spd_fdb);

  return results_.p_limited;
}
#pragma endregion

#pragma region PowerLimiter
PowerLimiter::PowerLimiter(
    const StaticParams &static_params)
    : static_params_(static_params)
{
  HW_ASSERT(static_params_.p_steering_ratio >= 0 &&
                static_params_.p_steering_ratio <= 1,
            "p_steering_ratio must be in [0, 1]");
  PowerModel::Params wheel_motor_params =
      {
          .k1 = static_params_.wheel_motor_params.k1,
          .k2 = static_params_.wheel_motor_params.k2,
          .k3 = static_params_.wheel_motor_params.k3,
          .kp = static_params_.wheel_motor_params.kp,
          .out_limit = static_params_.wheel_motor_params.out_limit};
  for (size_t i = 0; i < static_params_.wheel_motor_params.motor_cnt; i++) {
    wheel_motor_model_list_.emplace_back(wheel_motor_params);
  }

  PowerModel::Params steering_motor_params =
      {
          .k1 = static_params_.steering_motor_params.k1,
          .k2 = static_params_.steering_motor_params.k2,
          .k3 = static_params_.steering_motor_params.k3,
          .kp = 0,
          .out_limit = static_params_.steering_motor_params.out_limit};
  for (size_t i = 0; i < static_params_.steering_motor_params.motor_cnt; i++) {
    steering_motor_model_list_.emplace_back(steering_motor_params);
  }
}

void PowerLimiter::updateWheelModel(const float *spd_ref_ptr, const float *spd_fdb_ptr, const float *i_ffd_ptr, const float *i_fdb_ptr)
{
  HW_ASSERT(spd_ref_ptr != nullptr, "spd_ref_ptr is nullptr");
  HW_ASSERT(spd_fdb_ptr != nullptr, "spd_fdb_ptr is nullptr");

  for (auto &model : wheel_motor_model_list_) {
    model.update(
        *spd_ref_ptr++,
        *spd_fdb_ptr++,
        i_ffd_ptr == nullptr ? 0 : *i_ffd_ptr++,
        i_fdb_ptr == nullptr ? 0 : *i_fdb_ptr++);
  }
}

void PowerLimiter::updateSteeringModel(const float *spd_fdb_ptr, const float *i_input_ptr, const float *i_fdb_ptr)
{
  HW_ASSERT(spd_fdb_ptr != nullptr, "spd_fdb_ptr is nullptr");
  HW_ASSERT(i_input_ptr != nullptr, "i_input_ptr is nullptr");

  for (auto &model : steering_motor_model_list_) {
    model.update(
        *spd_fdb_ptr++,
        *i_input_ptr++,
        i_fdb_ptr == nullptr ? 0 : *i_fdb_ptr++);
  }
}

void PowerLimiter::calc(const RuntimeParams &params, float *spd_limited, float *i_limited)
{
  runtime_params_ = params;

  /* 计算模型功率 */
  wheel_motor_datas_.p_model = 0;
  steering_motor_datas_.p_model = 0;
  datas_.p_model = static_params_.p_bias;

  for (auto &model : wheel_motor_model_list_) {
    wheel_motor_datas_.p_model += model.status().p_model;
    datas_.p_model += model.status().p_model;
  }
  for (auto &model : steering_motor_model_list_) {
    steering_motor_datas_.p_model += model.status().p_model;
    datas_.p_model += model.status().p_model;
  }

  /* 计算底盘期望功率 */
  datas_.p_ref = calcPref(params);

  /* 限制舵电机功率 */
  steering_motor_datas_.p_ref =
      (datas_.p_ref - static_params_.p_bias) * static_params_.p_steering_ratio;

  solveILimited(steering_motor_model_list_, steering_motor_datas_);

  /* 分配轮电机功率 */
  wheel_motor_datas_.p_ref = datas_.p_ref - static_params_.p_bias - steering_motor_datas_.p_limited ;

  /* 求解一元二次方程，计算速度削减系数 k_limit，并计算限制速度 */
  solveSpdLimited(wheel_motor_model_list_, wheel_motor_datas_);

  /* 计算底盘总功率 */
  datas_.k_limit = wheel_motor_datas_.k_limit;
  datas_.p_predict =
      wheel_motor_datas_.p_predict +
      steering_motor_datas_.p_predict +
      static_params_.p_bias;
  datas_.p_limited =
      wheel_motor_datas_.p_limited +
      steering_motor_datas_.p_limited +
      static_params_.p_bias;

  /* 返回限制结果 */
  if (spd_limited != nullptr) {
    for (auto &model : wheel_motor_model_list_) {
      *spd_limited++ = model.results().spd_limited;
    }
  }

  if (i_limited != nullptr) {
    for (auto &model : steering_motor_model_list_) {
      *i_limited++ = model.results().i_limited;
    }
  }
}

void PowerLimiter::solveSpdLimited(PowerModelList &power_model_list, Datas &datas)
{
  /* 计算限制前功率 */
  datas.p_predict = 0;
  for (auto &model : power_model_list) {
    datas.p_predict += model.status().p_predict;
  }

  /* 当预测功率小于动态P_ref时，不限制功率 */
  if (datas.p_predict < datas.p_ref) {
    datas.k_limit = 1.0f;
  } else {
    /* 计算一元二次方程的系数 */
    float alpha = 0, beta = 0, gamma = 0;
    for (auto &model : power_model_list) {
      /* 计算中间变量
       * i_pre = kp * (k_limit * spd_ref - spd_fdb) + i_ffd
       */
      float primary_coefficient =  // 一次系数
          model.params().kp * model.status().spd_ref;
      float zero_order_coefficient =  // 零次系数
          -model.params().kp * model.status().spd_fdb + model.status().i_ffd;

      /* 由功率公式 p_ref = sum(k1 * spd_fdb * i_pre  + k2 * i_pre^2)
       * 推导可得一元二次方程：
       * sum(k2 * primary^2) * k^2 +
       * sum(k1 * spd_fdb * primary + 2 * k2 * primary * zero_order) * k +
       * sum(k1 * spd_fdb * zero_order + k2 * zero_order^2 + k3 * fabsf(spd_fdb))- p_ref = 0
       */
      alpha += model.params().k2 * primary_coefficient * primary_coefficient;

      beta += model.params().k1 * model.status().spd_fdb * primary_coefficient +
              2 * model.params().k2 * primary_coefficient * zero_order_coefficient;

      gamma += model.params().k1 * model.status().spd_fdb * zero_order_coefficient +
               model.params().k2 * zero_order_coefficient * zero_order_coefficient +
               model.params().k3 * fabsf(model.status().spd_fdb);
    }
    gamma -= datas.p_ref;

    /* 求解速度削减系数 */
    datas.k_limit = solveQuadraticEquation(alpha, beta, gamma);
  }
  /* 计算限制后功率 */
  datas.p_limited = 0;
  for (auto &model : power_model_list) {
    datas.p_limited += model.limitBySpd(datas.k_limit);
  }
}

void PowerLimiter::solveILimited(PowerModelList &power_model_list, Datas &datas)
{
  /* 计算限制前功率 */
  datas.p_predict = 0;
  for (auto &model : power_model_list) {
    datas.p_predict += model.status().p_predict;
  }

  /* 当预测功率小于动态P_ref时，不限制功率 */
  if (datas.p_predict < datas.p_ref) {
    datas.k_limit = 1.0f;
  } else {
    /* 计算一元二次方程的系数 */
    float alpha = 0, beta = 0, gamma = 0;
    for (auto &model : power_model_list) {
      /* 对于作负功的电机，不限制其电流，负功大小绝对值记为 p_negative
       * 由公式 p_ref + p_negative = sum(k1 * spd_fdb * i_pre  + k2 * i_pre^2 + k3 * fabsf(spd_fdb))
       * 推导可得一元二次方程：
       * sum(k2) * k^2 +
       * sum(k1 * spd_fdb) * k +
       * sum(k3 * fabsf(spd_fdb))- p_negative - p_ref = 0
       */
      if (model.status().p_predict < 0) {  // 电机功率为负时，不限制电流
        gamma -= model.status().p_predict;
      } else {
        alpha += model.params().k2 * model.status().i_predict * model.status().i_predict;
        beta += model.params().k1 * model.status().spd_fdb * model.status().i_predict;
        gamma += model.params().k3 * fabsf(model.status().spd_fdb);
      }
    }
    gamma -= datas.p_ref;

    /* 求解电流削减系数 */
    datas.k_limit = solveQuadraticEquation(alpha, beta, gamma);
  }
  /* 计算限制后功率 */
  datas.p_limited = 0;
  for (auto &model : power_model_list) {
    if (model.status().p_predict < 0) {
      datas.p_limited += model.limitByCurrent(1.0);
    } else {
      datas.p_limited += model.limitByCurrent(datas.k_limit);
    }
  }
}

PowerModel &PowerLimiter::getWheelMotorModel(size_t idx)
{
  HW_ASSERT(idx < wheel_motor_model_list_.size(), "Illegal index %d", idx);
  auto iter = wheel_motor_model_list_.begin();
  for (size_t i = 0; i < idx; i++) {
    ++iter;
  }
  return *iter;
}

PowerModel &PowerLimiter::getSteeringMotorModel(size_t idx)
{
  HW_ASSERT(idx < steering_motor_model_list_.size(), "Illegal index %d", idx);
  auto iter = steering_motor_model_list_.begin();
  for (size_t i = 0; i < idx; i++) {
    ++iter;
  }
  return *iter;
}

#pragma endregion

/* Private function prototypes -----------------------------------------------*/

float PowerModel::calcCurrent(float spd_ref, float spd_fdb, float i_ffd)
{
  float current = params_.kp * (spd_ref - spd_fdb) + i_ffd;
  return Bound(current, -params_.out_limit, params_.out_limit);
}

float PowerModel::calcPower(float i, float spd_fdb)
{
  float power = params_.k1 * i * spd_fdb + params_.k2 * i * i + params_.k3 * fabsf(spd_fdb);
  return power;
}

float PowerLimiter::calcPref(const RuntimeParams &params)
{
  /* 危险能量下，p_ref 设为 0 */
  if (runtime_params_.remaining_energy < runtime_params_.danger_energy) {
    return 0;
  }

  /* p_ref 图像为过点(energy_converge, p_referee_max)，斜率为 p_slope 的直线 */
  float p_ref = runtime_params_.p_referee_max +
                runtime_params_.p_slope *
                    (runtime_params_.remaining_energy -
                     runtime_params_.energy_converge);

  /* 限幅 */
  p_ref = Bound(p_ref, runtime_params_.p_ref_min, runtime_params_.p_ref_max);
  return p_ref;
}

float PowerLimiter::solveQuadraticEquation(float alpha, float beta, float gamma)
{
  float k_limit = 0;
  if (alpha != 0) {
    float discriminant = beta * beta - 4 * alpha * gamma;
    if (discriminant > 0) {
      float sqrt_discriminant = 0;
      arm_sqrt_f32(discriminant, &sqrt_discriminant);
      k_limit = (-beta + sqrt_discriminant) / (2 * alpha);
    } else {
      k_limit = -beta / (2 * alpha);
    }
  } else if (beta != 0) {
    k_limit = (-gamma) / beta;
  } else {
    k_limit = 0;
  }
  return Bound(k_limit, 0, 1);
}

HW_OPTIMIZE_O2_END
}  // namespace power_limiter
}  // namespace hello_world