/**
 *******************************************************************************
 * @file      :power_limiter.hpp
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_ALGORITHMS_POWER_LIMITER_HPP_
#define HW_COMPONENTS_ALGORITHMS_POWER_LIMITER_HPP_
/* Includes ------------------------------------------------------------------*/
#include "allocator.hpp"
#include "list.hpp"
#include "system.hpp"

namespace hello_world
{
namespace power_limiter
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

HW_OPTIMIZE_O2_START

/* 模型参数 */
struct PowerModelParams {
  float k1;         ///< Iω 项系数
  float k2;         ///< I^2 项系数
  float k3;         ///< fabsf(ω) 项系数
  float kp;         ///< 速度环比例系数
  float out_limit;  ///< 输出限幅，单位：A
};

/* 运行状态 */
struct PowerModelStatus {
  float spd_ref;    ///< 电机期望转速，单位：rad/s
  float spd_fdb;    ///< 电机反馈转速，单位：rad/s
  float i_ffd;      ///< 前馈电流，单位：A
  float i_fdb;      ///< 电机反馈的实际电流，单位：A
  float i_predict;  ///< 预测电流（限制前），单位：A
  float p_model;    ///< 模型功率，单位：W
  float p_predict;  ///< 预测功率（限制前），单位：W
};

/* 计算结果 */
struct PowerModelResults {
  float k_limit;      ///< 削减系数，范围：[0, 1]
  float i_limited;    ///< 输出电流（限制后），单位：A
  float spd_limited;  ///< 期望速度（限制后），单位：rad/s
  float p_limited;    ///< 输出功率（限制后），单位：W
};

/* 单个电机的功率模型 */
class PowerModel : public MemMgr
{
 public:
  typedef PowerModelParams Params;
  typedef PowerModelStatus Status;
  typedef PowerModelResults Results;

  PowerModel() = default;
  PowerModel(const Params &params);
  PowerModel(const PowerModel &) = default;
  PowerModel &operator=(const PowerModel &) = default;
  PowerModel(PowerModel &&) = default;
  PowerModel &operator=(PowerModel &&) = default;
  virtual ~PowerModel() = default;

  /* 配置方法 */

  void init(const Params &params);

  void setParams(const Params &params);

  void setParams(float k1, float k2, float k3);

  /* 计算接口 */

  /**
   * @brief      根据反馈转速和输入电流计算单个电机输出功率
   * @param       spd_fdb: 电机反馈转速，单位：rad/s
   * @param       i_input: 输入电流，单位：A
   * @param       i_fdb: 电机反馈的实际电流，单位：A
   * @retval      p_predict: 电机预测功率（限制前），单位：W
   * @note       主要用于舵电机
   */
  float update(float spd_fdb, float i_input, float i_fdb);

  /**
   * @brief      根据速度误差和前馈计算单个电机输出功率
   * @param       spd_ref: 电机期望转速，单位：rad/s
   * @param       spd_fdb: 电机反馈转速，单位：rad/s
   * @param       i_ffd: 前馈电流，单位：A
   * @param       i_fdb: 电机反馈的实际电流，单位：A
   * @retval      p_predict: 输出功率（限制后），单位：W
   * @note       主要用于轮电机
   */
  float update(float spd_ref, float spd_fdb, float i_ffd, float i_fdb);

  /**
   * @brief      计算限制结果
   * @param       k_limit: 速度削减系数，范围：[0, 1]
   * @retval      p_limited_: 输出电流（限制后），单位：A
   * @note       需要事先调用 update(float spd_ref, float spd_fdb, float i_ffd) 进行更新
   *             该函数在 PowerLimiter 内部调用，也可在外部自行调试
   */
  float limitBySpd(float k_limit);

  /**
   * @brief      计算限制结果
   * @param       k_limit: 电流削减系数，范围：[0, 1]
   * @retval      p_limited_: 输出电流（限制后），单位：A
   * @note       需要事先调用 update(float i_input) 进行更新
   *             该函数在 PowerLimiter 内部调用，也可在外部自行调试
   */
  float limitByCurrent(float k_limit);

  /* 数据获取 */

  const Params &params(void) const { return params_; }

  const Status &status(void) const { return status_; }

  const Results &results(void) const { return results_; }

 private:
  /**
   * @brief      计算电流预测值
   * @param       spd_ref: 电机期望转速，单位：rad/s
   * @param       spd_fdb: 电机反馈转速，单位：rad/s
   * @param       i_ffd: 前馈电流，单位：A
   * @retval      i_predict: 电流预测值，单位：A
   * @note        会根据output_limit进行限幅
   */
  float calcCurrent(float spd_ref, float spd_fdb, float i_ffd);

  /**
   * @brief      计算电机输出功率
   * @param       i: 电机输出电流，单位：A
   * @param       spd_fdb: 电机反馈转速，单位：rad/s
   * @retval      power: 电机输出功率，单位：W
   * @note        None
   */
  float calcPower(float i, float spd_fdb);

  /* 模型参数 */
  Params params_;

  /* 运行状态 */
  Status status_;

  /* 限制结果 */
  Results results_;
};

/* 功率限制静态参数 */
struct PowerLimiterStaticParams {
  struct WheelMotorParams {
    float k1;            ///< Iω 项系数
    float k2;            ///< I^2 项系数
    float k3;            ///< fabsf(ω) 项系数
    float kp;            ///< 速度环比例系数
    float out_limit;     ///< 输出限幅，单位：A
    size_t motor_cnt;    ///< 电机数量
  } wheel_motor_params;  ///< 轮电机功率模型参数
  struct SteeringMotorParams {
    float k1;               ///< Iω 项系数
    float k2;               ///< I^2 项系数
    float k3;               ///< fabsf(ω) 项系数
    float out_limit;        ///< 输出限幅，单位：A
    size_t motor_cnt;       ///< 电机数量
  } steering_motor_params;  ///< 舵电机功率模型参数

  float p_bias;            ///< 底盘静息功率，单位：W
  float p_steering_ratio;  ///< 分配给舵电机的功率比例，非舵轮底盘为 0，范围：[0, 1]
};

/* 功率限制运行时参数 */
struct PowerLimiterRuntimeParams {
  /* 期望功率上限，可以根据模式设置（低速/高速），单位：W */
  float p_ref_max;
  /* 裁判系统给出的底盘功率限制，单位：W */
  float p_referee_max;
  /* 期望功率下限，建议设为当前 p_referee_max 乘一个 0 ~ 1 的比例系数，单位：W */
  float p_ref_min;
  /**
   * 剩余能量：
   * 采用裁判系统控制时为裁判系统缓冲能量 buffer_energy，单位：J
   * 采用超级电容控制时为剩余能量百分比 supercap_remaining_power，范围：[0,100]，单位：%
   */
  float remaining_energy;
  /* 剩余能量收敛值，当剩余能量到达该值时，期望功率等于裁判系统给出的底盘功率上限 p_referee_max */
  float energy_converge;
  /* 期望功率随当前剩余能量线性变化的斜率 */
  float p_slope;
  /* 危险能量值，若剩余能量低于该值，p_ref_ 设为 0 */
  float danger_energy;
};

/* 功率限制计算结果 */
struct PowerLimiterDatas {
  float p_ref;      ///< 期望功率，单位：W
  float p_model;    ///< 模型功率，单位：W
  float p_predict;  ///< 预测功率（限制前），单位：W
  float p_limited;  ///< 输出功率（限制后），单位：W
  float k_limit;    ///< 削减系数，范围：[0, 1]
};

class PowerLimiter : public MemMgr
{
 public:
  typedef PowerLimiterStaticParams StaticParams;
  typedef PowerLimiterRuntimeParams RuntimeParams;
  typedef PowerLimiterDatas Datas;
  typedef tools::list<PowerModel> PowerModelList;

  /**
   * @brief       构造函数
   * @param        static_param: 功率限制静态参数
   * @retval       None
   * @note        None
   */
  PowerLimiter(const StaticParams &static_params);

  ~PowerLimiter() = default;

  /**
   * @brief      更新轮电机功率模型
   * @param       spd_ref_ptr: 期望速度数组指针，单位：rad/s
   * @param       spd_fdb_ptr: 反馈速度数组指针，单位：rad/s
   * @param       i_fdb_ptr: 反馈电流数组指针，单位：Aw
   * @param       i_ffd_ptr: 前馈电流数组指针，单位：A
   * @retval      None
   * @note        数组指针大小需与轮电机功率模型数量一致
   */
  void updateWheelModel(
      const float *spd_ref_ptr,
      const float *spd_fdb_ptr,
      const float *i_ffd_ptr = nullptr,
      const float *i_fdb_ptr = nullptr);

  /**
   * @brief      更新舵电机功率模型
   * @param       spd_fdb_ptr: 反馈速度数组指针，单位：rad/s
   * @param       i_input_ptr: 输入电流数组指针，单位：A
   * @param       i_fdb_ptr: 反馈电流数组指针，单位：A
   * @retval      None
   * @note        数组指针大小需与舵电机功率模型数量一致
   */
  void updateSteeringModel(
      const float *spd_fdb_ptr,
      const float *i_input_ptr,
      const float *i_fdb_ptr = nullptr);

  /**
   * @brief      根据动态参数计算功率限制
   * @param       &params: 功率限制运行时参数
   * @retval      None
   * @note       需要先更新电机模型
   */
  void calc(const RuntimeParams &params, float *spd_limited = nullptr, float *i_limited = nullptr);

  /**
   * @brief      根据给定期望功率 p_ref 计算速度削减系数 k_limit，并计算限制速度
   * @param       power_model_list: 电机功率模型列表
   * @param       datas: 电机功率计算数据（需要输入p_ref）
   * @retval      None
   * @note       在 calc() 内部调用，用户也可在外部自定义功率限制逻辑，调用该函数求解
   */
  void solveSpdLimited(PowerModelList &power_model_list, Datas &datas);

  /**
   * @brief      根据给定期望功率 p_ref 计算电流削减系数 k_limit，并计算限制电流
   * @param       power_model_list: 电机功率模型列表
   * @param       datas: 电机功率计算数据（需要输入p_ref）
   * @retval      None
   * @note       在 calc() 内部调用，用户也可在外部自定义功率限制逻辑，调用该函数求解
   */
  void solveILimited(PowerModelList &power_model_list, Datas &datas);

  /* 数据获取 */

  /**
   * @brief      获取轮电机功率模型
   * @param       idx: 轮电机功率模型索引
   * @retval      PowerModel: 轮电机功率模型
   * @note        索引按照注册顺序依次排列
   */
  PowerModel &getWheelMotorModel(size_t idx);

  /**
   * @brief      获取舵电机功率模型
   * @param       idx: 舵电机功率模型索引
   * @retval      PowerModel: 舵电机功率模型
   * @note        索引按照注册顺序依次排列
   */
  PowerModel &getSteeringMotorModel(size_t idx);

  const Datas &datas(void) const { return datas_; }

  const Datas &wheel_motor_datas(void) const { return wheel_motor_datas_; }

  const Datas &steering_motor_datas(void) const { return steering_motor_datas_; }

 private:
  /**
   * @brief      根据剩余能量大小，计算底盘期望功率
   * @retval      p_ref: 底盘期望功率，单位：W，范围：[p_referee_max * p_min_pct, p_referee_max]
   * @note        在 calc() 内部调用
   */
  float calcPref(const RuntimeParams &params);

  /**
   * @brief      求解一元二次方程
   * @param       alpha: 二次项系数
   * @param       beta: 一次项系数
   * @param       gamma: 常数项
   * @retval      k_limit: 削减系数
   * @note        默认alpha > 0
   *              存在两个解时，选择较大解
   *              无解时，返回对称轴
   *              alpha, beta 均为 0 时，返回 0
   */
  float solveQuadraticEquation(float alpha, float beta, float gamma);

  Datas datas_;                               ///< 底盘总功率计算数据，k_limit为轮电机速度削减系数
  Datas wheel_motor_datas_;                   ///< 轮电机功率计算数据，k_limit为速度削减系数
  Datas steering_motor_datas_;                ///< 舵电机功率计算数据，k_limit为电流削减系数
  StaticParams static_params_;                ///< 静态参数
  RuntimeParams runtime_params_;              ///< 运行时参数
  PowerModelList wheel_motor_model_list_;     ///< 轮电机功率模型列表
  PowerModelList steering_motor_model_list_;  ///< 舵电机功率模型列表
};

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace power_limiter
}  // namespace hello_world
#endif /* HW_COMPONENTS_ALGORITHMS_POWER_LIMITER_HPP_ */
