/**
 *******************************************************************************
 * @file      : fric_2motor.cpp
 * @brief     : 单级共速双摩擦轮模块，暂时供步兵、哨兵、无人机使用。
 * @history   :
 *  Version     Date            Author                       Note
 *  V0.9.0      2024.12.12      ZhouShichan, LouKaiyang      1. 初版编写完成
 *******************************************************************************
 * @attention : 后续可能推出 n 级 n 摩擦轮模块，级数和每一级的摩擦轮数量可以自定义，
 *              同时允许每个摩擦轮单独指定期望速度。
 *******************************************************************************
 *  Copyright (c) 2025 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "fric_2motor.hpp"

#include <cstring>
/* Private macro -------------------------------------------------------------*/
namespace hello_world
{
namespace module
{
namespace fric_impl
{
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
/* Private function definitions ----------------------------------------------*/
#pragma region 构造函数
Fric::Fric(const Config &config)
{
  setConfig(config);
  status_.normal_spd_ref = cfg_.default_spd_ref;
  /* 设置一个假的裁判系统反馈弹速初值，这样就不用对第一发弹之前的情况做特殊处理 */
  if (cfg_.opt_blt_spd_cl.is_enabled) {
    rfr_data_.bullet_spd = (cfg_.opt_blt_spd_cl.max_target_blt_spd +
                            cfg_.opt_blt_spd_cl.min_target_blt_spd) /
                           2.0f;
    rfr_data_.last_bullet_spd = rfr_data_.bullet_spd;
  }
}

void Fric::setConfig(const Config &config)
{
  HW_ASSERT(config.default_spd_ref > 0, "Default reference fric speed need to be greater than 0.");
  HW_ASSERT(config.default_spd_ref_backward < 0, "Default reference fric backward speed need to be less than 0.");
  HW_ASSERT(config.stuck_curr_thre > 0, "Current threshold when the fric is stuck need to be greater than 0.");
  HW_ASSERT(config.spd_delta_thre > 0, "Fric speed delta threshold need to be greater than 0.");
  HW_ASSERT(config.spd_err_thre > 0, "Fric speed error threshold need to be greater than 0.");
  HW_ASSERT(config.spd_stop_thre > 0, "Fric speed stop threshold need to be greater than 0.");
  HW_ASSERT(config.opt_blt_spd_cl.min_reasonable_blt_spd > 0, "Minimun reasonable bullet speed need to be greater than 0.");
  HW_ASSERT(config.opt_blt_spd_cl.max_reasonable_blt_spd > 0, "Maximun reasonable bullet speed need to be greater than 0.");
  HW_ASSERT(config.opt_blt_spd_cl.min_target_blt_spd > 0, "Minimun target bullet speed need to be greater than 0.");
  HW_ASSERT(config.opt_blt_spd_cl.max_target_blt_spd > 0, "Maximun target bullet speed need to be greater than 0.");
  HW_ASSERT(config.opt_blt_spd_cl.spd_gradient >= 0, "Speed gradient need to be greater than 0.");
  cfg_ = config;
  if (cfg_.opt_blt_spd_cl.min_reasonable_blt_spd > cfg_.opt_blt_spd_cl.max_reasonable_blt_spd) {
    float tmp = cfg_.opt_blt_spd_cl.min_reasonable_blt_spd;
    cfg_.opt_blt_spd_cl.min_reasonable_blt_spd = cfg_.opt_blt_spd_cl.max_reasonable_blt_spd;
    cfg_.opt_blt_spd_cl.max_reasonable_blt_spd = tmp;
  }
  if (cfg_.opt_blt_spd_cl.min_target_blt_spd > cfg_.opt_blt_spd_cl.max_target_blt_spd) {
    float tmp = cfg_.opt_blt_spd_cl.min_target_blt_spd;
    cfg_.opt_blt_spd_cl.min_target_blt_spd = cfg_.opt_blt_spd_cl.max_target_blt_spd;
    cfg_.opt_blt_spd_cl.max_target_blt_spd = tmp;
  }
}
#pragma endregion
#pragma region 数据更新

void Fric::update()
{
  updateData();
  updatePwrState();
  debug_pwr_state_ = getPwrState();
};

void Fric::updateData()
{
  updateWorkTick();
  updateMotorData();
};

void Fric::updatePwrState()
{
  /* 从电机状态获取电源状态，裁判系统发上来的信息可能因为固件版本等问题出现错误 */
  is_power_on_ = is_any_motor_pwr_on_;

  if (!is_power_on_) {
    setPwrState(PwrState::kDead);
    return;
  }

  PwrState current_state = pwr_state_;
  PwrState next_state = current_state;
  if (current_state == PwrState::kDead) {
    /* 死亡状态下，如果上电，则切到复活状态 */
    if (is_power_on_) {
      next_state = PwrState::kResurrection;
    }
  } else if (current_state == PwrState::kResurrection) {
    if (status_.is_ready) {
      next_state = PwrState::kWorking;
    }
  } else if (current_state == PwrState::kWorking) {
    /* 工作状态下，保持当前状态 */
  } else {
    /* 其他状态，认为是死亡状态 */
    next_state = PwrState::kDead;
  }
  setPwrState(next_state);
};

void Fric::updateMotorData()
{
  bool is_any_motor_pwr_on = false;
  Motor *motor_ptr = nullptr;

  for (size_t i = 0; i < 2; i++) {
    motor_ptr = motor_ptr_[i];
    HW_ASSERT(motor_ptr != nullptr, "ptr to fric motor %d is nullptr", i);
    status_.last_spd_fdb[i] = status_.spd_fdb[i];
    if (motor_ptr->isOffline()) {
      status_.spd_fdb[i] = 0;
      status_.cur_fdb[i] = 0;

      status_.resetSpdStatus();
      status_.resetStuckStatus();
    } else {
      status_.spd_fdb[i] = motor_ptr->vel();
      status_.cur_fdb[i] = motor_ptr->curr();

      is_any_motor_pwr_on = true;

      updateStuckStatus();
      updateSpdStatus();
    };
  }

  is_any_motor_pwr_on_ = is_any_motor_pwr_on;
};

/** 
 * @brief       更新裁判系统反馈数据
 * @param        &inp_data: 裁判系统反馈数据
 * @note        用户需要在外部调用该函数更新裁判系统数据
 */
void Fric::updateRfrData(const FricRfrInputData &inp_data)
{
  if (rfr_data_.bullet_spd != inp_data.bullet_spd) {
    /* 弹速需要限制在最小和最大值之间，超过此值意味着数据有误 */
    if (inp_data.bullet_spd >= cfg_.opt_blt_spd_cl.min_reasonable_blt_spd &&
        inp_data.bullet_spd <= cfg_.opt_blt_spd_cl.max_reasonable_blt_spd) {
      rfr_data_.last_bullet_spd = rfr_data_.bullet_spd;
      rfr_data_.bullet_spd = inp_data.bullet_spd;
    }
  }
  /* 更新当前数据 */
  rfr_data_.is_power_on = inp_data.is_power_on;
  rfr_data_.is_new_bullet_shot = inp_data.is_new_bullet_shot;
};

/** 
 * @brief       根据电机反馈电流，更新摩擦轮卡弹状态
 */
void Fric::updateStuckStatus()
{
  if (status_.cur_fdb[0] <= -cfg_.stuck_curr_thre ||
      status_.cur_fdb[1] <= -cfg_.stuck_curr_thre) {
    status_.stuck_duration += interval_ticks_;
    if (status_.stuck_duration >= 100) {
      status_.stuck_status = StuckStatus::kBackward;
    }
  } else if (status_.cur_fdb[0] >= cfg_.stuck_curr_thre ||
             status_.cur_fdb[1] >= cfg_.stuck_curr_thre) {
    status_.stuck_duration += interval_ticks_;
    if (status_.stuck_duration >= 100) {
      status_.stuck_status = StuckStatus::kForward;
    }
  } else {
    status_.stuck_duration = 0;
    status_.stuck_status = StuckStatus::kNone;
  }
};

/** 
 * @brief       更新摩擦轮电机速度状态
 * @note        该函数通过 摩擦轮速度波动、摩擦轮速度与期望值的误差 这两个值
 *              来判断摩擦轮是否已经准备好正常发弹
 */
void Fric::updateSpdStatus()
{
  bool hold_flag[2] = {false, false};

  for (size_t i = 0; i < 2; i++) {
    float spd_fdb_delta = status_.spd_fdb[i] - status_.last_spd_fdb[i];
    float spd_err = status_.actual_spd_ref - status_.spd_fdb[i];
    hold_flag[i] = fabsf(spd_fdb_delta) < cfg_.spd_delta_thre &&
                   fabsf(spd_err) < cfg_.spd_err_thre;
  }

  if (hold_flag[0] && hold_flag[1]) {
    status_.spd_ready_duration += interval_ticks_;
  } else {
    status_.spd_ready_duration = 0;
  }
};

#pragma endregion

#pragma region 执行任务
void Fric::runOnDead()
{
  resetDataOnDead();
};

void Fric::runOnResurrection()
{
  if (status_.spd_ready_duration > 200) {
    status_.is_ready = true;
  }
  calcSpdRef();
  calcMotorInput();
};

void Fric::runOnWorking()
{
  calcSpdRef();
  calcMotorInput();
};
void Fric::runAlways()
{
  setCommData((pwr_state_ != PwrState::kDead));
}
void Fric::standby()
{
  setWorkingMode(WorkingMode::kStop);
  calcSpdRef();
  calcMotorInput();
  setCommData(true);
};

void Fric::calcSpdRef()
{
  if (working_mode_ == WorkingMode::kStop || status_.stuck_status != StuckStatus::kNone) {
    /* 1. 停止模式下，摩擦轮停转；
       2. 当摩擦轮卡住时，摩擦轮停转，防止电机堵转并出现损坏 */
    status_.actual_spd_ref = 0.0f;
    return;
  } else if (working_mode_ == WorkingMode::kBackward) {
    /* 摩擦轮倒转模式是为了把卡住的弹丸往回推出，速度不宜过快 */
    status_.actual_spd_ref = cfg_.default_spd_ref_backward;
    return;
  }

  /* 开启了弹速闭环优化时，当有一颗新弹丸射出，且弹速超出期望阈值时，调整摩擦轮转速使弹速收敛 */
  if (pwr_state_ == PwrState::kWorking && cfg_.opt_blt_spd_cl.is_enabled &&
      rfr_data_.is_new_bullet_shot) {
    if (rfr_data_.bullet_spd > cfg_.opt_blt_spd_cl.max_target_blt_spd) {
      status_.normal_spd_ref -= cfg_.opt_blt_spd_cl.spd_gradient;
    } else if (rfr_data_.bullet_spd < cfg_.opt_blt_spd_cl.min_target_blt_spd) {
      status_.normal_spd_ref += cfg_.opt_blt_spd_cl.spd_gradient;
    }
  }
  status_.actual_spd_ref = status_.normal_spd_ref;
}

void Fric::calcMotorInput()
{
  Motor *motor_ptr = nullptr;
  Pid *pid_ptr = nullptr;

  for (size_t i = 0; i < 2; i++) {
    motor_ptr = motor_ptr_[i];
    pid_ptr = pid_ptr_[i];

    HW_ASSERT(motor_ptr != nullptr, "pointer to Fric %d Motor is nullptr", i);
    HW_ASSERT(pid_ptr_ != nullptr, "pointer to Fric %d PID is nullptr", i);

    /* 摩擦轮 Stop 模式，当反馈速度小于阈值时，停止控制电机。（允许用户上手转一下摩擦轮） 
    Q1：为什么需要等到反馈速度小于阈值才停止控制电机？
    A1：如果一切到 Stop 模式就直接停止控制电机，摩擦轮受惯性影响需要转很久才能停下来；
        所以最好是先控制速度为0，让摩擦轮转速快速下降；然后再停止控制电机。
    */
    if (working_mode_ == WorkingMode::kStop && fabsf(status_.spd_fdb[i]) < cfg_.spd_stop_thre) {
      status_.cur_ref[i] = 0.0f;
      continue;
    }

    if (motor_ptr->isOffline()) {
      status_.cur_ref[i] = 0.0f;
    } else {
      float ref = status_.actual_spd_ref;
      float fdb[2] = {status_.spd_fdb[i], status_.cur_fdb[i]};
      pid_ptr->calc(&ref, fdb, nullptr, &status_.cur_ref[i]);
    }
  }

  if (cfg_.opt_spd_same_pid_enabled) {
    HW_ASSERT(pid_ptr_[2] != nullptr, "Speed same Pid ptr is nullptr");
    float ref = 0;
    float fdb = motor_ptr_[0]->vel() - motor_ptr_[1]->vel();
    float out = 0;
    pid_ptr_[2]->calc(&ref, &fdb, nullptr, &out);
    status_.cur_ref[0] += out;
    status_.cur_ref[1] -= out;
  }
};
#pragma endregion

#pragma region 数据重置

/**
 * @brief 供外部调用的复位函数
 *
 * 会重置大部分数据，包括电源状态、控制模式、工作模式、电机指令等
 */
void Fric::reset()
{
  pwr_state_ = PwrState::kDead;
  last_pwr_state_ = PwrState::kDead;
  debug_pwr_state_ = PwrState::kDead;

  working_mode_ = WorkingMode::kStop;
  is_power_on_ = false;
  is_any_motor_pwr_on_ = false;
  resurrection_tick_ = 0;
  status_.is_ready = false;
  status_.resetSpdStatus();
  status_.resetStuckStatus();
  status_.actual_spd_ref = 0;
  status_.normal_spd_ref = cfg_.default_spd_ref;
  memset(status_.spd_fdb, 0, sizeof(status_.spd_fdb));
  memset(status_.cur_fdb, 0, sizeof(status_.cur_fdb));
  memset(status_.cur_ref, 0, sizeof(status_.cur_ref));

  rfr_data_.is_power_on = false;
  rfr_data_.is_new_bullet_shot = false;
  rfr_data_.bullet_spd = (cfg_.opt_blt_spd_cl.min_target_blt_spd + cfg_.opt_blt_spd_cl.max_target_blt_spd) / 2.0f;

  resetPids();
};

/**
 * @brief 复位数据(在死亡状态下)
 *
 * 仅重置部分数据，包括电源状态、电机指令等。
 * 不重置控制模式、工作模式等外部传递进来的指令。
 * 不重置由外部传递活内部通讯组件维护的状态数据。
 *
 * @note 该函数仅供内部调用，外部调用请使用 reset() 函数
 */
void Fric::resetDataOnDead()
{
  working_mode_ = WorkingMode::kStop;
  resurrection_tick_ = 0;
  status_.is_ready = false;
  status_.resetSpdStatus();
  status_.resetStuckStatus();
  status_.actual_spd_ref = 0;
  status_.normal_spd_ref = cfg_.default_spd_ref;
  memset(status_.cur_ref, 0, sizeof(status_.cur_ref));
  rfr_data_.is_power_on = false;
  rfr_data_.is_new_bullet_shot = false;
  rfr_data_.bullet_spd = (cfg_.opt_blt_spd_cl.min_target_blt_spd + cfg_.opt_blt_spd_cl.max_target_blt_spd) / 2.0f;

  resetPids();
};

/**
 * @brief 重置所有的PID控制器
 *
 * 这个函数遍历所有的PID控制器，如果控制器存在，就调用其reset方法进行重置。
 * 这个函数只会在 reset(), resetDataOnDead() 以及 runAlways() / setCommData(false) 中被调用.
 * @note 这个函数不会创建或删除任何PID控制器，只是重置它们的状态。
 */
void Fric::resetPids()
{
  for (size_t i = 0; i < 2; i++) {
    Pid *pid_ptr = pid_ptr_[i];
    if (pid_ptr != nullptr) {
      pid_ptr->reset();
    }
  }
};

#pragma endregion

#pragma region 设置通信组件数据

/**
 * @brief 设置通信数据
 *
 * 这个函数设置摩擦轮和弹仓电机的电流期望值。如果电机在线，就设置电流期望值为计算得到的值；
 * 如果电机离线，就设置电流期望值为0。
 *
 * @note 这个函数不会创建或删除任何电机或离线检查器，只是设置它们的状态。
 */
void Fric::setCommData(bool is_working)
{
  Motor *motor_ptr = nullptr;
  Pid *pid_ptr = nullptr;
  for (size_t i = 0; i < 2; i++) {
    motor_ptr = motor_ptr_[i];
    pid_ptr = pid_ptr_[i];
    HW_ASSERT(motor_ptr != nullptr, "ptr to fric motor %d is nullptr", i);
    if ((!motor_ptr->isOffline()) && is_working) {
      motor_ptr->setInput(status_.cur_ref[i]);
    } else {
      pid_ptr->reset();
      motor_ptr->setInput(0);
    };
  }
};

#pragma endregion

#pragma region 注册函数

void Fric::registerMotor(Motor *ptr, MotorIdx idx)
{
  HW_ASSERT(ptr != nullptr, "pointer to motor %d is nullptr", static_cast<uint8_t>(idx));
  HW_ASSERT(idx == MotorIdx::kFirst || idx == MotorIdx::kSecond, "Idx %d is meaningless", static_cast<uint8_t>(idx));
  motor_ptr_[static_cast<uint8_t>(idx)] = ptr;
};
void Fric::registerPid(Pid *ptr, PidIdx idx)
{
  HW_ASSERT(ptr != nullptr, "pointer to PID %d is nullptr", static_cast<uint8_t>(idx));
  HW_ASSERT(idx == PidIdx::kFirst || idx == PidIdx::kSecond || idx == PidIdx::kSameSpd, "Idx %d is meaningless", static_cast<uint8_t>(idx));
  pid_ptr_[static_cast<uint8_t>(idx)] = ptr;
};
#pragma endregion

}  // namespace fric_impl
}  // namespace module
}  // namespace hello_world