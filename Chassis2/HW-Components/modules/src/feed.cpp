/** 
 *******************************************************************************
 * @file      : feed.cpp
 * @brief     : 拨弹盘模块组件
 * @history   :
 *  Version     Date            Author                         Note
 *  V0.9.0      2024-12-12      ZhouShichan, LouKaiyang        1. 初版编写完成
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "feed.hpp"
/* Private macro -------------------------------------------------------------*/
namespace hello_world
{
namespace module
{
namespace feed_impl
{
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
#pragma region 构造函数
Feed::Feed(const Config &config)
{
  setConfig(config);
  trigger_interval_ = config.default_trigger_interval;
  safe_num_bullet_ = config.default_safe_num_blt;
}

void Feed::setConfig(const Config &config)  // 设定配置
{
  HW_ASSERT(config.ang_ref_offset >= 0, "ang_ref_offset must be not less than 0");
  HW_ASSERT(config.ang_per_blt > 0, "ang_per_blt must be greater than 0");
  HW_ASSERT(config.heat_per_blt > 0, "heat_per_blt must be greater than 0");
  HW_ASSERT(config.stuck_curr_thre > 0, "stuck_curr_thre must be greater than 0");
  HW_ASSERT(config.resurrection_pos_err > 0, "resurrection_pos_err must be greater than 0");
  HW_ASSERT(config.stuck_duration_thre > 0, "stuck_duration_thre must be greater than 0");
  HW_ASSERT(config.hold_duration_thre > 0, "hold_duration_thre must greater than 0");
  HW_ASSERT(config.default_trigger_interval > 0, "default_trigger_interval must be greater than 0");
  HW_ASSERT(config.default_safe_num_blt >= 0, "default_safe_num_blt must be not less than 0");
  cfg_ = config;
}
#pragma endregion
#pragma region 数据更新
void Feed::update()
{
  updateData();
  updatePwrState();
  debug_pwr_state_ = getPwrState();
};

void Feed::updateData()
{
  updateWorkTick();
  updateMotorData();
  updateFakeHeat();
};

void Feed::updatePwrState()
{
  if (!is_power_on_ || !is_fric_ok_) {
    setPwrState(PwrState::kDead);
    return;
  }

  PwrState current_state = pwr_state_;
  PwrState next_state = current_state;
  if (current_state == PwrState::kDead) {
    // 死亡状态下，如果上电，则切到复活状态
    if (is_power_on_) {
      next_state = PwrState::kResurrection;
    }
  } else if (current_state == PwrState::kResurrection) {
    // 复活状态下，如果摩擦轮准备就绪且拨盘准备就绪，则切到工作状态
    // Q1: 为什么要判断摩擦轮是否准备就绪？
    // A1: 1.) 摩擦轮上电控制时瞬时电流很大；
    //     2.) 需要摩擦轮正常启动之后才能启动拨盘，避免弹丸卡在摩擦轮中，导致摩擦轮堵转
    // Q2: 为什么要判断拨盘是否准备就绪？
    // A2: 复活模式下需要根据拨盘反馈角度，设置拨盘的目标角度；如果拨盘未上电，拨盘反馈角度为 0 ，
    //     使用此值来计算目标角度会导致拨盘控制出错，可能导致超发、连发等问题；
    //     如果拨盘为非绝对编码，需要将拨盘复位（反向转直到电机堵转，并重置电机零点）
    if (status_.is_ang_zero_inited && is_fric_ok_) {
      next_state = PwrState::kWorking;
    }
  } else if (current_state == PwrState::kWorking) {
    // 工作状态下，保持当前状态
  } else {
    // 其他状态，认为是死亡状态
    next_state = PwrState::kDead;
  }
  setPwrState(next_state);
};
/** 
 * @brief       更新拨弹盘电机在线状态、反馈数据、保持状态和卡弹状态。
 */
void Feed::updateMotorData()
{
  bool is_motor_pwr_on = false;
  HW_ASSERT(motor_ptr_ != nullptr, "ptr to feed motor is nullptr", motor_ptr_);
  status_.last_ang_fdb = status_.ang_fdb;
  if (motor_ptr_->isOffline()) {
    status_.ang_fdb = 0;
    status_.spd_fdb = 0;
    status_.cur_fdb = 0;

    status_.resetHoldStatus();
    status_.resetStuckStatus();
  } else {
    status_.ang_fdb = motor_ptr_->angle();
    status_.spd_fdb = motor_ptr_->vel();
    status_.cur_fdb = motor_ptr_->curr();

    is_motor_pwr_on = true;

    updateMotorHoldStatus();
    updateMotorStuckStatus();
  }
  // 从电机状态获取电源状态，裁判系统发来的信息可能因为固件版本等问题出现错误
  is_power_on_ = is_motor_pwr_on;
};

/** 
 * @brief       判断拨弹盘是否与上一周期保持同一位置
 * @note        ang_hold_duration 只在复活模式下使用, 详见 calcAngRefOnResurrection()
 */
void Feed::updateMotorHoldStatus()
{
  float fdb_delta = AngleNormRad(status_.ang_fdb - status_.last_ang_fdb);
  if (fabsf(fdb_delta) < 0.001) {
    status_.ang_hold_duration += interval_ticks_;
  } else {
    status_.ang_hold_duration = 0;
  }
};

/** 
 * @brief       更新电机卡弹状态
 * @note        根据电机反馈电流是否持续超阈值，判断电机是否卡弹
 */
void Feed::updateMotorStuckStatus()
{
  if (status_.cur_fdb <= -cfg_.stuck_curr_thre) {
    status_.stuck_duration += interval_ticks_;
    if (status_.stuck_duration >= cfg_.stuck_duration_thre) {
      status_.stuck_status = StuckStatus::kBackward;
      stuck_backward_cnt_++;
    }
  } else if (status_.cur_fdb >= cfg_.stuck_curr_thre) {
    status_.stuck_duration += interval_ticks_;
    if (status_.stuck_duration >= cfg_.stuck_duration_thre) {
      status_.stuck_status = StuckStatus::kForward;
      stuck_forward_cnt_++;
    }
  } else {
    status_.stuck_duration = 0;
    status_.stuck_status = StuckStatus::kNone;
  }
};

/** 
 * @brief       更新拨弹盘所需的裁判系统数据
 * @param        &inp_data: 结构体引用，离线采用默认值或上一次有效值
 * @note        需在Robot中调用
 */
void Feed::updateRfrData(const RfrInputData &inp_data)
{
  // 记录上一不同的数据
  if (rfr_data_.heat != inp_data.heat) {
    rfr_data_.last_heat_ = rfr_data_.heat;
  }

  // 更新当前数据
  rfr_data_.is_rfr_on = inp_data.is_rfr_on;
  rfr_data_.is_power_on = inp_data.is_power_on;
  rfr_data_.is_new_bullet_shot = inp_data.is_new_bullet_shot;
  rfr_data_.heat_limit = inp_data.heat_limit;
  rfr_data_.heat = inp_data.heat;
  rfr_data_.heat_cooling_ps = inp_data.heat_cooling_ps;
};

/** 
 * @brief      假热量冷却
 * @note       在 update() 中调用 
 */
void Feed::updateFakeHeat()
{
  if (fake_heat_ > 0) {
    fake_heat_ -= rfr_data_.heat_cooling_ps * interval_ticks_ * 0.001f;
    fake_heat_ = std::max(fake_heat_, 0.0f);
  }
}
#pragma endregion

#pragma region 执行任务

void Feed::runOnDead()
{
  resetDataOnDead();
};

void Feed::runOnResurrection()
{
  calcAngRefOnResurrection();
  calcMotorInput();
};

void Feed::runOnWorking()
{
  genTriggerSignal();
  calcFakeHeat();
  calcAngRefOnWorking();
  calcMotorInput();
};

void Feed::runAlways()
{
  setCommData((pwr_state_ != PwrState::kDead));
}

/** 
 * @brief       拨弹盘待机
 * @note        不控制电机, 几乎不更改任何数据
 */
void Feed::standby()
{
  // 重置保持原位状态与卡弹状态
  status_.resetHoldStatus();
  status_.resetStuckStatus();
  // 为了实现拨弹盘退出待机时的无扰切换
  status_.ang_ref = status_.ang_fdb;
  status_.is_ang_ref_on_grid = false;
  status_.cur_ref = 0;

  setCommData(false);
};

/** 
 * @brief       生成拨弹信号 
 * @note        直接更改trigger_signal_
 */
void Feed::genTriggerSignal()
{
  // 前向卡弹或用户不允许拨弹时不能生成拨弹信号
  if (status_.stuck_status == StuckStatus::kForward || !is_trigger_allowed_) {
    trigger_signal_ = false;
    return;
  }
  // 根据控制模式获取对应的射击信号
  bool shoot_flag = false;
  if (ctrl_mode_ == CtrlMode::kManual) {
    shoot_flag = manual_shoot_flag_;
  } else if (ctrl_mode_ == CtrlMode::kAuto) {
    shoot_flag =
        ((vision_shoot_flag_ == ShootFlag::kShootOnce && last_vision_shoot_flag_ == ShootFlag::kNoShoot) ||
         (vision_shoot_flag_ == ShootFlag::kShootContinuous) ||
         manual_shoot_flag_);
  }
  // 复位射击指令，防止用户停止更新后仍持续发弹
  manual_shoot_flag_ = false;
  vision_shoot_flag_ = ShootFlag::kNoShoot;

  // 根据射击信号、频率限制、热量限制决定能否生成拨弹信号
  trigger_signal_ = shoot_flag && limitTriggerByFreq(trigger_interval_);
  trigger_signal_ = trigger_signal_ && limitTriggerByHeat(is_heat_limited_, safe_num_bullet_);

  // 若成功生成拨弹信号，则更新'上一次拨弹时间'和拨弹计数器
  if (trigger_signal_) {
    last_trigger_tick_ = work_tick_;
    trigger_cnt_++;
  }
}

/** 
 * @brief       用拨弹频率限制拨弹
 * @param       interval: 允许拨弹时间间隔
 * @retval      是否允许拨弹
 */
bool Feed::limitTriggerByFreq(uint32_t interval) const { return (work_tick_ - last_trigger_tick_) > interval; };

/** 
 * @brief       用热量限制拨弹
 * @param       is_heat_limited:        是否限制热量
 * @param       safe_num: 安全热量值对应的弹丸个数
 * @retval      是否允许拨弹
 * @note        若当前再发射 safe_num 颗弹丸会超热量, 则返回 false
 */
bool Feed::limitTriggerByHeat(bool is_heat_limited, float safe_num) const
{
  if (!is_heat_limited) {
    return true;
  }
  float heat = rfr_data_.heat > fake_heat_ ? rfr_data_.heat : fake_heat_;
  float num_allowed_trigger = (rfr_data_.heat_limit - heat) / cfg_.heat_per_blt;
  return num_allowed_trigger >= safe_num;
};

void Feed::calcFakeHeat()
{
  if (rfr_data_.is_rfr_on) {
    // 裁判系统在线，以裁判系统的发弹数据包是否更新判断有无发弹，以此维护假热量
    // 此处假热量的作用是防止裁判系统热量更新延迟导致意外超热量死亡
    fake_heat_ += cfg_.heat_per_blt * static_cast<float>(rfr_data_.is_new_bullet_shot);
  } else {
    // 裁判系统离线，用拨弹盘是否拨动维护假热量
    // 此处假热量的作用是完全替代裁判系统热量
    // 双发会导致 fake_heat_ 偏低，空拨会导致 fake_heat_ 偏高，暂时没有好的处理方法
    // 发生卡弹，热量回退
    if (status_.stuck_status == StuckStatus::kForward) {
      fake_heat_ = std::max(fake_heat_ - cfg_.heat_per_blt, 0.0f);
    }
    fake_heat_ += cfg_.heat_per_blt * static_cast<float>(trigger_signal_);
  }
}

void Feed::calcAngRefOnResurrection()
{
  HW_ASSERT(motor_ptr_ != nullptr, "pointer to Feed Motor is nullptr", motor_ptr_);

  // 当拨盘初始化之后哪怕再进入该函数也直接退出
  if (status_.is_ang_zero_inited) {
    return;
  }

  // 拨盘以缓慢的速度反向旋转，直至接触到限位【期望状态】或因为其他原因卡住【意外状态，无法检测】
  // 假设现在所有的拨盘电机都不是多圈绝对双编码的，需要一个复位操作
  // 理想状态是将限位设置为 0°
  status_.ang_ref = AngleNormRad(status_.ang_fdb - cfg_.resurrection_pos_err);

  // 以下状态不会保持(每个控制周期都会根据反馈值重新判断)
  // 反向接触到限位可能有两种表现情况：
  // 1. 电流较大，出现堵转现象并被程序发现
  // 2. 电流较小，反馈角度长时间没有变化
  if (status_.stuck_status == StuckStatus::kBackward || status_.ang_hold_duration > cfg_.hold_duration_thre) {
    motor_ptr_->setAngleValue(0.0f);  // 设定电机零位
    status_.is_ang_zero_inited = true;
    status_.ang_ref = cfg_.ang_ref_offset;  // 设定拨盘初始期望角度
    status_.is_ang_ref_on_grid = true;

    status_.resetStuckStatus();
    return;
  }
};

void Feed::calcAngRefOnWorking()
{
  if (status_.stuck_status == StuckStatus::kForward) {
    // 拨盘前向堵转时，回退目标角度
    status_.ang_ref = AngleNormRad(status_.ang_ref - cfg_.ang_per_blt);
    // 清空堵转计时器，给电机充分时间回退
    // 清除前向堵转标志位，避免连续回退
    status_.resetStuckStatus();
    return;
  }

  if (status_.stuck_status == StuckStatus::kBackward) {
    // 拨盘后向堵转时，将目标角度设置为当前反馈角度略微往前一点
    status_.ang_ref = AngleNormRad(status_.ang_fdb + 0.01f);
    // 清空堵转计时器，给电机充分时间前向转动
    // 清除前向堵转标志位，避免连续回退
    status_.resetStuckStatus();
    status_.is_ang_ref_on_grid = false;
    return;
  }

  // 非堵转情况下，才会生成trigger_signal_
  if (trigger_signal_) {
    if (status_.is_ang_ref_on_grid == false) {
      status_.ang_ref = searchAngRef(status_.ang_fdb, true);
      status_.is_ang_ref_on_grid = true;
    } else {
      // 触发信号有效时，计算目标角度
      status_.ang_ref = AngleNormRad(status_.ang_ref + cfg_.ang_per_blt);
    }
  }
};

void Feed::calcMotorInput()
{
  // 拨弹电机
  HW_ASSERT(motor_ptr_ != nullptr, "pointer to Feed Motor is nullptr", motor_ptr_);
  HW_ASSERT(pid_ptr_ != nullptr, "pointer to Feed PID is nullptr", pid_ptr_);

  if (motor_ptr_->isOffline()) {
    status_.cur_ref = 0.0f;
  } else {
    float ref = status_.ang_ref;
    float fdb[2] = {status_.ang_fdb, status_.spd_fdb};
    pid_ptr_->calc(&ref, fdb, nullptr, &status_.cur_ref);
  }
};
#pragma endregion

#pragma region 数据重置
/** 
 * @brief       供外部调用的复位函数
 * @note        会重置几乎所有数据，包括电源状态、控制模式、工作模式、电机指令等
 */
void Feed::reset()
{
  pwr_state_ = PwrState::kDead;
  last_pwr_state_ = PwrState::kDead;
  debug_pwr_state_ = PwrState::kDead;

  ctrl_mode_ = CtrlMode::kManual;
  manual_shoot_flag_ = false;
  last_vision_shoot_flag_ = ShootFlag::kNoShoot;
  vision_shoot_flag_ = ShootFlag::kNoShoot;
  trigger_interval_ = cfg_.default_trigger_interval;
  safe_num_bullet_ = cfg_.default_safe_num_blt;
  is_heat_limited_ = true;
  is_trigger_allowed_ = true;

  is_fric_ok_ = false;

  is_power_on_ = false;
  fake_heat_ = 0.0f;

  trigger_signal_ = false;
  last_trigger_tick_ = 0;

  status_.is_ang_zero_inited = false;
  status_.is_ang_ref_on_grid = false;
  status_.resetHoldStatus();
  status_.resetStuckStatus();

  status_.ang_ref = 0;
  status_.ang_fdb = 0;
  status_.last_ang_fdb = 0;
  status_.spd_fdb = 0;
  status_.cur_fdb = 0;
  status_.cur_ref = 0;

  trigger_cnt_ = 0;
  stuck_forward_cnt_ = 0;
  stuck_backward_cnt_ = 0;

  resetPids();
};

/** 
 * @brief 复位数据(在死亡状态下)
 * 
 * 仅重置部分数据，包括电源状态、电机指令等。
 * 不重置控制模式、工作模式等外部传递进来的指令。
 * 不重置由外部传递和内部通讯组件维护的状态数据。
 * 
 * @note 该函数仅供内部调用，外部调用请使用 reset() 函数
 */
void Feed::resetDataOnDead()
{
  // 何时会进入 Dead 模式?
  // 1. 首次上电; 2. 机器人阵亡导致发射机构断电; 3. 允许发弹量为 0 导致发射机构断电;
  // 4. 拨盘电机离线
  // 对于情况 1, 2, 应当清零假热量; 但对于 3, 4, 应当使假热量自然冷却.
  // 考虑到情况 3, 4 一旦出现，持续时间相对较长, 应足以使热量冷却至 0;
  // 即使未冷却至 0, 也有裁判系统反馈的热量信息兜底, 因此统一在 Dead 模式清零假热量.
  // (不考虑未冷却至 0 且裁判系统离线的极端情况)
  fake_heat_ = 0.0f;

  trigger_signal_ = false;
  last_trigger_tick_ = 0;

  status_.is_ang_zero_inited = false;
  status_.is_ang_ref_on_grid = false;
  status_.resetHoldStatus();
  status_.resetStuckStatus();

  status_.ang_ref = 0.0f;
  status_.cur_ref = 0.0f;

  resetPids();
};

/**
 * @brief 重置所有的PID控制器.
 * 
 * 这个函数遍历所有的PID控制器, 如果控制器存在, 就调用其reset方法进行重置.
 * 这个函数只会在 reset(), resetDataOnDead() 以及 runAlways() / setCommData(false) 中被调用.
 * @note 这个函数不会创建或删除任何PID控制器, 只是重置它们的状态.
 */
void Feed::resetPids()
{
  HW_ASSERT(pid_ptr_ != nullptr, "pointer to Feed PID is nullptr", pid_ptr_);
  pid_ptr_->reset();
};
#pragma endregion

#pragma region 通信数据设置函数

/**
 * @brief 设置通信数据
 * 
 * 这个函数设置拨弹电机的电流期望值. 如果电机在线, 就设置电流期望值为计算得到的值;
 * 如果电机离线, 就设置电流期望值为0, 同时重置对应的PID.
 * 
 * @note 这个函数不会创建或删除任何电机或离线检查器，只是设置它们的状态.
 */
void Feed::setCommData(bool is_working)
{
  float motor_cur_ref = status_.cur_ref;
  HW_ASSERT(motor_ptr_ != nullptr, "pointer to Feed Motor is nullptr", motor_ptr_);
  HW_ASSERT(pid_ptr_ != nullptr, "pointer to Feed PID is nullptr", pid_ptr);

  if ((!motor_ptr_->isOffline()) && is_working) {
    motor_ptr_->setInput(motor_cur_ref);
  } else {
    resetPids();
    motor_ptr_->setInput(0);
  }
};

#pragma endregion

#pragma region 数据更新接口
/** 
 * @brief       设定拨弹限制
 * @param       is_trigger_allowed: 是否允许拨弹
 * @param       is_heat_limited:    是否限制热量
 * @param       safe_num:           安全热量值对应的弹丸个数
 * @param       interval:           拨弹间隔时间, 单位 ms   
 * @note        该函数用于设定拨弹频率限制和热量限制。
 *              当 is_trigger_allowed 为 false 时, 后续所有参数设置不起作用。
 *              当 is_heat_limited 为 false 时, safe_num 参数设置不起作用。
 */
void Feed::setTriggerLimit(bool is_trigger_allowed, bool is_heat_limited, float safe_num, uint32_t interval)
{
  HW_ASSERT(safe_num >= 0, "safe_num must be not less than 0!");
  is_trigger_allowed_ = is_trigger_allowed;
  if (!is_trigger_allowed) {
    return;
  }
  is_heat_limited_ = is_heat_limited;
  if (is_heat_limited) {
    safe_num_bullet_ = safe_num;
  }
  trigger_interval_ = interval;
}

#pragma endregion

#pragma region 其他工具函数

/** 
 * @brief       寻找下一个拨弹电机最佳目标角度
 * @param       fdb:            当前拨弹电机反馈角度, 单位 rad
 * @param       is_forward:     向前/向后拨弹
 * @retval      目标角度, 单位 rad
 * @note        
 */
float Feed::searchAngRef(float fdb, bool is_forward) const
{
  float offset = cfg_.ang_ref_offset;
  float ang_per_blt = cfg_.ang_per_blt;
  float idx = roundf(AngleNormRad(fdb - offset) / ang_per_blt);
  if (is_forward) {
    return AngleNormRad(offset + ang_per_blt * (idx + 1));
  } else {
    return AngleNormRad(offset + ang_per_blt * idx);
  }
};
#pragma endregion
}  // namespace feed_impl
}  // namespace module
}  // namespace hello_world