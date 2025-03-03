/**
 *******************************************************************************
 * @file      :chassis.cpp
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
#include "chassis.hpp"
/* Private macro -------------------------------------------------------------*/
// DEBUG:
float wheel_speed_fdb_debug = 0;
float wheel_speed_ref_debug = 0;
// float steer_speed_fdb_debug = 0;
// float steer_speed_ref_debug = 0;

// TODO: norm_cmd_和cmd_好像没有关联起来，非常奇怪
// Robot的runOnWorking里产生各个模块的norm_cmd_
namespace robot {
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

#pragma region 数据更新
void Chassis::update() {
  updateData();
  updatePwrState();
};

void Chassis::updateData() {
  updateWorkTick();
  updateGimbalBoardData();
  updateMotorData();
  updateCapData();
  updateIsPowerOn();
};

/**
 * 更新电源状态
 *
 * 无论处于任何状态，只要掉电就切换到死亡状态。
 * 死亡状态下，如果上电，则切到复活状态；
 * 复活状态下，如果有轮或舵电机上电完毕（底盘已经准备完毕），且云台板 IMU
 * 准备就绪，则切到工作状态； 工作状态下，保持当前状态；
 * 其他状态，认为是死亡状态。
 */
void Chassis::updatePwrState() {
  // 无论任何状态，断电意味着要切到死亡状态
  if (!is_power_on_) {
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
    // 1. 为什么要判断云台板准备就绪？
    // 云台板计算零飘完零飘后会告知底盘准备就绪，如果云台板还未准备好，底盘就能够运动会导致云台板的姿态计算结果可能不准确
    // 2. 为什么只要有轮或舵电机上电完毕，就认为底盘就准备好了？
    // 因为底盘逆解不需要所有轮或舵电机都在线也可以运行
    // 在后续正常工作状态下，会对轮或舵电机的状态进行检测
    // 如果有轮或舵电机掉电，会进行专门的处理，比如 pid 清空、can 发无效数据等
    // 3. 所有控制模式都需要 yaw 轴电机的角度，为什么不判断 yaw 轴电机的状态？
    // 因为 yaw 轴电机的状态不影响底盘的运动解算
    // 当 yaw 轴电机离线时，底盘会按照底盘坐标系进行解算（yaw
    // 电机离线后数据清空，默认返回 0，能跑但是疯了，但总比不能跑强） 当 yaw
    // 轴电机上电时，底盘会按照图传坐标系进行解算
    // if (is_gimbal_imu_ready_ && is_any_wheel_online_) {
    // //TODO：底盘独立运动模式，逻辑待优化
    if (is_any_wheel_online_) {
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

void Chassis::updateGimbalBoardData() {
  // 当云台板通讯丢失时，认为无云台板
  // 此时，直接认为云台板 IMU 准备就绪，使得可以在无云台板下工作
  HW_ASSERT(gc_comm_ptr_ != nullptr, "pointer to GimbalChassisComm is nullptr",
            gc_comm_ptr_);
  if (gc_comm_ptr_->isOffline()) {
    is_gimbal_imu_ready_ = false;
  } else {
    is_gimbal_imu_ready_ =
        gc_comm_ptr_->main_board_data().gp.is_gimbal_imu_ready;
  }
};

void Chassis::updateMotorData() {
  Motor *wheel_motor_ptr = nullptr;
  WheelMotorIdx wmis[4] = {
      kWheelMotorIdxLeftFront,
      kWheelMotorIdxLeftRear,
      kWheelMotorIdxRightRear,
      kWheelMotorIdxRightFront,
  };
  bool is_all_wheel_online = true;
  bool is_any_wheel_online = false;

  for (size_t i = 0; i < 4; i++) {
    WheelMotorIdx wmi = wmis[i];
    wheel_motor_ptr = wheel_motor_ptr_[wmi];

    HW_ASSERT(wheel_motor_ptr != nullptr,
              "pointer to wheel motor %d is nullptr", wmi);
    if (wheel_motor_ptr->isOffline()) {
      wheel_speed_fdb_[wmi] = 0.0;
      wheel_current_fdb_[wmi] = 0.0;
      is_all_wheel_online = false;
    } else {
      is_any_wheel_online = true;
      wheel_speed_fdb_[wmi] = wheel_motor_ptr->vel();
      wheel_current_fdb_[wmi] = wheel_motor_ptr->curr();
    }
  }
  is_all_wheel_online_ = is_all_wheel_online;
  is_any_wheel_online_ = is_any_wheel_online;

  HW_ASSERT(yaw_motor_ptr_ != nullptr, "pointer to Yaw motor is nullptr",
            yaw_motor_ptr_);
  if (yaw_motor_ptr_->isOffline()) {
    theta_i2r_ = 0.0;
  } else {
    theta_i2r_ = -yaw_motor_ptr_->angle();
  }
};

void Chassis::updateCapData() {
  HW_ASSERT(cap_ptr_ != nullptr, "pointer to Capacitor is nullptr", cap_ptr_);
  if (cap_ptr_->isOffline()) {
    is_high_spd_enabled_ = false;
    cap_remaining_energy_ = 0.0f;
  } else {
    is_high_spd_enabled_ = cap_ptr_->isUsingSuperCap();
    cap_remaining_energy_ = cap_ptr_->getRemainingPower();
  }
};

void Chassis::updateIsPowerOn() {
  is_power_on_ = is_any_wheel_online_ || rfr_data_.is_pwr_on;
  if (!is_power_on_) {
    last_pwr_off_tick_ = work_tick_;
  }
};

#pragma endregion

#pragma region 执行任务
void Chassis::runOnDead() { resetDataOnDead(); };

void Chassis::runOnResurrection() { resetDataOnResurrection(); };

void Chassis::runOnWorking() {
  revNormCmd();
  calcWheelSpeedRef();
  calcWheelLimitedSpeedRef();
  calcWheelCurrentRef();
  calcWheelCurrentLimitedRef();
};

void Chassis::runAlways() { setCommData(pwr_state_ == PwrState::kWorking); };

void Chassis::standby() {
  resetDataOnStandby();
  setCommData(false);
};
#pragma endregion

#pragma region 工作状态下，获取控制指令的函数

// TODO:调试1
hello_world::pid::MultiNodesPid::Datas pid_data; // TODO:PID调试数据
// 使用示例：pid_data = pid_ptr->getPidAt(0).datas();
uint16_t cnt = 0;
void Chassis::revNormCmd() {
  float beta = 0.01f;
  static bool first_follow_flag = true;
  Cmd cmd = norm_cmd_;

  switch (working_mode_) {
  case WorkingMode::Depart: {
    // 分离模式不对 norm_cmd_ 进行额外处理
    gyro_dir_ = GyroDir::Unspecified;
    first_follow_flag = true;
    cmd.w = 0.0f;
    break;
  }

  case WorkingMode::Gyro: {
    // 陀螺模式下，如果外部没有设置陀螺旋转方向，
    // 则每次进入陀螺模式时，随机选择一个方向
    first_follow_flag = true;

    if (gyro_dir_ == GyroDir::Unspecified) {
      if (rand() % 2 == 0) {
        gyro_dir_ = GyroDir::Clockwise;
      } else {
        gyro_dir_ = GyroDir::AntiClockwise;
      }
    }
    // 小陀螺模式下，旋转分量为定值
    cmd.w = config_.gyro_rot_spd * (int8_t)gyro_dir_;
    break;
  }

  case WorkingMode::Follow: {
    gyro_dir_ = GyroDir::Unspecified;
    // TODO：掉头模式
    // //  在转头过程中，底盘不响应跟随转动指令
    // if (work_tick_ - last_rev_head_tick_ < 800) {
    //   break;
    // }
    static float theta_ref = 0.0f;
    float theta_fdb = 0.0f;

    // TODO：底盘独立运动模式的跟随模式，逻辑待优化
    if (is_gimbal_imu_ready_) {
      // 跟随模式下，云台在线，更新跟随目标
      theta_fdb = theta_i2r_;
      // TODO：跟随模式云台前馈记录
      // float omega_feedforward = config_.yaw_sensitivity * omega_feedforward_;
      // follow_omega_pid_ptr_->calc(&theta_ref, &theta_fdb, &omega_feedforward,
      //                             &cmd.w);

      if (first_follow_flag) {
        first_follow_flag = false;
        if (fabs(theta_fdb) <= PI / 2) {
          theta_ref = 0.0f;
        } else {
          theta_ref = PI;
        }
      }

      follow_omega_pid_ptr_->calc(&theta_ref, &theta_fdb, nullptr, &cmd.w);

      // TODO:大小死区处理,优化零漂震荡问题
      const float omega_dead_upper = 0.25f;
      const float omega_dead_lower = 0.05f;
      static float omega_dead_limit = omega_dead_upper;
      static bool dead_cross_flag = false;
      if (!dead_cross_flag && fabs(cmd.w) > omega_dead_upper) {
        omega_dead_limit = omega_dead_lower;
        dead_cross_flag = true;
      } else if (dead_cross_flag && fabs(cmd.w) < omega_dead_lower) {
        omega_dead_limit = omega_dead_upper;
        dead_cross_flag = false;
      }
      if (fabs(cmd.w) < omega_dead_limit) {
        cmd.w = 0.0f;
      }

      // if (fabs(cmd.w) < 0.1f) {
      //   cmd.w = 0.0f;
      //   cnt++;
      // }
    } else {
      // TODO：跟随模式下，云台掉线；独立底盘逻辑，类似纯前馈
      theta_ref = theta_fdb;
      float omega_feedforward = config_.yaw_sensitivity * omega_feedforward_;
      follow_omega_pid_ptr_->calc(&theta_ref, &theta_fdb, &omega_feedforward,
                                  &cmd.w);
    }
    break;
  }
  default: {
    break;
  }
  }

  if (is_high_spd_enabled_) {
    cmd *= 1.5;
    beta = 1.0;
  }

  // TODO：修改限幅逻辑，保证限幅后平移速度方向不变
  cmd.v_x = hello_world::Bound(cmd.v_x * config_.normal_trans_vel,
                               -config_.max_trans_vel, config_.max_trans_vel);
  cmd.v_y = hello_world::Bound(cmd.v_y * config_.normal_trans_vel,
                               -config_.max_trans_vel, config_.max_trans_vel);
  cmd.w = hello_world::Bound(cmd.w, -config_.max_rot_spd, config_.max_rot_spd);

  // 注意Chassis::cmd_由本函数赋值，目前框架下本函数不能直接注释掉
  setCmdSmoothly(
      cmd, beta); // 斜坡优化已在robot.cpp中用斜坡滤波器实现，beta暂设为1.0
};

void Chassis::calcWheelSpeedRef() {
  // 底盘坐标系下，x轴正方向为底盘正前方，y轴正方向为底盘正左方，z轴正方向为底盘正上方
  // 轮子顺序按照象限顺序进行编号：左前，左后，右后，右前
  HW_ASSERT(ik_solver_ptr_ != nullptr, "pointer to IK solver is nullptr",
            ik_solver_ptr_);
  hello_world::chassis_ik_solver::MoveVec move_vec(cmd_.v_x, cmd_.v_y, cmd_.w);
  float theta_i2r = theta_i2r_;
  ik_solver_ptr_->solve(move_vec, theta_i2r, nullptr);
  ik_solver_ptr_->getRotSpdAll(wheel_speed_ref_raw_);
}

float rfr_pwr = 0.0f; // TODO:功限调试
void Chassis::calcWheelLimitedSpeedRef() {
  hello_world::power_limiter::PowerLimiterRuntimeParams runtime_params = {
      /* 期望功率上限，可以根据模式设置（低速/高速），单位：W */
      .p_ref_max = 1.2f * rfr_data_.pwr_limit, // 60.0f, 1.2f*, 2.0f*
      /* 裁判系统给出的底盘功率限制，单位：W */
      // .p_referee_max = static_cast<float>(rfr_data_.pwr_limit),
      .p_referee_max = 120.0f,
      /* 期望功率下限，建议设为当前 p_referee_max 乘一个 0 ~ 1
         的比例系数，单位：W */
      .p_ref_min = 0.8f * rfr_data_.pwr_limit, // 50.0f,
      /* 剩余能量：采用裁判系统控制时为裁判系统缓冲能量 buffer_energy，单位：J
       */
      .remaining_energy = static_cast<float>(rfr_data_.pwr_buffer),
      /* 剩余能量收敛值，当剩余能量到达该值时，期望功率等于裁判系统给出的底盘功率上限
         p_referee_max */
      .energy_converge = 40.0f,
      /* 期望功率随当前剩余能量线性变化的斜率
      >= (p_ref_max - p_referee_max) / (remaining_energy - energy_converge)
      开启超电时 remaining_energy 可取 100 */
      .p_slope = 1.2f, // 1.6f
      /* 危险能量值，若剩余能量低于该值，p_ref_ 设为 0 */
      .danger_energy = 5.0f,
  };
  rfr_pwr = rfr_data_.pwr;

  // TODO:关闭超电时需一并注释掉，防止间断通信误入本分支
  if (!cap_ptr_->isOffline()) {
    runtime_params.remaining_energy = cap_ptr_->getRemainingPower();
  }

  // TODO:待适配
  // if (is_high_spd_enabled_) {
  //   runtime_params.p_rfr_max += 800.0f;
  //   runtime_params.z_rfr_measure = 60.0f;
  // }
  pwr_limiter_ptr_->updateWheelModel(wheel_speed_ref_raw_, wheel_speed_fdb_,
                                     nullptr, nullptr);
  pwr_limiter_ptr_->calc(runtime_params, wheel_speed_ref_limited_,
                         nullptr); // 更新运行时参数
};
void Chassis::calcWheelCurrentRef() {
  // 计算每个轮子的期望转速
  // 期望转速由 PID 控制器计算，期望转速与实际转速之间的差距由限幅器控制
  WheelPidIdx wpis[4] = {
      kWheelPidIdxLeftFront,
      kWheelPidIdxLeftRear,
      kWheelPidIdxRightRear,
      kWheelPidIdxRightFront,
  };
  WheelSpeedRampIdx wris[4] = {
      kWheelSpeedRampIdxLeftFront,
      kWheelSpeedRampIdxLeftRear,
      kWheelSpeedRampIdxRightRear,
      kWheelSpeedRampIdxRightFront,
  };

  MultiNodesPid *pid_ptr = nullptr;
  Ramp *ramp_ptr = nullptr;
  for (size_t i = 0; i < 4; i++) {
    pid_ptr = wheel_pid_ptr_[wpis[i]];
    ramp_ptr = wheel_speed_ramp_ptr_[wris[i]];
    HW_ASSERT(pid_ptr != nullptr, "pointer to Wheel PID %d is nullptr",
              wpis[i]);
    HW_ASSERT(ramp_ptr != nullptr, "pointer to Wheel Speed Ramp %d is nullptr",
              wris[i]);
    // ramp_ptr->calc(&wheel_speed_ref_raw_[i], &wheel_speed_ref_ramped_[i]);
    // pid_ptr->calc(&wheel_speed_ref_ramped_[i], &wheel_speed_fdb_[i], nullptr,
    //               &wheel_current_ref_[i]);
    pid_ptr->calc(&wheel_speed_ref_raw_[i], &wheel_speed_fdb_[i], nullptr,
                  &wheel_current_ref_[i]);
  }
}
void Chassis::calcWheelCurrentLimitedRef() {
  // 计算每个轮子的期望转速
  // 期望转速由 PID 控制器计算，期望转速与实际转速之间的差距由限幅器控制
  WheelPidIdx wpis[4] = {
      kWheelPidIdxLeftFront,
      kWheelPidIdxLeftRear,
      kWheelPidIdxRightRear,
      kWheelPidIdxRightFront,
  };
  MultiNodesPid *pid_ptr = nullptr;
  for (size_t i = 0; i < 4; i++) {
    pid_ptr = wheel_pid_ptr_[wpis[i]];
    HW_ASSERT(pid_ptr != nullptr, "pointer to Wheel PID %d is nullptr",
              wpis[i]);
    pid_ptr->calc(&wheel_speed_ref_limited_[i], &wheel_speed_fdb_[i], nullptr,
                  &wheel_current_ref_limited_[i]);
  }
};

#pragma endregion

#pragma region 数据重置函数
void Chassis::reset() {
  pwr_state_ = PwrState::kDead;      ///< 电源状态
  last_pwr_state_ = PwrState::kDead; ///< 上一电源状态

  // 由 robot 设置的数据
  use_cap_flag_ = false; ///< 是否使用超级电容
  gyro_dir_ =
      GyroDir::Unspecified; ///< 小陀螺方向，正为绕 Z 轴逆时针，负为顺时针，
  norm_cmd_.reset();
  rfr_data_ = RfrData(); ///< 底盘 RFR 数据

  working_mode_ = WorkingMode::Depart;      ///< 工作模式
  last_working_mode_ = WorkingMode::Depart; ///< 上一次工作模式

  // 在 update 函数中更新的数据
  is_power_on_ = false; ///< 底盘电源是否开启
  // work_tick_ = 0;          ///< 记录底盘模块的运行时间，单位为 ms
  // last_pwr_off_tick_ = 0;  ///< 上一次底盘电源处于关闭状态的时间戳，单位为
  // ms，实际上是作为上电瞬间的记录

  // 在 runOnWorking 函数中更新的数据
  resetMotorsRef();
  resetCmds();
  // gimbal board fdb data  在 update 函数中更新
  is_gimbal_imu_ready_ = false; ///< 云台主控板的IMU是否准备完毕

  // motor fdb data 在 update 函数中更新
  is_all_wheel_online_ = false; ///< 所有轮电机是否都处于就绪状态
  is_any_wheel_online_ = false; ///< 任意轮电机是否处于就绪状态

  theta_i2r_ = 0.0f; ///< 图传坐标系绕 Z
                     ///< 轴到底盘坐标系的旋转角度，右手定则判定正反向，单位 rad

  // cap fdb data 在 update 函数中更新
  is_high_spd_enabled_ = false; ///< 是否开启了高速模式 （开启意味着从电容取电）
  cap_remaining_energy_ = 0.0f; ///< 剩余电容能量百分比，单位 %

  resetMotorsFdb();
  resetPids(); ///< 重置 PID 控制器参数
};
void Chassis::resetDataOnDead() {
  // 由 robot 设置的数据
  // 在 update 函数中更新的数据
  // 在 runOnWorking 函数中更新的数据
  resetCmds();

  // gimbal board fdb data  在 update 函数中更新
  // motor fdb data 在 update 函数中更新
  // cap fdb data 在 update 函数中更新
  resetMotorsFdb();
  resetPids(); ///< 重置 PID 控制器参数
};
void Chassis::resetDataOnResurrection() {
  // 由 robot 设置的数据
  // 在 update 函数中更新的数据
  // 在 runOnWorking 函数中更新的数据
  resetCmds();

  // gimbal board fdb data  在 update 函数中更新
  // motor fdb data 在 update 函数中更新
  // cap fdb data 在 update 函数中更新
  resetMotorsFdb();
  resetPids(); ///< 重置 PID 控制器参数
};

void Chassis::resetDataOnStandby() {
  // 由 robot 设置的数据
  // 在 update 函数中更新的数据
  // 在 runOnWorking 函数中更新的数据
  resetCmds();

  // gimbal board fdb data  在 update 函数中更新
  // motor fdb data 在 update 函数中更新
  // cap fdb data 在 update 函数中更新
  resetMotorsFdb();
  resetPids(); ///< 重置 PID 控制器参数
};

// 重置控制指令
void Chassis::resetCmds() {
  cmd_.reset();      ///< 控制指令，基于图传坐标系
  last_cmd_.reset(); ///< 上一控制周期的控制指令，基于图传坐标系

  rev_head_flag_ = false;  ///< 恢复反转标志位
  last_rev_head_tick_ = 0; ///< 上一次转向后退的时间戳
};

// 重置轮电机参考数据
void Chassis::resetMotorsRef() {
  memset(wheel_speed_ref_raw_, 0, sizeof(wheel_speed_ref_raw_));
  memset(wheel_speed_ref_ramped_, 0, sizeof(wheel_speed_ref_ramped_));
  memset(wheel_speed_ref_limited_, 0, sizeof(wheel_speed_ref_limited_));
  memset(wheel_current_ref_, 0, sizeof(wheel_current_ref_));
  memset(wheel_current_ref_limited_, 0, sizeof(wheel_current_ref_limited_));
}

// 重置轮电机反馈数据
void Chassis::resetMotorsFdb() {
  memset(wheel_speed_fdb_, 0, sizeof(wheel_speed_fdb_));
  memset(wheel_current_fdb_, 0, sizeof(wheel_current_fdb_));
}

void Chassis::resetPids() {
  for (size_t i = 0; i < kWheelPidNum; i++) {
    wheel_pid_ptr_[i]->reset();
  }
  follow_omega_pid_ptr_->reset();
};
#pragma endregion

#pragma region 通讯数据设置函数

// 将轮、舵电机相关信息补充进本函数
void Chassis::setCommDataMotors(bool working_flag) {
  // TODO：测试yaw电机
  HW_ASSERT(yaw_motor_ptr_ != nullptr, "pointer to Yaw motor is nullptr",
            yaw_motor_ptr_);
  yaw_motor_ptr_->setInput(0);

  // 轮电机根据期望电流输入发送数据
  WheelMotorIdx wmis[4] = {
      kWheelMotorIdxLeftFront,
      kWheelMotorIdxLeftRear,
      kWheelMotorIdxRightRear,
      kWheelMotorIdxRightFront,
  };
  WheelPidIdx wpis[4] = {
      kWheelPidIdxLeftFront,
      kWheelPidIdxLeftRear,
      kWheelPidIdxRightRear,
      kWheelPidIdxRightFront,
  };
  Motor *wheel_motor_ptr = nullptr;
  MultiNodesPid *wheel_pid_ptr = nullptr;

  for (size_t i = 0; i < 4; i++) {
    WheelMotorIdx wmi = wmis[i];
    WheelPidIdx wpi = wpis[i];

    wheel_motor_ptr = wheel_motor_ptr_[wmi];
    wheel_pid_ptr = wheel_pid_ptr_[wpi];

    HW_ASSERT(wheel_motor_ptr != nullptr,
              "pointer to wheel motor %d is nullptr", wmi);
    if (!working_flag || wheel_motor_ptr->isOffline()) {
      wheel_pid_ptr->reset();
      wheel_motor_ptr->setInput(0);
    } else {
      // wheel_motor_ptr->setInput(0); // TODO：调试
      wheel_motor_ptr->setInput(wheel_current_ref_[wmi]); // TODO：功限调试
      // wheel_motor_ptr->setInput(wheel_current_ref_limited_[wmi]);
    }
  }
};

void Chassis::setCommDataCap(bool working_flag) {
  // 电容根据电容充电状态发送数据
  HW_ASSERT(cap_ptr_ != nullptr, "pointer to Capacitor is nullptr", cap_ptr_);
  if (working_flag && use_cap_flag_) {
    cap_ptr_->enable();
  } else {
    cap_ptr_->disable();
  }
  if (pwr_state_ != PwrState::kWorking) {
    cap_ptr_->setRfrData(rfr_data_.pwr_buffer, rfr_data_.pwr_limit, 0);
  } else {
    cap_ptr_->setRfrData(rfr_data_.pwr_buffer, rfr_data_.pwr_limit,
                         rfr_data_.current_hp);
  }
};

#pragma endregion

#pragma region 注册函数

void Chassis::registerWheelMotor(Motor *ptr, int idx) {
  HW_ASSERT(ptr != nullptr, "pointer to wheel motor %d is nullptr", idx);
  HW_ASSERT(idx >= 0 && idx < kWheelMotorNum,
            "index of wheel motor out of range", idx);
  wheel_motor_ptr_[idx] = ptr;
};
void Chassis::registerYawMotor(Motor *ptr) {
  HW_ASSERT(ptr != nullptr, "pointer to Yaw motor is nullptr", ptr);
  yaw_motor_ptr_ = ptr;
}
void Chassis::registerCap(Cap *ptr) {
  HW_ASSERT(ptr != nullptr, "pointer to Capacitor is nullptr", ptr);
  cap_ptr_ = ptr;
};

void Chassis::registerIkSolver(ChassisIkSolver *ptr) {
  HW_ASSERT(ptr != nullptr, "pointer to IK solver is nullptr", ptr);
  ik_solver_ptr_ = ptr;
};
void Chassis::registerWheelPid(MultiNodesPid *ptr, int idx) {
  HW_ASSERT(ptr != nullptr, "pointer to wheel PID %d is nullptr", idx);
  HW_ASSERT(idx >= 0 && idx < kWheelPidNum, "index of wheel PID out of range",
            idx);
  wheel_pid_ptr_[idx] = ptr;
};
void Chassis::registerFollowOmegaPid(MultiNodesPid *ptr) {
  HW_ASSERT(ptr != nullptr, "pointer to PID is nullptr", ptr);
  follow_omega_pid_ptr_ = ptr;
};
void Chassis::registerPwrLimiter(PwrLimiter *ptr) {
  HW_ASSERT(ptr != nullptr, "pointer to PwrLimiter is nullptr", ptr);
  pwr_limiter_ptr_ = ptr;
};
void Chassis::registerWheelSpeedRamp(Ramp *ptr, int idx) {
  HW_ASSERT(ptr != nullptr, "pointer to wheel speed ramp %d is nullptr", idx);
  HW_ASSERT(idx >= 0 && idx < kWheelSpeedRampNum,
            "index of wheel speed ramp out of range", idx);
  wheel_speed_ramp_ptr_[idx] = ptr;
};

void Chassis::registerGimbalChassisComm(GimbalChassisComm *ptr) {
  HW_ASSERT(ptr != nullptr, "pointer to GimbalChassisComm is nullptr", ptr);
  gc_comm_ptr_ = ptr;
};

#pragma endregion

/* Private function definitions ----------------------------------------------*/
} // namespace robot