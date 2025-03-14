/**
 *******************************************************************************
 * @file      :robot.cpp
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
#include "can.h"
#include "rfr_pkg/rfr_id.hpp"

#include "robot.hpp"

/* Private macro -------------------------------------------------------------*/

namespace robot {
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

#pragma region 数据更新
// 状态机主要接口函数
void Robot::update() {
  updateData();
  updatePwrState();
};

void Robot::updateData() {
  updateWorkTick();
  updateImuData();
  updateGimbalChassisCommData();
  updateVisionData();
};

void Robot::updateImuData() {
  HW_ASSERT(imu_ptr_ != nullptr, "IMU pointer is null", imu_ptr_);
  imu_ptr_->update();
  if (imu_ptr_->isOffsetCalcFinished()) {
    is_imu_caled_offset_ = true;
  }
};

void Robot::updateGimbalChassisCommData() {
  GimbalChassisComm::RefereeData::ChassisPart &referee_data =
      gc_comm_ptr_->referee_data().cp;

  Feed::RfrInputData feed_rfr_input_data;
  feed_rfr_input_data.is_rfr_on = true;
  feed_rfr_input_data.is_power_on = true;
  feed_rfr_input_data.is_new_bullet_shot = referee_data.is_new_bullet_shot;
  feed_rfr_input_data.heat_limit = referee_data.shooter_heat_limit;
  feed_rfr_input_data.heat = referee_data.shooter_heat;
  feed_rfr_input_data.heat_cooling_ps = referee_data.shooter_cooling;
  feed_ptr_->updateRfrData(feed_rfr_input_data);

  Fric::RfrInputData fric_rfr_input_data;
  fric_rfr_input_data.is_power_on = true;
  fric_rfr_input_data.is_new_bullet_shot = referee_data.is_new_bullet_shot;
  fric_rfr_input_data.bullet_spd = referee_data.bullet_speed;
  fric_ptr_->updateRfrData(fric_rfr_input_data);

  gimbal_ptr_->updateIsRfrPwrOn(true);

  vision_ptr_->setBulletSpeed(referee_data.bullet_speed);
};

void Robot::updateVisionData() {
  if (vision_ptr_->isOffline()) {
    gimbal_ptr_->setVisionTargetDetected(false);
    feed_ptr_->setVisionShootFlag(Vision::ShootFlag::kNoShoot);
    return;
  }
  gimbal_ptr_->setVisionTargetDetected(vision_ptr_->getIsEnemyDetected());
  feed_ptr_->setVisionShootFlag(vision_ptr_->getShootFlag());
}

void Robot::updatePwrState() {
  PwrState pre_state = pwr_state_;
  PwrState next_state = pre_state;
  if (pre_state == PwrState::kDead) {
    // 主控板程序在跑就意味着有电，所以直接从死亡状态进入复活状态
    next_state = PwrState::kResurrection;
  } else if (pre_state == PwrState::kResurrection) {
    if (is_imu_caled_offset_) {
      next_state = PwrState::kWorking;
    }
  } else if (pre_state == PwrState::kWorking) {
    // 工作状态下，保持当前状态
  } else {
    // 其他状态，认为是死亡状态
    next_state = PwrState::kDead;
  }
  setPwrState(next_state);
};

#pragma endregion

#pragma region 执行任务
// 状态机主要接口函数
void Robot::runOnDead() {
  resetDataOnDead();
  standby();
};

void Robot::runOnResurrection() {
  resetDataOnResurrection();
  standby();
};

void Robot::runOnWorking() {
  genModulesCmd();

  // TODO(LKY) 后续可以考虑优化组件库，添加一个is_enabled的bool变量
  if (fric_ptr_->getWorkingMode() == Fric::WorkingMode::kStop) {
    laser_ptr_->disable();
  } else {
    laser_ptr_->enable();
  }

  HW_ASSERT(gimbal_ptr_ != nullptr, "Gimbal FSM pointer is null", gimbal_ptr_);
  gimbal_ptr_->update();
  gimbal_ptr_->run();

  HW_ASSERT(fric_ptr_ != nullptr, "Fric FSM pointer is null", fric_ptr_);
  fric_ptr_->update();
  fric_ptr_->run();

  transmitFricStatus();

  HW_ASSERT(feed_ptr_ != nullptr, "Feed FSM pointer is null", feed_ptr_);
  feed_ptr_->update();
  feed_ptr_->run();
};

void Robot::runAlways() {
  setCommData();
  sendCommData();
};

void Robot::standby() {
  laser_ptr_->disable();

  HW_ASSERT(gimbal_ptr_ != nullptr, "Gimbal FSM pointer is null", gimbal_ptr_);
  gimbal_ptr_->update();
  gimbal_ptr_->standby();

  HW_ASSERT(fric_ptr_ != nullptr, "Fric FSM pointer is null", fric_ptr_);
  fric_ptr_->update();
  fric_ptr_->standby();

  HW_ASSERT(feed_ptr_ != nullptr, "Feed FSM pointer is null", feed_ptr_);
  feed_ptr_->update();
  feed_ptr_->standby();
}

void Robot::genModulesCmd() {
  HW_ASSERT(gc_comm_ptr_ != nullptr, "GimbalChassisComm pointer is null",
            gc_comm_ptr_);

  // GimbalChassisComm::MainBoardData::ChassisPart &main_board_data =
  // gc_comm_ptr_->main_board_data().cp;
  GimbalChassisComm::GimbalData::ChassisPart &gimbal_data =
      gc_comm_ptr_->gimbal_data().cp;
  GimbalChassisComm::ShooterData::ChassisPart &shooter_data =
      gc_comm_ptr_->shooter_data().cp;
  // GimbalChassisComm::RefereeData::ChassisPart &referee_data =
  // gc_comm_ptr_->referee_data().cp;

  // gimbal
  CtrlMode gimbal_ctrl_mode = CtrlMode::kManual;
  if (gimbal_data.ctrl_mode == robot::CtrlMode::kAuto) {
    gimbal_ctrl_mode = CtrlMode::kAuto;
  };
  if (gimbal_ctrl_mode == CtrlMode::kAuto &&
      !vision_ptr_->getIsEnemyDetected()) {
    gimbal_ctrl_mode = CtrlMode::kManual;
  }

  if (gimbal_ctrl_mode == CtrlMode::kManual) {
    gimbal_ptr_->setNormCmdDelta(gimbal_data.yaw_delta,
                                 gimbal_data.pitch_delta);
    gimbal_ptr_->setRevHeadFlag(gimbal_data.turn_back_flag);
  } else if (gimbal_ctrl_mode == CtrlMode::kAuto) {
    gimbal_ptr_->setVisionCmd(vision_ptr_->getPoseRefYaw(),
                              vision_ptr_->getPoseRefPitch());
  }
  gimbal_ptr_->setCtrlMode(gimbal_ctrl_mode);
  gimbal_ptr_->setWorkingMode(gimbal_data.working_mode);

  // shooter
  // 操作手指令优先级高于视觉指令
  feed_ptr_->setManualShootFlag(shooter_data.shoot_flag(true));
  if (shooter_data.ctrl_mode == CtrlMode::kAuto) {
    feed_ptr_->setVisionShootFlag(vision_ptr_->getShootFlag());
    // feed_ptr_->setVisionShootFlag(Vision::ShootFlag::kNoShoot); // TODO:调试
  }
  if (shooter_data.working_mode == ShooterWorkingMode::kBackward ||
      shooter_data.working_mode == ShooterWorkingMode::kStop) {
    feed_ptr_->setTriggerLimit(false, false, 0.0f, 0);
  } else {
    feed_ptr_->setTriggerLimit(true, false, 2.5f, 40); // TODO:待测试合理冗余值
  }

  feed_ptr_->setCtrlMode(shooter_data.ctrl_mode);
  fric_ptr_->setWorkingMode(shooter_data.working_mode);
  // fric_ptr_->setWorkingMode(ShooterWorkingMode::kShoot); // TODO:调试
};

void Robot::transmitFricStatus() {
  feed_ptr_->setFricStatus(fric_ptr_->getStatus());
  // feed_ptr_->setFricStatus(true); // TODO:调试
};
#pragma endregion

#pragma region 数据重置函数
void Robot::reset() {
  pwr_state_ = PwrState::kDead;      ///< 电源状态
  last_pwr_state_ = PwrState::kDead; ///< 上一电源状态
};
void Robot::resetDataOnDead() {
  pwr_state_ = PwrState::kDead;      ///< 电源状态
  last_pwr_state_ = PwrState::kDead; ///< 上一电源状态
};
void Robot::resetDataOnResurrection() {};

#pragma endregion

#pragma region 通信数据设置函数
void Robot::setCommData() {
  // TODO(ZSC): 可能以后会在这里分工作状态
  setGimbalChassisCommData();
  setVisionCommData();
  // TODO(ZSC): 可能的其他通讯数据设置函数
  // 其他通讯模块的数据由各个子模块负责设置
  // 主控板非工作模式时，这些数据保持默认值
};

void Robot::setVisionCommData() {
  HW_ASSERT(vision_ptr_ != nullptr, "Vision pointer is null", vision_ptr_);
  HW_ASSERT(gimbal_ptr_ != nullptr, "Gimbal pointer is null", gimbal_ptr_);

  Vision::WorkState vision_work_State = Vision::WorkState::kStandby;
  if (gimbal_ptr_->getCtrlMode() == CtrlMode::kAuto ||
      feed_ptr_->getCtrlMode() == CtrlMode::kAuto) {
    vision_work_State = Vision::WorkState::kNormal;
  }

  hello_world::referee::RfrId robot_id =
      gc_comm_ptr_->referee_data().cp.robot_id;
  if (hello_world::referee::ids::GetTeamColor(robot_id) ==
      hello_world::referee::ids::TeamColor::kRed) {
    vision_ptr_->setTargetColor(Vision::TargetColor::kRed);
  } else if (hello_world::referee::ids::GetTeamColor(robot_id) ==
             hello_world::referee::ids::TeamColor::kBlue) {
    vision_ptr_->setTargetColor(Vision::TargetColor::kBlue);
  } else {
    vision_ptr_->setTargetColor(Vision::TargetColor::kPurple);
  }

  vision_ptr_->setPose(gimbal_ptr_->getJointRollAngFdb(),
                       gimbal_ptr_->getJointPitchAngFdb(),
                       gimbal_ptr_->getJointYawAngFdb());

  vision_ptr_->setWorkState(vision_work_State);
}

void Robot::setGimbalChassisCommData() {
  HW_ASSERT(gc_comm_ptr_ != nullptr, "GimbalChassisComm pointer is null",
            gc_comm_ptr_);
  HW_ASSERT(gimbal_ptr_ != nullptr, "Gimbal pointer is null", gimbal_ptr_);

  HW_ASSERT(feed_ptr_ != nullptr, "Feed pointer is null", feed_ptr_);
  HW_ASSERT(fric_ptr_ != nullptr, "Fric pointer is null", fric_ptr_);

  // main board
  GimbalChassisComm::MainBoardData::GimbalPart &main_board_data =
      gc_comm_ptr_->main_board_data().gp;
  main_board_data.is_gimbal_imu_ready = is_imu_caled_offset_;

  // gimbal
  GimbalChassisComm::GimbalData::GimbalPart &gimbal_data =
      gc_comm_ptr_->gimbal_data().gp;

  // shooter
  GimbalChassisComm::ShooterData::GimbalPart &shooter_data =
      gc_comm_ptr_->shooter_data().gp;
};

void Robot::sendCommData() {
  sendCanData();
  sendUsartData();
};
void Robot::sendCanData() {
  sendFricMotorData(); // TODO:调试
  sendFeedMotorData();

  if (work_tick_ % 2 == 0) {
    sendGimbalMotorData();
  } else if (work_tick_ % 2 == 1) {
    sendGimbalChassisCommData();
  }
};
void Robot::sendFricMotorData() {
  MotorIdx motor_idx[2] = {MotorIdx::kMotorIdxFricLeft,
                           MotorIdx::kMotorIdxFricRight};
  TxDevIdx tx_dev_idx[2] = {TxDevIdx::kMotorFricLeft,
                            TxDevIdx::kMotorFricRight};
  for (size_t i = 0; i < 2; i++) {
    HW_ASSERT(motor_ptr_[motor_idx[i]] != nullptr, "Motor pointer is null",
              motor_ptr_[motor_idx[i]]);
    tx_dev_mgr_pairs_[(uint32_t)tx_dev_idx[i]].setTransmitterNeedToTransmit();
  }
};

void Robot::sendFeedMotorData() {
  TxDevIdx tx_dev_idx = TxDevIdx::kMotorFeed;
  HW_ASSERT(motor_ptr_[MotorIdx::kMotorIdxFeed] != nullptr,
            "Motor pointer is null", motor_ptr_[MotorIdx::kMotorIdxFeed]);
  tx_dev_mgr_pairs_[(uint32_t)tx_dev_idx].setTransmitterNeedToTransmit();
}

void Robot::sendGimbalMotorData() {
  MotorIdx motor_idx[2] = {MotorIdx::kMotorIdxYaw, MotorIdx::kMotorIdxPitch};
  TxDevIdx tx_dev_idx[2] = {TxDevIdx::kMotorYaw, TxDevIdx::kMotorPitch};
  for (size_t i = 0; i < 2; i++) {
    HW_ASSERT(motor_ptr_[motor_idx[i]] != nullptr, "Motor pointer is null",
              motor_ptr_[motor_idx[i]]);
    tx_dev_mgr_pairs_[(uint32_t)tx_dev_idx[i]].setTransmitterNeedToTransmit();
  }
};
void Robot::sendGimbalChassisCommData() {
  HW_ASSERT(gc_comm_ptr_ != nullptr, "GimbalChassisComm pointer is null",
            gc_comm_ptr_);
  tx_dev_mgr_pairs_[(uint32_t)TxDevIdx::kGimbalChassis]
      .setTransmitterNeedToTransmit();
};
void Robot::sendUsartData() {
  if (work_tick_ % 10 == 0) {
    sendVisionData();
  }
};
void Robot::sendVisionData() {
  HW_ASSERT(vision_ptr_ != nullptr, "Vision pointer is null", vision_ptr_);
  tx_dev_mgr_pairs_[(uint32_t)TxDevIdx::kVision].setTransmitterNeedToTransmit();
};
#pragma endregion

#pragma region 注册函数

void Robot::registerGimbal(Gimbal *ptr) {
  HW_ASSERT(ptr != nullptr, "Gimbal pointer is null", ptr);
  gimbal_ptr_ = ptr;
};

void Robot::registerFeed(Feed *ptr) {
  HW_ASSERT(ptr != nullptr, "Feed pointer is null", ptr);
  feed_ptr_ = ptr;
};
void Robot::registerFric(Fric *ptr) {
  HW_ASSERT(ptr != nullptr, "Fric pointer is null", ptr);
  fric_ptr_ = ptr;
};
void Robot::registerBuzzer(Buzzer *ptr) {
  HW_ASSERT(ptr != nullptr, "Buzzer pointer is null", ptr);
  buzzer_ptr_ = ptr;
};
void Robot::registerImu(Imu *ptr) {
  HW_ASSERT(ptr != nullptr, "IMU pointer is null", ptr);
  imu_ptr_ = ptr;
};
void Robot::registerLaser(Laser *ptr) {
  HW_ASSERT(ptr != nullptr, "pointer to laser is nullptr", ptr);
  laser_ptr_ = ptr;
}
void Robot::registerMotor(Motor *dev_ptr, uint8_t idx,
                          CanTxMgr *tx_dev_mgr_ptr) {
  HW_ASSERT(dev_ptr != nullptr, "Motor pointer is null", dev_ptr);
  HW_ASSERT(idx < kMotorNum, "Motor index is out of range", idx);
  HW_ASSERT(tx_dev_mgr_ptr != nullptr, "CanTxMgr pointer is null",
            tx_dev_mgr_ptr);

  // MotorIdx motor_idx[kMotorNum] = {
  //     kMotorIdxFricLeft,  ///< 左摩擦轮电机下标
  //     kMotorIdxFricRight, ///< 右摩擦轮电机下标
  //     kMotorIdxFeed,      ///< 拨盘电机下标
  //     kMotorIdxYaw,       ///< YAW 轴电机下标
  //     kMotorIdxPitch,     ///< PITCH 轴电机下标
  // };

  TxDevIdx tx_dev_idx[kMotorNum] = {
      TxDevIdx::kMotorFricLeft,  ///< 左摩擦轮电机通信设备下标
      TxDevIdx::kMotorFricRight, ///< 右摩擦轮电机通信设备下标
      TxDevIdx::kMotorFeed,      ///< 拨盘电机通信设备下标
      TxDevIdx::kMotorYaw,       ///< YAW 轴电机通信设备下标
      TxDevIdx::kMotorPitch,     ///< PITCH 轴电机通信设备下标
  };

  motor_ptr_[idx] = dev_ptr;
  tx_dev_mgr_pairs_[(uint32_t)tx_dev_idx[idx]].transmitter_ptr_ = dev_ptr;
  tx_dev_mgr_pairs_[(uint32_t)tx_dev_idx[idx]].tx_mgr_ptr_ = tx_dev_mgr_ptr;
};
void Robot::registerGimbalChassisComm(GimbalChassisComm *dev_ptr,
                                      CanTxMgr *tx_dev_mgr_ptr) {
  HW_ASSERT(dev_ptr != nullptr, "GimbalChassisComm pointer is null", dev_ptr);
  HW_ASSERT(tx_dev_mgr_ptr != nullptr, "CanTxMgr pointer is null",
            tx_dev_mgr_ptr);

  gc_comm_ptr_ = dev_ptr;
  tx_dev_mgr_pairs_[(uint32_t)TxDevIdx::kGimbalChassis].transmitter_ptr_ =
      dev_ptr;
  tx_dev_mgr_pairs_[(uint32_t)TxDevIdx::kGimbalChassis].tx_mgr_ptr_ =
      tx_dev_mgr_ptr;
};

void Robot::registerVision(Vision *dev_ptr, UartTxMgr *tx_dev_mgr_ptr) {
  HW_ASSERT(dev_ptr != nullptr, "pointer to Vision is nullptr", dev_ptr);
  HW_ASSERT(tx_dev_mgr_ptr != nullptr, "CanTxMgr pointer is null",
            tx_dev_mgr_ptr);
  vision_ptr_ = dev_ptr;
  tx_dev_mgr_pairs_[(uint32_t)TxDevIdx::kVision].transmitter_ptr_ = dev_ptr;
  tx_dev_mgr_pairs_[(uint32_t)TxDevIdx::kVision].tx_mgr_ptr_ = tx_dev_mgr_ptr;
};

#pragma endregion
/* Private function definitions ----------------------------------------------*/
} // namespace robot