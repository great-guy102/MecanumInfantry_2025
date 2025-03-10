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
#include "robot.hpp"

/* Private macro -------------------------------------------------------------*/

namespace robot {
/* Private constants ---------------------------------------------------------*/
const hello_world::referee::RobotPerformanceData kDefaultRobotPerformanceData = {
    .robot_id = static_cast<uint8_t>(
        hello_world::referee::ids::RobotId::kRedStandard3), ///< 本机器人ID

    .robot_level = 0,  ///< 机器人等级
    .current_hp = 200, ///< 机器人当前血量
    .maximum_hp = 200, ///< 机器人血量上限

    .shooter_barrel_cooling_value = 40, ///< 机器人枪口热量每秒冷却值
    .shooter_barrel_heat_limit = 100,   ///< 机器人枪口热量上限

    .chassis_power_limit =
        55, ///< 底盘功率限制，若不选择底盘或发射机构类型，则在七分钟比赛阶段开始后，未选择的底盘性能类型将被默认选择为“血量优先”

    .power_management_gimbal_output =
        1, ///< 电源管理模块 gimbal 口输出：0-关闭，1-开启
    .power_management_chassis_output =
        1, ///< 电源管理模块 chassis 口输出：0-关闭，1-开启
    .power_management_shooter_output =
        1, ///< 电源管理模块 shooter 口输出：0-关闭，1-开启
}; // namespace robot
const hello_world::referee::RobotPowerHeatData kDefaultRobotPowerHeatData = {
    .chassis_voltage = 24000, ///< 电源管理模块的chassis口输出电压，单位：mV
    .chassis_current = 0,     ///< 电源管理模块的chassis口输出电流，单位：mA
    .chassis_power = 0,       ///< 底盘功率，单位：W
    .buffer_energy = 60,      ///< 缓冲能量，单位：J

    .shooter_17mm_1_barrel_heat = 0, ///< 第一个17mm发射机构的枪口热量
    .shooter_17mm_2_barrel_heat = 0, ///< 第二个17mm发射机构的枪口热量
    .shooter_42mm_barrel_heat = 0,   ///< 42mm发射机构的枪口热量
};
const hello_world::referee::RobotShooterData kDefaultRobotShooterData = {
    .bullet_type = static_cast<uint8_t>(
        hello_world::referee::BulletType::k17mm), ///< 弹丸类型，@see
                                                  ///< BulletType

    .shooter_id = static_cast<uint8_t>(
        hello_world::referee::ShooterId::k17mm1), ///< 发射机构ID，@see
                                                  ///< ShooterId

    .launching_frequency = 0, ///< 弹丸射频，单位：Hz
    .bullet_speed = 15.5f,    ///< 弹丸初速度，单位：m/s
};
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
  updateRfrData();
  updateRcData();
};

void Robot::updateImuData() {
  HW_ASSERT(imu_ptr_ != nullptr, "IMU pointer is null", imu_ptr_);
  imu_ptr_->update();
  if (imu_ptr_->isOffsetCalcFinished()) {
    is_imu_caled_offset_ = true;
  }
};

void Robot::updateRcData() {
  HW_ASSERT(rc_ptr_ != nullptr, "RC pointer is null", rc_ptr_);
  if (manual_ctrl_src_ == ManualCtrlSrc::kRc) {
    if (rc_ptr_->isUsingKeyboardMouse()) {
      setManualCtrlSrc(ManualCtrlSrc::kKb);
    }
  } else if (manual_ctrl_src_ == ManualCtrlSrc::kKb) {
    if (rc_ptr_->isRcSwitchChanged()) { // 不检测摇杆变化，防止控制源来回切换
      setManualCtrlSrc(ManualCtrlSrc::kRc);
    }
  }
};

void Robot::updateRfrData() {
  HW_ASSERT(referee_ptr_ != nullptr, "RFR pointer is null", referee_ptr_);
  HW_ASSERT(chassis_ptr_ != nullptr, "Chassis FSM pointer is null",
            chassis_ptr_);

  Chassis::RfrData chassis_rfr_data;

  PerformancePkg::Data rpp_data = kDefaultRobotPerformanceData;
  PowerHeatPkg::Data rph_data = kDefaultRobotPowerHeatData;
  ShooterPkg::Data rsp_data = kDefaultRobotShooterData;

  if (!referee_ptr_->isOffline()) {
    rpp_data = rfr_performance_pkg_ptr_->getData();
    rph_data = rfr_power_heat_pkg_ptr_->getData();
    rsp_data = rfr_shooter_pkg_ptr_->getData();
  }

  chassis_rfr_data.is_rfr_on = (!referee_ptr_->isOffline());
  chassis_rfr_data.is_pwr_on = rpp_data.power_management_chassis_output;
  chassis_rfr_data.pwr_limit = rpp_data.chassis_power_limit;
  chassis_rfr_data.voltage = rph_data.chassis_voltage;
  chassis_rfr_data.pwr_buffer = rph_data.buffer_energy;
  chassis_rfr_data.pwr = rph_data.chassis_power;
  chassis_rfr_data.current_hp = rpp_data.current_hp;
  chassis_ptr_->setRfrData(chassis_rfr_data);

  GimbalChassisComm::RefereeData::ChassisPart &gimbal_rfr_data =
      gc_comm_ptr_->referee_data().cp;

  bool is_new_bullet_shot = false;
  if (!referee_ptr_->isOffline()) {
    if (!rfr_shooter_pkg_ptr_->isHandled()) {
      rfr_shooter_pkg_ptr_->setHandled();
      is_new_bullet_shot = true;
    }
  }
  gimbal_rfr_data.is_new_bullet_shot = is_new_bullet_shot;
  gimbal_rfr_data.robot_id = (RobotId)rpp_data.robot_id;

  // TODO: 换成实际的枪口
  gimbal_rfr_data.bullet_speed = rsp_data.bullet_speed;
  gimbal_rfr_data.shooter_heat = rph_data.shooter_17mm_1_barrel_heat;
  gimbal_rfr_data.shooter_cooling = rpp_data.shooter_barrel_cooling_value;
  gimbal_rfr_data.shooter_heat_limit = rpp_data.shooter_barrel_heat_limit;
};

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
void Robot::runOnDead() {
  resetDataOnDead();
  standby();
};

void Robot::runOnResurrection() {
  resetDataOnResurrection();
  standby();
};

void Robot::runOnWorking() {
  HW_ASSERT(buzzer_ptr_ != nullptr, "buzzer pointer is null", buzzer_ptr_);
  if (buzzer_ptr_->is_playing()) {
    buzzer_ptr_->play();
  }
  genModulesCmd();
  HW_ASSERT(chassis_ptr_ != nullptr, "Chassis FSM pointer is null",
            chassis_ptr_);
  chassis_ptr_->update();
  chassis_ptr_->run();
};

void Robot::runAlways() {
  setCommData();
  sendCommData();
};

void Robot::standby() {
  HW_ASSERT(chassis_ptr_ != nullptr, "Chassis FSM pointer is null",
            chassis_ptr_);
  chassis_ptr_->update();
  chassis_ptr_->standby();
};

#pragma endregion

#pragma region 生成控制指令
void Robot::genModulesCmd() {
  // TODO: 这里应该生成各个模块的指令，包括底盘指令、云台指令、底盘状态等等
  // 指令应该通过各个模块的接口发送给各个模块
  if (manual_ctrl_src_ == ManualCtrlSrc::kRc) {
    genModulesCmdFromRc();
  } else if (manual_ctrl_src_ == ManualCtrlSrc::kKb) {
    genModulesCmdFromKb();
  }
};

void Robot::genModulesCmdFromRc() {
  // 这里应该生成各个模块的指令，包括底盘指令、云台指令、底盘状态等等
  // 指令应该通过各个模块的接口发送给各个模块
  RcSwitchState l_switch = rc_ptr_->rc_l_switch();
  RcSwitchState r_switch = rc_ptr_->rc_r_switch();
  float rc_wheel = rc_ptr_->rc_wheel();

  Chassis::WorkingMode chassis_working_mode = Chassis::WorkingMode::Depart;
  Gimbal::WorkingMode gimbal_working_mode = Gimbal::WorkingMode::Normal;
  Shooter::WorkingMode shooter_working_mode = Shooter::WorkingMode::kStop;
  Chassis::GyroDir gyro_dir = Chassis::GyroDir::Unspecified;

  CtrlMode gimbal_ctrl_mode = CtrlMode::kManual;
  CtrlMode shooter_ctrl_mode = CtrlMode::kManual;

  bool use_cap_flag = false;
  bool shoot_flag = false;
  // bool rev_head_flag = false; //TODO:掉头模式

  // TODO: 后续需要加入慢拨模式
  if (l_switch == RcSwitchState::kUp) {
    // * 左上
    chassis_working_mode = Chassis::WorkingMode::Depart;

    if (r_switch == RcSwitchState::kUp) {
      // * 左上右上
      shooter_working_mode = Shooter::WorkingMode::kShoot;
      shoot_flag =
          (rc_wheel > 0.9f); // TODO：待修改判定在外部，自动模式也能手动发弹
      use_cap_flag = true;
    } else if (r_switch == RcSwitchState::kMid) {
      // * 左上右中
      shooter_working_mode = Shooter::WorkingMode::kShoot;
      gimbal_ctrl_mode = CtrlMode::kAuto;
      shooter_ctrl_mode = CtrlMode::kAuto;
    } else if (r_switch == RcSwitchState::kDown) {
      // * 左上右下
      gimbal_working_mode = Gimbal::WorkingMode::PidTest;
    }
  } else if (l_switch == RcSwitchState::kMid) {
    // * 左中
    chassis_working_mode = Chassis::WorkingMode::Follow;

    if (r_switch == RcSwitchState::kUp) {
      // * 左中右上
      shooter_working_mode = Shooter::WorkingMode::kShoot;
      shoot_flag = (rc_wheel > 0.9f);
      use_cap_flag = true;
    } else if (r_switch == RcSwitchState::kMid) {
      // * 左中右中
      shooter_working_mode = Shooter::WorkingMode::kShoot;
      gimbal_ctrl_mode = CtrlMode::kAuto;
      shooter_ctrl_mode = CtrlMode::kAuto;
      // rev_head_flag = (rc_wheel > 0.9f); // TODO:掉头模式
    } else if (r_switch == RcSwitchState::kDown) {
      // * 左中右下
      gimbal_working_mode = Gimbal::WorkingMode::PidTest;
    }
  } else if (l_switch == RcSwitchState::kDown) {
    // * 左下
    chassis_working_mode = Chassis::WorkingMode::Gyro;

    if (r_switch == RcSwitchState::kUp) {
      // * 左下右上
      use_cap_flag = true;
      gyro_dir = Chassis::GyroDir::Clockwise;
    } else if (r_switch == RcSwitchState::kMid) {
      // * 左下右中
      gimbal_ctrl_mode = CtrlMode::kAuto;
      shooter_ctrl_mode = CtrlMode::kAuto;
      gyro_dir = Chassis::GyroDir::Clockwise;
      // rev_head_flag = (rc_wheel > 0.9f); // TODO:掉头模式
    } else if (r_switch == RcSwitchState::kDown) {
      // * 左下右下
      gyro_dir = Chassis::GyroDir::AntiClockwise;
      gimbal_working_mode = Gimbal::WorkingMode::PidTest;
    }
  }

  ChassisCmd chassis_cmd = {0};
  // ChassisCmd chassis_cmd_raw = {0};
  // chassis_cmd_raw.v_x = hello_world::Bound(rc_ptr_->rc_rv(), -1, 1);
  // chassis_cmd_raw.v_y = hello_world::Bound(-rc_ptr_->rc_rh(), -1, 1);
  // ramp_cmd_vx_ptr_->calc(&(chassis_cmd_raw.v_x), &(chassis_cmd.v_x));
  // ramp_cmd_vy_ptr_->calc(&(chassis_cmd_raw.v_y), &(chassis_cmd.v_y));
  chassis_cmd.v_x = hello_world::Bound(rc_ptr_->rc_rv(), -1, 1);
  chassis_cmd.v_y = hello_world::Bound(-rc_ptr_->rc_rh(), -1, 1);
  chassis_cmd.w = 0.0f;
  chassis_ptr_->setNormCmd(chassis_cmd);
  chassis_ptr_->setWorkingMode(chassis_working_mode);
  chassis_ptr_->setGyroDir(gyro_dir);
  chassis_ptr_->setUseCapFlag(use_cap_flag);
  // TODO：掉头模式
  //  if (rev_head_flag) {
  //    chassis_ptr_->revHead();
  //  }

  // TODO：跟随模式云台前馈记录
  float omega_feedforward = hello_world::Bound(-rc_ptr_->rc_lh(), -1, 1);
  chassis_ptr_->setOmegaFeedforward(omega_feedforward);

  GimbalCmd gimbal_cmd;
  gimbal_cmd.pitch = hello_world::Bound(rc_ptr_->rc_lv(), -1, 1);
  gimbal_cmd.yaw = hello_world::Bound(-rc_ptr_->rc_lh(), -1,
                                      1); // 右手系，z轴竖直向上，左转为正
  gimbal_ptr_->setNormCmdDelta(gimbal_cmd);
  gimbal_ptr_->setCtrlMode(gimbal_ctrl_mode);
  gimbal_ptr_->setWorkingMode(gimbal_working_mode);
  // gimbal_ptr_->setRevHeadFlag(rev_head_flag); //TODO：掉头模式

  shooter_ptr_->setCtrlMode(shooter_ctrl_mode);
  shooter_ptr_->setWorkingMode(shooter_working_mode);
  shooter_ptr_->setShootFlag(shoot_flag);
};

void Robot::genModulesCmdFromKb() {
  // // TODO: 这里应该生成各个模块的指令，包括底盘指令、云台指令、底盘状态等等
  // // 指令应该通过各个模块的接口发送给各个模块
  Chassis::WorkingMode chassis_working_mode = chassis_ptr_->getWorkingMode();
  Gimbal::WorkingMode gimbal_working_mode = gimbal_ptr_->getWorkingMode();
  Shooter::WorkingMode shooter_working_mode = Shooter::WorkingMode::kStop;

  CtrlMode gimbal_ctrl_mode = CtrlMode::kManual;
  CtrlMode shooter_ctrl_mode = CtrlMode::kManual;

  bool use_cap_flag = false; // TODO待加入超级电容
  bool shoot_flag = false;
  // bool rev_head_flag = false; //TODO:掉头模式

  if (rc_ptr_->key_Q()) {
    chassis_working_mode = Chassis::WorkingMode::Gyro;
  } else if (rc_ptr_->key_E()) {
    chassis_working_mode = Chassis::WorkingMode::Follow;
  } else if (rc_ptr_->key_C()) {
    chassis_working_mode = Chassis::WorkingMode::Depart;
  } else {
    // if (chassis_working_mode == Chassis::WorkingMode::Depart) {
    //   chassis_working_mode = Chassis::WorkingMode::Follow;
    // } //TODO:待优化控制逻辑
  }

  if (rc_ptr_->key_X()) {
    // rev_head_flag = true; //TODO：掉头模式
  }
  if (rc_ptr_->key_Z()) {
    shooter_working_mode = Shooter::WorkingMode::kBackward;
  }

  if (rc_ptr_->mouse_l_btn()) {
    shoot_flag = true;
  }
  if (rc_ptr_->mouse_r_btn()) {
    gimbal_ctrl_mode = CtrlMode::kAuto;
    shooter_ctrl_mode = CtrlMode::kAuto;
  }

  ChassisCmd chassis_cmd = {0};
  // ChassisCmd chassis_cmd_raw = {0};
  // chassis_cmd_raw.v_x = hello_world::Bound(
  //     ((int8_t)rc_ptr_->key_W() - (int8_t)rc_ptr_->key_S()), -1, 1);
  // chassis_cmd_raw.v_y = hello_world::Bound(
  //     ((int8_t)rc_ptr_->key_A() - (int8_t)rc_ptr_->key_D()), -1, 1);
  // ramp_cmd_vx_ptr_->calc(&(chassis_cmd_raw.v_x), &(chassis_cmd.v_x));
  // ramp_cmd_vy_ptr_->calc(&(chassis_cmd_raw.v_y), &(chassis_cmd.v_y));
  chassis_cmd.v_x = hello_world::Bound(
      ((int8_t)rc_ptr_->key_W() - (int8_t)rc_ptr_->key_S()), -1, 1);
  chassis_cmd.v_y = hello_world::Bound(
      ((int8_t)rc_ptr_->key_A() - (int8_t)rc_ptr_->key_D()), -1, 1);
  chassis_cmd.w = 0.0f;
  chassis_ptr_->setNormCmd(chassis_cmd);
  chassis_ptr_->setWorkingMode(chassis_working_mode);
  chassis_ptr_->setUseCapFlag(rc_ptr_->key_SHIFT());
  // TODO：掉头模式
  //  if (rev_head_flag)
  //    chassis_ptr_->revHead();

  GimbalCmd gimbal_cmd;
  gimbal_cmd.pitch = hello_world::Bound(-0.01 * rc_ptr_->mouse_y(), -1, 1);
  gimbal_cmd.yaw = hello_world::Bound(-0.01 * rc_ptr_->mouse_x(), -1, 1);
  gimbal_ptr_->setNormCmdDelta(gimbal_cmd);
  gimbal_ptr_->setCtrlMode(gimbal_ctrl_mode);
  gimbal_ptr_->setWorkingMode(gimbal_working_mode);
  // TODO：掉头模式
  //  gimbal_ptr_->setRevHeadFlag(rev_head_flag);

  shooter_ptr_->setCtrlMode(shooter_ctrl_mode);
  shooter_ptr_->setWorkingMode(shooter_working_mode);
  shooter_ptr_->setShootFlag(shoot_flag);
};

#pragma endregion

#pragma region 数据重置函数
void Robot::reset() {
  pwr_state_ = PwrState::kDead;      ///< 电源状态
  last_pwr_state_ = PwrState::kDead; ///< 上一电源状态

  manual_ctrl_src_ = ManualCtrlSrc::kRc;      ///< 手动控制源
  last_manual_ctrl_src_ = ManualCtrlSrc::kRc; ///< 上一手动控制源
};
void Robot::resetDataOnDead() {
  pwr_state_ = PwrState::kDead;      ///< 电源状态
  last_pwr_state_ = PwrState::kDead; ///< 上一电源状态

  manual_ctrl_src_ = ManualCtrlSrc::kRc;      ///< 手动控制源
  last_manual_ctrl_src_ = ManualCtrlSrc::kRc; ///< 上一手动控制源
};
void Robot::resetDataOnResurrection() {};

#pragma endregion

#pragma region 通信数据设置函数
void Robot::setCommData() {
  // TODO(ZSC): 可能以后会在这里分工作状态
  setGimbalChassisCommData();
  setUiDrawerData();
  // TODO(ZSC): 可能的其他通讯数据设置函数
  // 其他通讯模块的数据由各个子模块负责设置
  // 主控板非工作模式时，这些数据保持默认值
};

void Robot::setGimbalChassisCommData() {
  HW_ASSERT(gc_comm_ptr_ != nullptr, "GimbalChassisComm pointer is null",
            gc_comm_ptr_);
  HW_ASSERT(gimbal_ptr_ != nullptr, "Gimbal pointer is null", gimbal_ptr_);
  HW_ASSERT(shooter_ptr_ != nullptr, "Shooter pointer is null", shooter_ptr_);

  // gimbal
  GimbalChassisComm::GimbalData::ChassisPart &gimbal_data =
      gc_comm_ptr_->gimbal_data().cp;
  // gimbal_data.turn_back_flag = gimbal_ptr_->getRevHeadFlag();
  // //TODO：掉头模式
  gimbal_data.yaw_delta = gimbal_ptr_->getNormCmdDelta().yaw;
  gimbal_data.pitch_delta = gimbal_ptr_->getNormCmdDelta().pitch;
  gimbal_data.ctrl_mode = gimbal_ptr_->getCtrlMode();
  gimbal_data.working_mode = gimbal_ptr_->getWorkingMode();

  // shooter
  GimbalChassisComm::ShooterData::ChassisPart &shooter_data =
      gc_comm_ptr_->shooter_data().cp;
  shooter_data.setShootFlag(shooter_ptr_->getShootFlag());
  shooter_data.ctrl_mode = shooter_ptr_->getCtrlMode();
  shooter_data.working_mode = shooter_ptr_->getWorkingMode();
};

void Robot::setUiDrawerData() {
  // // Chassis
  // HW_ASSERT(chassis_fsm_ptr_ != nullptr, "Chassis FSM pointer is null",
  // chassis_fsm_ptr_);
  // ui_drawer_.setChassisPwrState(chassis_fsm_ptr_->pwr_state());
  // ui_drawer_.setChassisCtrlMode(chassis_fsm_ptr_->ctrl_mode());
  // ui_drawer_.setChassisWorkingMode(chassis_fsm_ptr_->working_mode());
  // ui_drawer_.setChassisManualCtrlSrc(chassis_fsm_ptr_->manual_ctrl_src());
  // ui_drawer_.setChassisHeadDir(chassis_fsm_ptr_->theta_i2r());

  // // Gimbal
  // HW_ASSERT(gimbal_fsm_ptr_ != nullptr, "Gimbal FSM pointer is null",
  // gimbal_fsm_ptr_);
  // ui_drawer_.setGimbalPwrState(gimbal_fsm_ptr_->pwr_state());
  // ui_drawer_.setGimbalCtrlMode(gimbal_fsm_ptr_->ctrl_mode());
  // ui_drawer_.setGimbalWorkingMode(gimbal_fsm_ptr_->working_mode());
  // ui_drawer_.setGimbalManualCtrlSrc(gimbal_fsm_ptr_->manual_ctrl_src());
  // ui_drawer_.setGimbalJointAngPitchFdb(gc_comm_ptr_->gimbal_data().pitch_fdb);
  // ui_drawer_.setGimbalJointAngPitchRef(gc_comm_ptr_->gimbal_data().pitch_ref);
  // ui_drawer_.setGimbalJointAngYawFdb(gc_comm_ptr_->gimbal_data().yaw_fdb);
  // ui_drawer_.setGimbalJointAngYawRef(gc_comm_ptr_->gimbal_data().yaw_ref);

  // // Shooter
  // HW_ASSERT(shooter_fsm_ptr_ != nullptr, "Shooter FSM pointer is null",
  // shooter_fsm_ptr_);
  // ui_drawer_.setShooterPwrState(shooter_fsm_ptr_->pwr_state());
  // ui_drawer_.setShooterCtrlMode(shooter_fsm_ptr_->ctrl_mode());
  // ui_drawer_.setShooterWorkingMode(shooter_fsm_ptr_->working_mode());
  // ui_drawer_.setShooterManualCtrlSrc(shooter_fsm_ptr_->manual_ctrl_src());
  // ui_drawer_.setHeat(shooter_fsm_ptr_->heat());
  // ui_drawer_.setHeatLimit(shooter_fsm_ptr_->heat_limit());

  // ui_drawer_.setFeedAngFdb(shooter_fsm_ptr_->feed_ang_fdb());
  // ui_drawer_.setFeedAngRef(shooter_fsm_ptr_->feed_ang_ref());
  // ui_drawer_.setFeedStuckFlag(shooter_fsm_ptr_->is_feed_stuck());
  // ui_drawer_.setFricSpdFdb(gc_comm_ptr_->shooter_data().fric_spd_fdb);
  // ui_drawer_.setFricSpdRef(shooter_fsm_ptr_->fric_spd_ref());
  // ui_drawer_.setFricStuckFlag(shooter_fsm_ptr_->is_fric_stuck());

  // // MiniGimbal
  // HW_ASSERT(mini_gimbal_fsm_ptr_ != nullptr, "MiniGimbal FSM pointer is
  // null", mini_gimbal_fsm_ptr_);
  // ui_drawer_.setMiniGimbalPwrState(mini_gimbal_fsm_ptr_->pwr_state());
  // ui_drawer_.setMiniGimbalCtrlMode(mini_gimbal_fsm_ptr_->ctrl_mode());
  // ui_drawer_.setMiniGimbalWorkingMode(mini_gimbal_fsm_ptr_->working_mode());
  // ui_drawer_.setMiniGimbalManualCtrlSrc(mini_gimbal_fsm_ptr_->manual_ctrl_src());
  // ui_drawer_.setMiniPitchAngNow(gc_comm_ptr_->gimbal_data().mini_pitch_);
  // // ui_drawer_.setMiniPitchPreSet();

  // // Cap
  // HW_ASSERT(cap_ptr_ != nullptr, "Cap pointer is null", cap_ptr_);
  // ui_drawer_.setCapPwrPercent(cap_ptr_->getRemainingPowerPercent());

  // // vision
  // HW_ASSERT(gc_comm_ptr_ != nullptr, "GimbalChassisComm pointer is null",
  // gc_comm_ptr_);
  // bool is_vision_valid = gc_comm_ptr_->vision_data().detected_targets != 0;
  // ui_drawer_.setVisTgtX(gc_comm_ptr_->vision_data().vtm_x, is_vision_valid);
  // ui_drawer_.setVisTgtY(gc_comm_ptr_->vision_data().vtm_y, is_vision_valid);

  // if (rc_ptr_->key_V()) {
  //   ui_drawer_.refresh();
  // }

  // ui_drawer_.setVisTgtX();
};

#pragma endregion

#pragma region 通信数据发送函数

void Robot::sendCommData() {
  sendCanData();
  sendUsartData();
};
void Robot::sendCanData() {
  if (work_tick_ % 10 == 0) {
    // sendCapData(); //TODO：待设置超电通信
  }
  if (work_tick_ > 1000) {
    sendGimbalChassisCommData();
  }
  if (work_tick_ > 2000) {
    sendYawMotorData();
    sendWheelMotorsData();
  }
};
void Robot::sendWheelMotorsData() {
  WheelMotorIdx motor_idx[4] = {
      kWheelMotorIdxLeftFront,  ///< 左前轮电机下标
      kWheelMotorIdxLeftRear,   ///< 左后轮电机下标
      kWheelMotorIdxRightRear,  ///< 右后轮电机下标
      kWheelMotorIdxRightFront, ///< 右前轮电机下标
  };
  TxDevIdx tx_dev_idx[4] = {
      TxDevIdx::kMotorWheelLeftFront,  ///< 左前轮电机下标
      TxDevIdx::kMotorWheelLeftRear,   ///< 左后轮电机下标
      TxDevIdx::kMotorWheelRightRear,  ///< 右后轮电机下标
      TxDevIdx::kMotorWheelRightFront, ///< 右前轮电机下标
  };
  Motor *dev_ptr = nullptr;
  for (size_t i = 0; i < 4; i++) {
    WheelMotorIdx midx = motor_idx[i];
    dev_ptr = motor_wheels_ptr_[midx];
    HW_ASSERT(dev_ptr != nullptr, "Wheel Motor pointer %d is null", midx);
    tx_dev_mgr_pairs_[(uint32_t)tx_dev_idx[i]].setTransmitterNeedToTransmit();
  }
};
void Robot::sendYawMotorData() {
  HW_ASSERT(motor_yaw_ptr_ != nullptr, "Yaw Motor pointer is null",
            motor_yaw_ptr_);
  tx_dev_mgr_pairs_[(uint32_t)TxDevIdx::kMotorYaw]
      .setTransmitterNeedToTransmit();
}
void Robot::sendCapData() {
  HW_ASSERT(cap_ptr_ != nullptr, "Cap pointer is null", cap_ptr_);
  tx_dev_mgr_pairs_[(uint32_t)TxDevIdx::kCap].setTransmitterNeedToTransmit();
};
void Robot::sendGimbalChassisCommData() {
  HW_ASSERT(gc_comm_ptr_ != nullptr, "GimbalChassisComm pointer is null",
            gc_comm_ptr_);
  tx_dev_mgr_pairs_[(uint32_t)TxDevIdx::kGimbalChassis]
      .setTransmitterNeedToTransmit();
};
void Robot::sendUsartData() {
  if (work_tick_ % 5 == 0) {
    sendRefereeData();
  }
};
void Robot::sendRefereeData() {
  // if (ui_drawer_.encode(rfr_tx_data_, rfr_tx_data_len_)) {
  //   HAL_UART_Transmit_DMA(&huart6, rfr_tx_data_, rfr_tx_data_len_);
  // }
  tx_dev_mgr_pairs_[(uint32_t)TxDevIdx::kReferee]
      .setTransmitterNeedToTransmit();
};
#pragma endregion

#pragma region 注册函数

void Robot::registerChassis(Chassis *ptr) {
  HW_ASSERT(ptr != nullptr, "Chassis pointer is null", ptr);
  chassis_ptr_ = ptr;
};
void Robot::registerGimbal(Gimbal *ptr) {
  HW_ASSERT(ptr != nullptr, "Gimbal pointer is null", ptr);
  gimbal_ptr_ = ptr;
};
void Robot::registerShooter(Shooter *ptr) {
  HW_ASSERT(ptr != nullptr, "Shooter pointer is null", ptr);
  shooter_ptr_ = ptr;
};
void Robot::registerRampCmdVx(Ramp *ptr) {
  HW_ASSERT(ptr != nullptr, "RampCmdVx pointer is null", ptr);
  ramp_cmd_vx_ptr_ = ptr;
};
void Robot::registerRampCmdVy(Ramp *ptr) {
  HW_ASSERT(ptr != nullptr, "RampCmdVy pointer is null", ptr);
  ramp_cmd_vy_ptr_ = ptr;
};
void Robot::registerBuzzer(Buzzer *ptr) {
  HW_ASSERT(ptr != nullptr, "Buzzer pointer is null", ptr);
  buzzer_ptr_ = ptr;
};
void Robot::registerImu(Imu *ptr) {
  HW_ASSERT(ptr != nullptr, "IMU pointer is null", ptr);
  imu_ptr_ = ptr;
};

void Robot::registerMotorWheels(Motor *dev_ptr, uint8_t idx,
                                CanTxMgr *tx_mgr_ptr) {
  HW_ASSERT(dev_ptr != nullptr, "Wheel Motor pointer is null", dev_ptr);
  HW_ASSERT(tx_mgr_ptr != nullptr, "CanTxMgr pointer is null", tx_mgr_ptr);
  HW_ASSERT(idx >= 0 && idx < kWheelMotorNum,
            "Wheel Motor index is out of range", idx);
  if (idx >= kWheelMotorNum || motor_wheels_ptr_[idx] == dev_ptr) {
    return;
  }

  WheelMotorIdx midx[4] = {
      WheelMotorIdx::kWheelMotorIdxLeftFront,
      WheelMotorIdx::kWheelMotorIdxLeftRear,
      WheelMotorIdx::kWheelMotorIdxRightRear,
      WheelMotorIdx::kWheelMotorIdxRightFront,
  };
  TxDevIdx tidx[4] = {
      TxDevIdx::kMotorWheelLeftFront,
      TxDevIdx::kMotorWheelLeftRear,
      TxDevIdx::kMotorWheelRightRear,
      TxDevIdx::kMotorWheelRightFront,
  };
  motor_wheels_ptr_[midx[idx]] = dev_ptr;
  tx_dev_mgr_pairs_[(uint32_t)tidx[idx]].transmitter_ptr_ = dev_ptr;
  tx_dev_mgr_pairs_[(uint32_t)tidx[idx]].tx_mgr_ptr_ = tx_mgr_ptr;
};

void Robot::registerMotorYaw(Motor *dev_ptr, CanTxMgr *tx_mgr_ptr) {
  HW_ASSERT(dev_ptr != nullptr, "Yaw Motor pointer is null", dev_ptr);
  HW_ASSERT(tx_mgr_ptr != nullptr, "CanTxMgr pointer is null", tx_mgr_ptr);
  if (motor_yaw_ptr_ == dev_ptr) {
    return;
  }
  motor_yaw_ptr_ = dev_ptr;
  tx_dev_mgr_pairs_[(uint32_t)TxDevIdx::kMotorYaw].transmitter_ptr_ = dev_ptr;
  tx_dev_mgr_pairs_[(uint32_t)TxDevIdx::kMotorYaw].tx_mgr_ptr_ = tx_mgr_ptr;
}

void Robot::registerCap(Cap *dev_ptr, CanTxMgr *tx_mgr_ptr) {
  HW_ASSERT(dev_ptr != nullptr, "Cap pointer is null", dev_ptr);
  HW_ASSERT(tx_mgr_ptr != nullptr, "CanTxMgr pointer is null", tx_mgr_ptr);
  if (cap_ptr_ == dev_ptr) {
    return;
  }

  cap_ptr_ = dev_ptr;
  tx_dev_mgr_pairs_[(uint32_t)TxDevIdx::kCap].transmitter_ptr_ = dev_ptr;
  tx_dev_mgr_pairs_[(uint32_t)TxDevIdx::kCap].tx_mgr_ptr_ = tx_mgr_ptr;
};

void Robot::registerGimbalChassisComm(GimbalChassisComm *dev_ptr,
                                      CanTxMgr *tx_mgr_ptr) {
  HW_ASSERT(dev_ptr != nullptr, "GimbalChassisComm pointer is null", dev_ptr);
  HW_ASSERT(tx_mgr_ptr != nullptr, "CanTxMgr pointer is null", tx_mgr_ptr);
  if (gc_comm_ptr_ == dev_ptr) {
    return;
  }

  gc_comm_ptr_ = dev_ptr;
  tx_dev_mgr_pairs_[(uint32_t)TxDevIdx::kGimbalChassis].transmitter_ptr_ =
      dev_ptr;
  tx_dev_mgr_pairs_[(uint32_t)TxDevIdx::kGimbalChassis].tx_mgr_ptr_ =
      tx_mgr_ptr;
};

void Robot::registerReferee(Referee *dev_ptr, UartTxMgr *tx_mgr_ptr) {
  HW_ASSERT(dev_ptr != nullptr, "Referee pointer is null", dev_ptr);
  HW_ASSERT(tx_mgr_ptr != nullptr, "UartRxMgr pointer is null", tx_mgr_ptr);
  if (referee_ptr_ == dev_ptr) {
    return;
  }

  // auto iter = transmitter_ptr_switch_map_.find(referee_ptr_);
  // if (iter != transmitter_ptr_switch_map_.end()) {
  //   transmitter_ptr_switch_map_.erase(iter);
  // }
  referee_ptr_ = dev_ptr;
  tx_dev_mgr_pairs_[(uint32_t)TxDevIdx::kReferee].transmitter_ptr_ = dev_ptr;
  tx_dev_mgr_pairs_[(uint32_t)TxDevIdx::kReferee].tx_mgr_ptr_ = tx_mgr_ptr;
};

void Robot::registerRc(DT7 *ptr) {
  HW_ASSERT(ptr != nullptr, "DT7 pointer is null", ptr);
  if (rc_ptr_ == ptr) {
    return;
  }
  rc_ptr_ = ptr;
}

void Robot::registerPerformancePkg(PerformancePkg *ptr) {
  HW_ASSERT(ptr != nullptr, "PerformancePkg pointer is null", ptr);
  if (rfr_performance_pkg_ptr_ == ptr) {
    return;
  }

  rfr_performance_pkg_ptr_ = ptr;
};
void Robot::registerPowerHeatPkg(PowerHeatPkg *ptr) {
  HW_ASSERT(ptr != nullptr, "PowerHeatPkg pointer is null", ptr);
  if (rfr_power_heat_pkg_ptr_ == ptr) {
    return;
  }
  rfr_power_heat_pkg_ptr_ = ptr;
};
void Robot::registerShooterPkg(ShooterPkg *ptr) {
  HW_ASSERT(ptr != nullptr, "ShooterPkg pointer is null", ptr);
  if (rfr_shooter_pkg_ptr_ == ptr) {
    return;
  }
  rfr_shooter_pkg_ptr_ = ptr;
};
#pragma endregion
/* Private function definitions
 * ----------------------------------------------*/
} // namespace robot