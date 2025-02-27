/**
 *******************************************************************************
 * @file      :robot.hpp
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
#ifndef ROBOT_MODULES_ROBOT_HPP_
#define ROBOT_MODULES_ROBOT_HPP_

/* Includes ------------------------------------------------------------------*/
#include "fsm.hpp"
#include "ramp.hpp"
#include "tick.hpp"

#include "DT7.hpp"
#include "buzzer.hpp"
#include "imu.hpp"
#include "motor.hpp"
#include "super_cap.hpp"

#include "can_tx_mgr.hpp"
#include "referee.hpp"
#include "transmitter.hpp"
#include "uart_rx_mgr.hpp"
#include "uart_tx_mgr.hpp"

#include "module_fsm.hpp"

#include "gimbal_chassis_comm.hpp"

#include "chassis.hpp"
#include "gimbal.hpp"
#include "shooter.hpp"

/* Exported macro ------------------------------------------------------------*/

namespace robot {
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

class Robot : public hello_world::module::ModuleFsm {
public:
  typedef hello_world::filter::Ramp Ramp;
  typedef hello_world::buzzer::Buzzer Buzzer;
  typedef hello_world::cap::SuperCap Cap;
  typedef hello_world::imu::Imu Imu;
  typedef hello_world::motor::Motor Motor;
  typedef hello_world::comm::Transmitter Transmitter;
  typedef hello_world::comm::CanTxMgr CanTxMgr;
  typedef hello_world::comm::UartTxMgr UartTxMgr;
  typedef hello_world::comm::TxMgr TxMgr;
  typedef hello_world::remote_control::DT7 DT7;
  typedef hello_world::remote_control::SwitchState RcSwitchState;

  typedef hello_world::referee::RobotPerformancePackage PerformancePkg;
  typedef hello_world::referee::RobotPowerHeatPackage PowerHeatPkg;
  typedef hello_world::referee::RobotShooterPackage ShooterPkg;
  typedef hello_world::referee::Referee Referee;
  typedef hello_world::referee::RfrId RobotId;

  typedef hello_world::module::PwrState PwrState;
  typedef hello_world::module::CtrlMode CtrlMode;
  typedef hello_world::module::ManualCtrlSrc ManualCtrlSrc;

  typedef robot::Chassis Chassis;
  typedef robot::Gimbal Gimbal;
  typedef robot::Shooter Shooter;

  class TxDevMgrPair {
  public:
    TxDevMgrPair(Transmitter *transmitter_ptr = nullptr,
                 TxMgr *tx_mgr_ptr = nullptr)
        : transmitter_ptr_(transmitter_ptr), tx_mgr_ptr_(tx_mgr_ptr) {};
    TxDevMgrPair(const TxDevMgrPair &other) = default;
    TxDevMgrPair &operator=(const TxDevMgrPair &other) = default;
    TxDevMgrPair(TxDevMgrPair &&other) = default;
    TxDevMgrPair &operator=(TxDevMgrPair &&other) = default;

    ~TxDevMgrPair() = default;

    void setTransmitterNeedToTransmit(void) {
      if (transmitter_ptr_ == nullptr || tx_mgr_ptr_ == nullptr) {
        return;
      }

      tx_mgr_ptr_->setTransmitterNeedToTransmit(transmitter_ptr_);
    }

  private:
    friend class Robot;
    Transmitter *transmitter_ptr_ = nullptr;
    TxMgr *tx_mgr_ptr_ = nullptr;
  };

  enum class TxDevIdx : uint8_t {
    kGimbalChassis,        ///< 云台与底盘通信开关
    kMotorWheelLeftFront,  ///< 左前轮电机通信开关
    kMotorWheelLeftRear,   ///< 左后轮电机通信开关
    kMotorWheelRightRear,  ///< 右后轮电机通信开关
    kMotorWheelRightFront, ///< 右前轮电机通信开关
    kMotorYaw,             ///< 云台电机通信开关
    kCap,                  ///< 超级电容通信开关
    kReferee,              ///< 裁判系统通信开关
    kNum,                  ///< 通信开关数量
  };

  enum WheelMotorIdx : uint8_t {
    kWheelMotorIdxLeftFront,  ///< 左前轮电机下标
    kWheelMotorIdxLeftRear,   ///< 左后轮电机下标
    kWheelMotorIdxRightRear,  ///< 右后轮电机下标
    kWheelMotorIdxRightFront, ///< 右前轮电机下标
    kWheelMotorNum,           ///< 轮电机数量
  };

public:
  Robot() {};
  ~Robot() {};

  // 状态机主要接口函数
  void update() override;

  void runOnDead() override;
  void runOnResurrection() override;
  void runOnWorking() override;
  void runAlways() override;

  void reset() override;
  void standby() override;

  void registerChassis(Chassis *ptr);
  void registerGimbal(Gimbal *ptr);
  void registerShooter(Shooter *ptr);

  void registerRampCmdVx(Ramp *ptr);
  void registerRampCmdVy(Ramp *ptr);
  void registerBuzzer(Buzzer *ptr);
  void registerCap(Cap *ptr, CanTxMgr *tx_mgr_ptr);
  void registerImu(Imu *ptr);
  void registerMotorWheels(Motor *motor_ptr, uint8_t idx, CanTxMgr *tx_mgr_ptr);
  void registerMotorYaw(Motor *ptr, CanTxMgr *tx_mgr_ptr);
  void registerGimbalChassisComm(GimbalChassisComm *ptr, CanTxMgr *tx_mgr_ptr);
  void registerReferee(Referee *ptr, UartTxMgr *tx_mgr_ptr);
  void registerRc(DT7 *ptr);

  void registerPerformancePkg(PerformancePkg *ptr);
  void registerPowerHeatPkg(PowerHeatPkg *ptr);
  void registerShooterPkg(ShooterPkg *ptr);

private:
  //  数据更新和工作状态更新，由 update 函数调用
  void updateData();
  void updateImuData();
  void updateRfrData();
  void updateRcData();
  void updatePwrState();

  void genModulesCmd();

  void genModulesCmdFromRc();
  void genModulesCmdFromKb();

  // 设置通讯组件数据函数
  void setCommData();
  void setGimbalChassisCommData();
  void setUiDrawerData();

  // 重置数据函数
  void resetDataOnDead();
  void resetDataOnResurrection();

  // 发送通讯组件数据函数
  void sendCommData();
  void sendCanData();
  void sendWheelMotorsData();
  void sendYawMotorData();
  void sendCapData();
  void sendGimbalChassisCommData();
  void sendRefereeData();
  void sendUsartData();

  void setManualCtrlSrc(ManualCtrlSrc src) {
    if (src != manual_ctrl_src_) {
      last_manual_ctrl_src_ = manual_ctrl_src_;
      manual_ctrl_src_ = src;
    }
  };

  // IMU 数据在 update 函数中更新
  bool is_imu_caled_offset_ = false; ///< IMU 数据是否计算完零飘

  // RC 数据在 update 函数中更新
  ManualCtrlSrc manual_ctrl_src_ = ManualCtrlSrc::kRc;      ///< 手动控制源
  ManualCtrlSrc last_manual_ctrl_src_ = ManualCtrlSrc::kRc; ///< 上一手动控制源

  // 主要模块状态机组件指针
  Chassis *chassis_ptr_ = nullptr; ///< 底盘模块指针
  Gimbal *gimbal_ptr_ = nullptr;   ///< 云台模块指针
  Shooter *shooter_ptr_ = nullptr; ///< 发射模块指针

  // 无通信功能的组件指针
  Buzzer *buzzer_ptr_ = nullptr; ///< 蜂鸣器指针
  Imu *imu_ptr_ = nullptr;       ///< 底盘 IMU 指针
  Ramp *ramp_cmd_vx_ptr_ = nullptr; ///< Vx斜坡滤波指针
  Ramp *ramp_cmd_vy_ptr_ = nullptr; ///< Vy斜坡滤波指针

  // 只接收数据的组件指针
  DT7 *rc_ptr_ = nullptr; ///< DT7 指针 只接收数据

  // 只发送数据的组件指针
  Cap *cap_ptr_ = nullptr; ///< 底盘超级电容指针 只发送数据

  Motor *motor_wheels_ptr_[kWheelMotorNum] = {
      nullptr}; ///< 四轮电机指针 只发送数据
  Motor *motor_yaw_ptr_ = nullptr; ///< 云台电机指针 只发送数据
  // 收发数据的组件指针
  GimbalChassisComm *gc_comm_ptr_ = nullptr; ///< 云台底盘通信模块指针 收发数据
  Referee *referee_ptr_ = nullptr;           ///< 裁判系统指针 收发数据
  PerformancePkg *rfr_performance_pkg_ptr_ =
      nullptr; ///< 裁判系统性能包指针 收发数据
  PowerHeatPkg *rfr_power_heat_pkg_ptr_ =
      nullptr; ///< 裁判系统电源和热量包指针 收发数据
  ShooterPkg *rfr_shooter_pkg_ptr_ = nullptr; ///< 裁判系统射击包指针 收发数据

  TxDevMgrPair tx_dev_mgr_pairs_[(uint32_t)TxDevIdx::kNum] = {
      {nullptr}}; ///< 发送设备管理器对数组
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
} // namespace robot
#endif /* ROBOT_MODULE_ROBOT_HPP_ */
