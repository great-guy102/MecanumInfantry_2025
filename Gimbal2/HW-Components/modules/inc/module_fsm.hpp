/**
 *******************************************************************************
 * @file      : module_fsm.hpp
 * @brief     : 模块组件状态机基类
 * @history   :
 *  Version     Date            Author                       Note
 *  V0.9.0      2024.12.12      ZhouShichan, LouKaiyang      1. 初版编写完成
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2025 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_MODULE_FSM_HPP_
#define HW_COMPONENTS_MODULE_FSM_HPP_

/* Includes ------------------------------------------------------------------*/
#include <string>

#include "allocator.hpp"
#include "motor.hpp"
#include "pid.hpp"
#include "tick.hpp"
/* Exported macro ------------------------------------------------------------*/
namespace hello_world
{
namespace module
{
/* Exported constants --------------------------------------------------------*/
// 通用状态枚举
/**
 * @brief 电源状态
 * 定义了与机器人模块相关的电源状态，包括死亡状态（断电）、复活状态（上电后的初始状态）和工作状态。
 */
enum class PwrState : uint8_t {
  kDead = 0u,     // 死亡状态
  kResurrection,  // 复活状态
  kWorking,       // 工作状态
};

/** 控制模式 */
enum class CtrlMode : uint8_t {
  kManual,  // 手动控制模式
  kAuto,    // 自动控制模式
};
/** 手动控制源 */
enum class ManualCtrlSrc : uint8_t {
  kRc,  // 遥控器操控
  kKb,  // 键盘操控
};
/* Exported types ------------------------------------------------------------*/

class ModuleFsm : public MemMgr
{
 public:
  /**
   * @brief 任务执行函数
   *
   * 负责根据当前状态和控制模式，执行相应的动作
   */
  void run()
  {
    if (pwr_state_ == PwrState::kDead) {
      runOnDead();
    } else if (pwr_state_ == PwrState::kResurrection) {
      runOnResurrection();
    } else if (pwr_state_ == PwrState::kWorking) {
      runOnWorking();
    }
    runAlways();
  }
  virtual void runOnDead() = 0;
  virtual void runOnResurrection() = 0;
  virtual void runOnWorking() = 0;
  virtual void runAlways() = 0;
  /**
   * @brief 进入待机状态
   *
   * 进入待机状态，停止所有动作
   */
  virtual void standby() = 0;
  /**
   * @brief 更新状态机
   *
   * 更新内部状态数据和状态机状态
   */
  virtual void update() = 0;
  /**
   * @brief 复位状态机
   *
   * 复位状态机，恢复初始状态
   */
  virtual void reset() = 0;

  // 接口函数
  virtual PwrState getPwrState() const { return pwr_state_; };
  virtual PwrState getLastPwrState() const { return last_pwr_state_; };

 protected:
  // 工具函数
  uint32_t getCurrentTickMs() const { return tick::GetTickMs(); };
  uint32_t updateWorkTick()
  {
    last_work_tick_ = work_tick_;
    work_tick_ = getCurrentTickMs();
    interval_ticks_ = work_tick_ - last_work_tick_;
    return work_tick_;
  }
  virtual void updatePwrState() = 0;
  void setPwrState(PwrState state)
  {
    if (state != pwr_state_) {
      last_pwr_state_ = pwr_state_;
      pwr_state_ = state;
    }
  };

  uint32_t work_tick_ = 0;       // 工作时钟，单位：ms
  uint32_t last_work_tick_ = 0;  // 上一工作时钟，单位：ms
  uint32_t interval_ticks_ = 0;  // 状态间隔时钟，单位：ms

  PwrState pwr_state_ = PwrState::kDead;       // 电源状态
  PwrState last_pwr_state_ = PwrState::kDead;  // 上一电源状态
};

class ModuleListManager : public MemMgr
{
 public:
  typedef motor::Motor Motor;
  typedef pid::MultiNodesPid Pid;
  typedef tools::list<Motor*> MotorList;
  typedef tools::list<Pid*> PidList;

  size_t motorSize() const { return motor_list_.size(); }
  size_t pidSize() const { return pid_list_.size(); }

  void registerMotor(Motor* ptr) { motor_list_.push_back(ptr); }
  void registerPid(Pid* ptr) { pid_list_.push_back(ptr); }

  void eraseMotorTail() { motor_list_.pop_back(); }
  void erasePidTail() { pid_list_.pop_back(); }

  void clearMotorList() { motor_list_.clear(); }
  void clearPidList() { pid_list_.clear(); }

  const Motor* getMotor(size_t idx) const
  {
    if (idx >= motorSize()) {
      return nullptr;
    }
    auto iter = motor_list_.begin();
    for (size_t i = 0; i < idx; i++) {
      iter++;
    }
    return *iter;
  }

  Motor* getMotor(size_t idx)
  {
    if (idx >= motorSize()) {
      return nullptr;
    }
    auto iter = motor_list_.begin();
    for (size_t i = 0; i < idx; i++) {
      iter++;
    }
    return *iter;
  }

  const Pid* getPid(size_t idx) const
  {
    if (idx >= pidSize()) {
      return nullptr;
    }
    auto iter = pid_list_.begin();
    for (size_t i = 0; i < idx; i++) {
      iter++;
    }
    return *iter;
  }

  Pid* getPid(size_t idx)
  {
    if (idx >= pidSize()) {
      return nullptr;
    }
    auto iter = pid_list_.begin();
    for (size_t i = 0; i < idx; i++) {
      iter++;
    }
    return *iter;
  }

 protected:
  MotorList motor_list_;  // 电机指针链表
  PidList pid_list_;      // PID指针链表
};

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
// 状态枚举转字符串函数，可用于 UI 绘制
inline std::string PwrStateToStr(PwrState state)
{
  if (state == PwrState::kDead)
    return "Dead";
  if (state == PwrState::kResurrection)
    return "Resurrection";
  if (state == PwrState::kWorking)
    return "Working";
  return "ErrWS";
};

inline std::string CtrlModeToStr(CtrlMode mode)
{
  if (mode == CtrlMode::kManual)
    return "Manual";
  if (mode == CtrlMode::kAuto)
    return "Auto";
  return "ErrCM";
};

inline std::string ManualCtrlSrcToStr(ManualCtrlSrc src)
{
  if (src == ManualCtrlSrc::kRc)
    return "Rc";
  if (src == ManualCtrlSrc::kKb)
    return "Kb";
  return "ErrMCS";
};

inline std::string CtrlModeSrcToStr(CtrlMode mode, ManualCtrlSrc src)
{
  if (mode == CtrlMode::kManual)
    return ManualCtrlSrcToStr(src);
  if (mode == CtrlMode::kAuto)
    return CtrlModeToStr(mode);
  return "ErrCM";
};
}  // namespace module
}  // namespace hello_world
#endif /* HW_COMPONENTS_MODULE_FSM_HPP_ */
