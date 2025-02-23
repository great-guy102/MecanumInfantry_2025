/**
 *******************************************************************************
 * @file      : feed.hpp
 * @brief     : 拨弹盘模块组件
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
#ifndef HW_COMPONENTS_MODULES_FEED_HPP_
#define HW_COMPONENTS_MODULES_FEED_HPP_

/* Includes ------------------------------------------------------------------*/
#include "allocator.hpp"
#include "module_fsm.hpp"
#include "motor.hpp"
#include "pid.hpp"
#include "vision.hpp"
/* Exported macro ------------------------------------------------------------*/
namespace hello_world
{
namespace module
{
namespace feed_impl
{
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/* 卡弹状态 (枚举类) */
enum class FeedStuckStatus : uint8_t {
  kNone = 0u,  // 没有卡住
  kForward,    // 向前卡住
  kBackward,   // 向后卡住
};
/* 裁判系统输入数据 (结构体) */
struct FeedRfrInputData {
  bool is_rfr_on = false;           // 裁判系统是否在线
  bool is_power_on = false;         // 发射机构电源是否开启
  bool is_new_bullet_shot = false;  // 是否有新弹丸射出
  float heat_limit = 100.0;         // 发射机构热量限制
  float heat = 0;                   // 发射机构热量
  float heat_cooling_ps = 10;       // 发射机构热量的每秒冷却值
};
/* 配置参数 (结构体) */
struct FeedConfig {
  float ang_ref_offset = 0.0f;  // 拨盘电机目标角度偏移量, >=0, 单位 rad(TODO 根据弹链长度调整)
  float ang_per_blt;            // 1发弹丸拨盘的转动角度, >0, 单位 rad (TODO 根据拨弹盘设计调整)
  float heat_per_blt;           // 1发弹丸的耗热量, >0 (TODO 根据兵种调整)

  float stuck_curr_thre;                         // 用于判断堵转的电流阈值, >0, 单位 A (TODO 根据拨弹电机调整)
  float resurrection_pos_err = 5.0f / 180 * PI;  // Resurrection 模式反转速度系数, 反转时以它作为位置环PID的误差值, >0, 单位 rad
  uint32_t stuck_duration_thre = 200;            // 堵转检测阈值时间, >0, 单位 ms

  uint32_t hold_duration_thre = 100;  // 角度保持不变检测阈值时间, >0, 单位 ms (在复活模式作为堵转检测的第二手段)

  uint32_t default_trigger_interval = 200;  // 默认拨弹频率限制间隔时间, >0, 单位 ms
  float default_safe_num_blt = 1.5f;        // 默认安全热量值对应的弹丸个数, >=0, 可以为小数
};

/* 被放置于 hello_world::module 命名空间下 */
class Feed : public ModuleFsm
{
 public:
  /* 类型定义 */
  typedef pid::MultiNodesPid Pid;
  typedef motor::Motor Motor;
  typedef vision::Vision::ShootFlag ShootFlag;
  typedef FeedStuckStatus StuckStatus;    // 卡弹状态 (枚举类)
  typedef FeedRfrInputData RfrInputData;  // 裁判系统输入数据 (结构体)
  typedef FeedConfig Config;              // 配置参数 (结构体)
  /* 唯一构造函数 */
  explicit Feed(const Config &config);

  /* 析构函数 */
  ~Feed() {};

  /* 模块状态机虚函数重写 */
  void update() override;
  void reset() override;
  void standby() override;

  /* 更新/设定类内数据 */
  void updateRfrData(const RfrInputData &inp_data);  // 更新裁判系统数据, 需在外部调用
  void setConfig(const Config &config);               // 设定配置
  void setFricStatus(bool is_fric_ok) { is_fric_ok_ = is_fric_ok; }  // 设定摩擦轮运行状态
  void setCtrlMode(CtrlMode mode) { ctrl_mode_ = mode; }             // 设定控制模式
  void setTriggerLimit(bool is_trigger_allowed, bool is_heat_limited, float safe_num, uint32_t interval);
  void setManualShootFlag(bool flag) { manual_shoot_flag_ = flag; }  // 设定手动发弹信号
  void setVisionShootFlag(ShootFlag flag)                            // 设定视觉发弹信号
  {
    last_vision_shoot_flag_ = vision_shoot_flag_;
    vision_shoot_flag_ = flag;
  }
  /* 读取内部数据 */
  Config &getConfig() { return cfg_; }              // 获取当前配置
  const Config &getConfig() const { return cfg_; }  // 获取当前配置 (适用于 const 实例)
  CtrlMode getCtrlMode() const { return ctrl_mode_; }
  StuckStatus getStuckStatus() const { return status_.stuck_status; }
  float getAngRef() const { return status_.ang_ref; }
  float getAngFdb() const { return status_.ang_fdb; }
  uint16_t getTriggerCnt() const { return trigger_cnt_; }
  uint16_t getStuckForwardCnt() const { return stuck_forward_cnt_; }
  uint16_t getStuckBackwardCnt() const { return stuck_backward_cnt_; }
  /* 注册组件指针 */
  void registerMotorFeed(Motor *ptr)
  {
    HW_ASSERT(ptr != nullptr, "motor_ptr_ must be not null");
    motor_ptr_ = ptr;
  }
  void registerPidFeed(Pid *ptr)
  {
    HW_ASSERT(ptr != nullptr, "pid_ptr_ must be not null");
    pid_ptr_ = ptr;
  }

 private:
  /* 数据更新和工作状态更新，由update函数调用 */
  void updateData();
  void updateMotorData();
  void updateFakeHeat();
  void updateMotorStuckStatus();
  void updateMotorHoldStatus();

  void updatePwrState() override;

  /* 执行任务 */
  void runOnDead() override;
  void runOnResurrection() override;
  void runOnWorking() override;
  void runAlways() override;

  void genTriggerSignal();
  bool limitTriggerByFreq(uint32_t interval) const;
  bool limitTriggerByHeat(bool is_heat_limited, float safe_num) const;

  void calcFakeHeat();

  void calcAngRefOnResurrection();
  void calcAngRefOnWorking();
  void calcMotorInput();

  float searchAngRef(float fdb, bool is_forward) const;

  /* 重置通讯组件数据函数 */
  void resetDataOnDead();
  void resetPids();

  /* 设置通讯组件数据函数 */
  void setCommData(bool is_working);

  Config cfg_;

  /* 由robot设置的数据 */
  CtrlMode ctrl_mode_ = CtrlMode::kManual;                  // 控制模式
  bool manual_shoot_flag_ = false;                          // 手动射击标志位，由外部设置，由 runOnWorking 函数读取并执行
  ShootFlag last_vision_shoot_flag_ = ShootFlag::kNoShoot;  // 上一时刻自动射击指令，由外部设置，由 runOnWorking 函数读取并执行
  ShootFlag vision_shoot_flag_ = ShootFlag::kNoShoot;       // 自动射击指令，由外部设置，由 runOnWorking 函数读取并执行

  uint32_t trigger_interval_ = 200;  // 拨弹间隔时间，单位 ms
  float safe_num_bullet_ = 1.5;      // 安全热量值对应的弹丸个数
  bool is_heat_limited_ = true;      // 是否限制热量
  bool is_trigger_allowed_ = true;   // 是否允许拨弹
  /* 从摩擦轮中获得的数据 */
  bool is_fric_ok_ = false;  // 摩擦轮是否正常运行

  /* 内部管理数据 */
  bool is_power_on_ = false;  // 拨弹盘是否上电
  float fake_heat_ = 0.0f;    // 枪口'假'热量, 自行记录的枪口热量

  bool trigger_signal_ = false;     // 拨弹信号, 由 limitTriggerByFrequence 和 limitTriggerByHeat 函数生成
  uint32_t last_trigger_tick_ = 0;  // 上一次拨弹的时刻, 用于限制拨弹频率

  struct FeedStatus {
    /* update 函数中更新的数据 */
    StuckStatus stuck_status = StuckStatus::kNone;  // 卡住状态
    uint32_t stuck_duration = 0;                    // 卡住持续时间
    uint32_t ang_hold_duration = 0;                 // 拨盘角度不变持续时间
    /* run 函数中更新的数据 */
    bool is_ang_zero_inited = false;  // 是否确定电机角度零位 (假定拨盘电机没有绝对编码)
    bool is_ang_ref_on_grid = false;  // 拨盘电机角度期望值是否在网格上, 即 ang_ref = AngNormRad(ang_ref_offset + k*ang_per_blt) ?
    float ang_ref;                    // 拨盘电机目标角度
    /* 从电机中拿到的数据 */
    float ang_fdb;       // 拨盘电机反馈角度 单位 rad
    float last_ang_fdb;  // 上一控制周期拨盘电机反馈角度 单位 rad
    float spd_fdb;       // 拨盘电机反馈速度 单位 rad/s
    float cur_fdb;       // 拨盘电机反馈电流 单位 A
    float cur_ref;       // 拨盘电机输入信号 单位 A
    /* 工具函数 */
    void resetStuckStatus()
    {
      stuck_status = StuckStatus::kNone;
      stuck_duration = 0;
    };
    void resetHoldStatus() { ang_hold_duration = 0; };
  } status_;  // 拨盘状态

  /* 裁判系统相关数据, 在外部调用 updateRfrData() 时更新 */
  struct FeedRfrData {
    bool is_rfr_on = false;           // 裁判系统是否在线
    bool is_power_on = false;         // 发射机构电源是否开启
    bool is_new_bullet_shot = false;  // 是否有新弹丸射出
    float heat_limit = 100.0;         // 发射机构热量限制
    float heat = 0;                   // 发射机构热量
    float last_heat_ = 0;             // 上一次发射机构热量
    float heat_cooling_ps = 10;       // 发射机构热量的每秒冷却值
  } rfr_data_;                        // 裁判系统中与发射机构相关的数据

  /* 组件指针 */
  Motor *motor_ptr_ = nullptr;  // 拨盘电机指针，有通信功能
  Pid *pid_ptr_ = nullptr;      // 拨盘PID指针，无通信功能

  /* debug变量，供调试使用 */
  PwrState debug_pwr_state_ = PwrState::kDead;
  uint16_t trigger_cnt_ = 0;         // 拨弹计数器
  uint16_t stuck_forward_cnt_ = 0;   // 前向卡弹计数器
  uint16_t stuck_backward_cnt_ = 0;  // 后向卡弹计数器
};  // class Feed
}  // namespace feed_impl
using feed_impl::Feed;
}  // namespace module
}  // namespace hello_world
#endif /* HW_COMPONENTS_MODULES_FEED_HPP_ */