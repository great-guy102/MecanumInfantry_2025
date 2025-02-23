/**
 *******************************************************************************
 * @file      :imu.hpp
 * @brief     : IMU 设备组件
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2024-12-01      Jinletian       1. 初版编写完成
 *******************************************************************************
 * @attention :
 *  1.update前，需要先调用initHardware()，否则update会失败，
 *    该函数内部会阻塞运行，请不要在中断中调用
 *  2.ImuConfig中大部分参数给出了默认值，可根据实际需要修改
 *    旋转矩阵rot_mat_flatten和硬件配置bmi088_hw_config需要用户自行配置
 *  3.计算传感器零漂时，姿态角会根据当前零漂计算值持续更新
 *******************************************************************************
 *  Copyright (c) 2025 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_DEVICES_IMU_IMU_HPP_
#define HW_COMPONENTS_DEVICES_IMU_IMU_HPP_

/* Includes ------------------------------------------------------------------*/
#include "BMI088.hpp"
#include "allocator.hpp"
#include "mahony.hpp"

namespace hello_world
{
namespace imu
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

// IMU 工作状态
enum class ImuStatus : uint8_t {
  kHardwareNotInited,  ///< 硬件未初始化
  kCalcingOffset,      ///< 正在计算零漂
  kWorking,            ///< 正常工作
};

// IMU 配置参数
struct ImuConfig {
  /*/< 加速度阈值，用于过滤短时撞击，单位：m/s^2 */
  float acc_threshold = 10.0f;
  /*/< 角速度静止阈值，计算零漂时默认设备处于静止状态，滤去异常数据，单位：rad/s */
  float gyro_stationary_threshold = 0.1f;
  size_t sample_num = 1000;                                ///< 零漂采样次数
  float samp_freq = 1000.0f;                               ///< 采样频率
  float kp = 1.0f;                                         ///< Mahony 比例系数
  float ki = 0.0f;                                         ///< Mahony 积分系数
  float rot_mat_flatten[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};  ///< 旋转矩阵（根据 IMU 安装角度配置）
  BMI088HWConfig bmi088_hw_config;                         ///< BMI088 硬件配置（无默认值，根据引脚配置）
  BMI088Config bmi088_config = BMI088Config();             ///< BMI088 设备配置
};

class Imu : public MemMgr
{
 public:
  typedef ahrs::Mahony Mahony;
  typedef ImuStatus Status;
  typedef ImuConfig Config;

  // 唯一构造函数
  explicit Imu(const Config &config) { config_ = config; }
  ~Imu() = default;

  /**
   * @brief       初始化 IMU 硬件，update前请务必调用此函数
   * @retval       None
   * @note        该函数内部会阻塞运行，请不要在中断中调用
   */
  void initHardware();

  /**
   * @brief       更新 IMU 数据
   * @retval       是否更新成功，如果之前没有调用initHardware函数，该函数会返回false
   * @note        None
   */
  bool update();

  /**
   * @brief      重新计算角速度计零漂
   * @retval      None
   * @note
   */
  void resetOffset()
  {
    if (status_ != Status::kHardwareNotInited) {
      status_ = Status::kCalcingOffset;
    }
    sample_cnt_ = 0;
    memset(gyro_offset_.data, 0, sizeof(gyro_offset_.data));
  };

  /* 数据获取 */
  /**
   * @brief      判断 IMU 零漂是否计算完成
   * @retval      bool: IMU 零漂是否计算完成
   * @note        None
   */
  bool isOffsetCalcFinished() { return status_ == Status::kWorking; };

  // 获取姿态角 (ZYX欧拉角)
  float roll() const { return ang_.x; }
  float pitch() const { return ang_.y; }
  float yaw() const { return ang_.z; }

  // 获取角速度 (去零漂)
  float gyro_roll() const { return gyro_.x; }
  float gyro_pitch() const { return gyro_.y; }
  float gyro_yaw() const { return gyro_.z; }

  // 获取加速度 (过滤短时撞击)
  float acc_x() const { return acc_.x; }
  float acc_y() const { return acc_.y; }
  float acc_z() const { return acc_.z; }

  // 获取温度
  float temp() const {return temp_; }

 private:
  void getRawData();

  void calcOffset();

  void updateAccGyro();

  void updateMahony();

  /* IMU 三轴数据 */
  union Imu3AxisData {
    struct
    {
      float x;
      float y;
      float z;
    };
    float data[3];
  };

  /* 原始数据 */
  Imu3AxisData raw_acc_ = {0};   ///< 原始加速度，单位：m/s^2
  Imu3AxisData raw_gyro_ = {0};  ///< 原始角速度，单位：rad/s
  float temp_ = 0.0f;            ///< 温度，单位：℃

  /* 零漂计算 */
  Imu3AxisData gyro_offset_ = {0};  ///< 角速度计零漂，单位：rad/s
  size_t sample_cnt_ = 0;           ///< 采样计数器，用于计算角速度计零漂
  Config config_ = Config();        ///< 零漂计算配置参数

  /* 输出数据 */
  Imu3AxisData acc_ = {0};   ///< 三轴加速度（过滤短时撞击），单位：m/s^2
  Imu3AxisData gyro_ = {0};  ///< 三轴角速度（去零漂），单位：rad/s
  Imu3AxisData ang_ = {0};   ///< 姿态角度（ZYX欧拉角，顺序为roll, pitch, yaw），单位：rad

  /* 工作状态 */
  Status status_ = Status::kHardwareNotInited;

  /* 各组件指针 */
  BMI088 *bmi088_ptr_ = nullptr;  ///< 硬件接口
  Mahony *mahony_ptr_ = nullptr;  ///< 姿态算法
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

}  // namespace imu
}  // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_IMU_IMU_HPP_ */
