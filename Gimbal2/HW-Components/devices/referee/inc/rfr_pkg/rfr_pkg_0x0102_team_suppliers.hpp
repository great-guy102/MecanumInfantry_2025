/**
 * @file      rfr_pkg_0x0102_team_suppliers.hpp
 * @author    ZhouShichan (zsc19823382069@163.com)
 * @date      2024-01-25
 * @brief
 * @par last editor  ZhouShichan (zsc19823382069@163.com)
 * @version   1.0.0
 *
 * @copyright Copyright (c) 2024 Hello World Team, Zhejiang University. All Rights Reserved.
 *
 * @attention
 *
 * @par history
 * | Version | Date | Author | Description |
 * | :---: | :---: | :---: | :---: |
 * | 1.0.0 | 2024-02-18 | ZhouShichan | 首次完成 |
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0102_TEAM_SUPPLIERS_HPP_
#define HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0102_TEAM_SUPPLIERS_HPP_

/* Includes ------------------------------------------------------------------*/
#include "rfr_pkg_core.hpp"

namespace hello_world
{

namespace referee
{
/* Exported macro ------------------------------------------------------------*/
/**
 * @enum SuppliedRobotId
 * @brief 补充弹丸的机器人 ID 枚举
 */
enum class SuppliedRobotId : uint8_t {
  kNone = 0u,             ///< 当前无机器人补弹
  kRedHero = 1u,          ///< 红方英雄机器人补弹
  kRedStandard3 = 3u,     ///< 红方步兵机器人补弹
  kRedStandard4 = 4u,     ///< 红方步兵机器人补弹
  kRedStandard5 = 5u,     ///< 红方步兵机器人补弹
  kBlueHero = 101u,       ///< 蓝方英雄机器人补弹
  kBlueStandard3 = 103u,  ///< 蓝方步兵机器人补弹
  kBlueStandard4 = 104u,  ///< 蓝方步兵机器人补弹
  kBlueStandard5 = 105u,  ///< 蓝方步兵机器人补弹
};

/**
 * @enum SupplierStep
 * @brief 出弹口开闭状态枚举
 */
enum class SupplierStep : uint8_t {
  kClosed = 0u,     ///< 关闭
  kPreparing = 1u,  ///< 弹丸准备中
  kReleasing = 2u,  ///< 弹丸释放
};

/**
 * @enum SuppliedNumber
 * @brief 补弹数量枚举
 */
enum class SuppliedNumber : uint8_t {
  k50 = 50u,    ///< 50 颗弹丸
  k100 = 100u,  ///< 100 颗弹丸
  k150 = 150u,  ///< 150 颗弹丸
  k200 = 200u,  ///< 200 颗弹丸
};
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/**
 * @struct TeamSuppliersData
 * @brief 补给站动作标识数据
 */
struct __REFEREE_PACKED TeamSuppliersData {
  uint8_t reserved;       ///< 保留位
  uint8_t robot_id;       ///< 补弹机器人 ID
  uint8_t Supplier_step;  ///< 出弹口开闭状态
  uint8_t supplied_num;   ///< 补弹数量
};
static_assert(sizeof(TeamSuppliersData) == 4, "TeamSuppliersData size error");
/** @class TeamSuppliersPackage
 * @brief 补给站动作标识数据包
 *
 * 数据说明：
 * - 命令码：0x0102
 * - 数据长度：4
 * - 发送频率：补给站弹丸释放时触发发送
 * - 发送方/接收方：服务器->己方全体机器人
 * - 所属数据链路：常规链路
 */
class TeamSuppliersPackage : public ProtocolRxPackage
{
 public:
  typedef TeamSuppliersData Data;

  virtual CmdId getCmdId(void) const override { return 0x0102; }
  virtual DataLength getDataLength(void) const override { return sizeof(Data); }
  virtual uint32_t getMaxRxIntervalMs(void) const override
  {
    return FREQ2INTERVAL(0);
  }

  virtual bool decode(const CmdId &cmd_id, const uint8_t *data_ptr) override
  {
    if (cmd_id == getCmdId()) {
      memcpy(&data_, data_ptr, sizeof(Data));
      last_decode_tick_ = getNowTickMs();
      is_handled_ = false;
      return true;
    }
    return false;
  }

  const Data &getData(void) const { return data_; }

  /**
   * @brief 补弹机器人 ID
   */
  SuppliedRobotId robot_id(void) const { return SuppliedRobotId(data_.robot_id); }
  /**
   * @brief 出弹口开闭状态
   */
  SupplierStep supplier_step(void) const { return SupplierStep(data_.Supplier_step); }
  /**
   * @brief 补弹数量
   */
  SuppliedNumber supplied_num(void) const { return SuppliedNumber(data_.supplied_num); }

 private:
  Data data_ = {0};
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace referee
}  // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0102_TEAM_SUPPLIERS_HPP_ */
