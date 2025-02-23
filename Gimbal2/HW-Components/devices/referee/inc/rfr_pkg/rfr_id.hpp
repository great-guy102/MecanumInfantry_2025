/**
 * @file      rfr_id.hpp
 * @author    ZhouShichan (zsc19823382069@163.com)
 * @date      2024-01-26
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
#ifndef HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_ID_HPP_
#define HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_ID_HPP_

/* Includes ------------------------------------------------------------------*/
#include "rfr_pkg_core.hpp"

namespace hello_world
{
namespace referee
{
namespace ids
{
/* Exported macro ------------------------------------------------------------*/

enum class TeamColor {
  kError = 0u,
  kRed,
  kBlue,
  kServer,
};

enum class IdType {
  kError = 0u,
  kRobot,
  kClient,
  kServer,
};

enum class RobotId {
  kRedHero = 1u,          ///< 红方英雄机器人 id: 001, 0x01
  kRedEngineer = 2u,      ///< 红方工程机器人 id: 002, 0x02
  kRedStandard3 = 3u,     ///< 红方3号步兵机器人 id: 003, 0x03
  kRedStandard4 = 4u,     ///< 红方4号步兵机器人 id: 004, 0x04
  kRedStandard5 = 5u,     ///< 红方5号步兵机器人 id: 005, 0x05
  kRedAerial = 6u,        ///< 红方空中机器人 id: 006, 0x06
  kRedSentry = 7u,        ///< 红方哨兵机器人 id: 007, 0x07
  kRedDart = 8u,          ///< 红方飞镖机器人 id: 008, 0x08
  kRedRadar = 9u,         ///< 红方雷达机器人 id: 009, 0x09
  kRedOutpost = 10u,      ///< 红方前哨站 id: 010, 0x0A
  kRedBase = 11u,         ///< 红方基地 id: 011, 0x0B
  kBlueHero = 101u,       ///< 蓝方英雄机器人 id: 101, 0x65
  kBlueEngineer = 102u,   ///< 蓝方工程机器人 id: 102, 0x66
  kBlueStandard3 = 103u,  ///< 蓝方3号步兵机器人 id: 103, 0x67
  kBlueStandard4 = 104u,  ///< 蓝方4号步兵机器人 id: 104, 0x68
  kBlueStandard5 = 105u,  ///< 蓝方5号步兵机器人 id: 105, 0x69
  kBlueAerial = 106u,     ///< 蓝方空中机器人 id: 106, 0x6A
  kBlueSentry = 107u,     ///< 蓝方哨兵机器人 id: 107, 0x6B
  kBlueDart = 108u,       ///< 蓝方飞镖机器人 id: 108, 0x6C
  kBlueRadar = 109u,      ///< 蓝方雷达机器人 id: 109, 0x6D
  kBlueOutpost = 110u,    ///< 蓝方前哨站 id: 110, 0x6E
  kBlueBase = 111u,       ///< 蓝方基地 id: 111, 0x6F
};

enum class ClientId {
  kRedHero = 0x0101,        ///< 红方英雄机器人客户端 id: 0x0101
  kRedEngineer = 0x0102,    ///< 红方工程机器人客户端 id: 0x0102
  kRedStandard3 = 0x0103,   ///< 红方3号步兵机器人客户端 id: 0x0103
  kRedStandard4 = 0x0104,   ///< 红方4号步兵机器人客户端 id: 0x0104
  kRedStandard5 = 0x0105,   ///< 红方5号步兵机器人客户端 id: 0x0105
  kRedAerial = 0x0106,      ///< 红方空中机器人客户端 id: 0x0106
  kBlueHero = 0x0165,       ///< 蓝方英雄机器人客户端 id: 0x0165
  kBlueEngineer = 0x0166,   ///< 蓝方工程机器人客户端 id: 0x0166
  kBlueStandard3 = 0x0167,  ///< 蓝方3号步兵机器人客户端 id: 0x0167
  kBlueStandard4 = 0x0168,  ///< 蓝方4号步兵机器人客户端 id: 0x0168
  kBlueStandard5 = 0x0169,  ///< 蓝方5号步兵机器人客户端 id: 0x0169
  kAerial = 0x016A,         ///< 蓝方空中机器人客户端 id: 0x016A
};

enum class ServerId {
  kServer = 0x8080,  ///< 裁判系统服务器 id: 0x8080
};
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

/**
 * @brief 将客户端 ID 转换为机器人 ID
 *
 * 根据串口协议规定，机器人 ID 与对应的客户端 ID 相差 0x0100，即 robot_id =
 * client_id - 0x0100。
 * @param client_id 客户端 ID
 * @return RfrId 机器人 ID
 * @attention 该函数不会检查 ID 是否为客户端 ID，请务必确保输入 ID 正确
 */
inline RfrId ClientId2RobotId(RfrId client_id) { return client_id - 0x0100; }

/**
 * @brief 将机器人 ID 转换为客户端 ID
 *
 * 根据串口协议规定，机器人 ID 与对应的客户端 ID 相差 0x0100，即 client_id =
 * robot_id + 0x0100。
 * @param robot_id 机器人 ID
 * @return RfrId 客户端 ID
 * @attention 该函数不会检查 ID 是否为机器人 ID，请务必确保输入 ID 正确
 */
inline RfrId RobotId2ClientId(RfrId robot_id) { return robot_id + 0x0100; }

/**
 * @brief 判断是否为机器人 ID
 *
 * 机器人 ID 为 1~11 和 101~111，即 0x01~0x0B 和 0x65~0x6F。
 * 通过判断输入值是否位于该区间来判断是否为机器人 ID。
 * @param id ID
 * @return bool 是否为机器人 ID
 */
inline bool IsRobotId(RfrId id)
{
  return ((uint16_t)RobotId::kRedHero <= id &&
          id <= (uint16_t)RobotId::kRedBase) ||
         ((uint16_t)RobotId::kBlueHero <= id &&
          id <= (uint16_t)RobotId::kBlueBase);
}

/**
 * @brief 判断是否为客户端 ID
 *
 * 客户端 ID 为 0x0101~0x0106 和 0x0165~0x016A。
 * 通过判断输入值是否位于该区间来判断是否为客户端 ID。
 * @param id ID
 * @return bool 是否为客户端 ID
 */
inline bool IsClientId(RfrId id)
{
  return ((uint16_t)ClientId::kRedHero <= id &&
          id <= (uint16_t)ClientId::kRedAerial) ||
         ((uint16_t)ClientId::kBlueHero <= id &&
          id <= (uint16_t)ClientId::kAerial);
}

/**
 * @brief 判断是否为服务器 ID
 *
 * 服务器 ID 为 0x8080。
 * 通过判断输入值是否等于该值来判断是否为服务器 ID。
 * @param id ID
 * @return bool 是否为服务器 ID
 */
inline bool IsServerId(RfrId id) { return id == RfrId(ServerId::kServer); }

/**
 * @brief 获取队伍颜色
 * @param id ID
 * @return TeamColor 队伍颜色
 */
inline TeamColor GetTeamColor(RfrId id)
{
  if (IsServerId(id)) {
    return TeamColor::kServer;
  }

  if (IsRobotId(id) || IsClientId(id)) {
    return (id & 0x0060) ? TeamColor::kBlue : TeamColor::kRed;
  }
  return TeamColor::kError;
}

/**
 * @brief 判断两个 ID 是否为同一颜色
 * @param id1 第一个 ID
 * @param id2 第二个 ID
 * @return bool 是否为同一颜色
 * @attention 该函数只判断两个 ID 是否为同一颜色(哪怕是 `kTeamColorError`)，即不能保
 *            证一定都是 `kTeamColorRed` 或 `kTeamColorBlue`。
 * @see GetTeemColor
 * @see TeamColor
 */
inline bool IsSameColer(RfrId id1, RfrId id2)
{
  return GetTeamColor(id1) == GetTeamColor(id2);
}

/**
 * @brief 获取 ID 类型
 * @param id ID
 * @return IdType ID 类型
 * @see IdType
 */
inline IdType GetIdType(RfrId id)
{
  if (IsRobotId(id)) {
    return IdType::kRobot;
  } else if (IsClientId(id)) {
    return IdType::kClient;
  } else if (IsServerId(id)) {
    return IdType::kServer;
  } else {
    return IdType::kError;
  }
}
}  // namespace ids
}  // namespace referee
}  // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_ID_HPP_ */
