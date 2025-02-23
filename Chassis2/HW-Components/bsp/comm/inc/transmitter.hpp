/**
 *******************************************************************************
 * @file      : transmitter.hpp
 * @brief     : 用于统一发送端接口的虚基类
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2024-07-10      Caikunzhen      1. 完成正式版
 *******************************************************************************
 * @attention :
 *  需要通过通用通信驱动发送的类需要继承该接口类
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_BSP_COMM_TRANSMITTER_HPP_
#define HW_COMPONENTS_BSP_COMM_TRANSMITTER_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstddef>
#include <cstdint>

#include "allocator.hpp"
#include "list.hpp"

namespace hello_world
{
namespace comm
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/**
 * @brief       发送端接口
 * @note        需要通过通用通信驱动发送的类需要继承该接口类
 */
class Transmitter : virtual public MemMgr
{
 public:
  typedef tools::list<uint32_t> TxIds;

  Transmitter(void) = default;
  Transmitter(const Transmitter&) = default;
  Transmitter& operator=(const Transmitter&) = default;
  Transmitter(Transmitter&&) = default;
  Transmitter& operator=(Transmitter&&) = default;

  virtual ~Transmitter(void) = default;

  /**
   * @brief       获取发送端当前 ID
   * @param        None
   * @retval       发送端当前 ID
   * @note        发送管理区会根据 ID 进行管理，相同 ID 的发送端会共同编译一条报文
   */
  virtual uint32_t txId(void) const = 0;

  /**
   * @brief       获取发送端所有 ID
   * @param        None
   * @retval       发送端所有 ID
   * @note        发送管理区会根据 ID 进行管理，相同 ID 的发送端会共同编译一条报文
   */
  virtual const TxIds &txIds(void) const = 0;

  /**
   * @brief       编码，由发送管理器调用
   * @param        len: 需传入缓冲区长度，传出编码后的数据长度
   * @param        data: 缓冲区指针
   * @retval       编码成功返回 true，否则返回 false
   * @note        建议在 len 小于所需长度时返回 false，以防止数组越界的情况。要求仅对
   *              需要编码的部分进行处理，在编码时发送管理器会以此调用相同 ID 的发送端
   *              对报文进行编译，当某个发送端返回编码失败时会直接放弃调用后续相同 ID
   *              的发送端的编码，因此建议在内部统计编码成功与失败的次数，便于调试时进
   *              行问题定位
   */
  virtual bool encode(size_t& len, uint8_t* data) = 0;

  /**
   * @brief       发送成功回调，由发送管理器在发送成功后调用
   * @retval       None
   * @note        成功发送后调用，建议在内部统计统计发送成功的次数，由于发送管理器只有
   *              在相同 ID 的发送端对同一条报文的编码都通过时才会将该报文发出，因此通
   *              过与编码成功次数的对比可判断编码成功的消息是否都成功发出，便于调试时
   *              进行问题定位
   */
  virtual void txSuccessCb(void) = 0;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace comm
}  // namespace hello_world

#endif /* HW_COMPONENTS_BSP_COMM_TRANSMITTER_HPP_ */
