/**
 *******************************************************************************
 * @file      :comm_task.cpp
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
#include "comm_task.hpp"

// hal
#include "can.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"

// HW-Components
#include "tick.hpp"

// custom
#include "gimbal_chassis_comm.hpp"
#include "ins_all.hpp"

using hello_world::comm::CanRxMgr;
using hello_world::comm::CanTxMgr;
using hello_world::comm::UartRxMgr;
using hello_world::comm::UartTxMgr;

using robot::GimbalChassisComm;
/* Private macro -------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

// rx communication components objects
static CanRxMgr *can1_rx_mgr_ptr = nullptr;
static CanTxMgr *can1_tx_mgr_ptr = nullptr;

static CanRxMgr *can2_rx_mgr_ptr = nullptr;
static CanTxMgr *can2_tx_mgr_ptr = nullptr;

static UartRxMgr *vision_rx_mgr_ptr = nullptr;
static UartTxMgr *vision_tx_mgr_ptr = nullptr;

static GimbalChassisComm *gc_comm_ptr = nullptr;

/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void PrivatePointerInit(void);
static void CommHardWareInit(void);
static void CommAddReceiver(void);
static void CommAddTransmitter(void);

/* Exported function definitions ---------------------------------------------*/

void CommTaskInit(void) {
  PrivatePointerInit();
  CommAddReceiver();
  CommAddTransmitter();
  CommHardWareInit();
};

void CommTask(void) {
  HW_ASSERT(can1_tx_mgr_ptr != nullptr, "can1_tx_mgr_ptr is nullptr",
            can1_tx_mgr_ptr);
  HW_ASSERT(can2_tx_mgr_ptr != nullptr, "can2_tx_mgr_ptr is nullptr",
            can2_tx_mgr_ptr);
  HW_ASSERT(vision_tx_mgr_ptr != nullptr, "vision_tx_mgr_ptr is nullptr",
            vision_tx_mgr_ptr);
  can1_tx_mgr_ptr->startTransmit();
  can2_tx_mgr_ptr->startTransmit();
  vision_tx_mgr_ptr->startTransmit();
};

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  HW_ASSERT(can1_rx_mgr_ptr != nullptr, "can1_rx_mgr_ptr is nullptr",
            can1_rx_mgr_ptr);
  HW_ASSERT(can2_rx_mgr_ptr != nullptr, "can2_rx_mgr_ptr is nullptr",
            can2_rx_mgr_ptr);
  can1_rx_mgr_ptr->rxFifoMsgPendingCallback(hcan);
  can2_rx_mgr_ptr->rxFifoMsgPendingCallback(hcan);

  HW_ASSERT(gc_comm_ptr != nullptr, "gc_comm_ptr is nullptr", gc_comm_ptr);
  if (!gc_comm_ptr->isOffline()) {
    HAL_IWDG_Refresh(&hiwdg);
  }
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  HW_ASSERT(can1_rx_mgr_ptr != nullptr, "can1_rx_mgr_ptr is nullptr",
            can1_rx_mgr_ptr);
  HW_ASSERT(can2_rx_mgr_ptr != nullptr, "can2_rx_mgr_ptr is nullptr",
            can2_rx_mgr_ptr);
  can1_rx_mgr_ptr->rxFifoMsgPendingCallback(hcan);
  can2_rx_mgr_ptr->rxFifoMsgPendingCallback(hcan);
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) {
  HW_ASSERT(can1_tx_mgr_ptr != nullptr, "can1_tx_mgr_ptr is nullptr",
            can1_tx_mgr_ptr);
  HW_ASSERT(can2_tx_mgr_ptr != nullptr, "can2_tx_mgr_ptr is nullptr",
            can2_tx_mgr_ptr);
  can1_tx_mgr_ptr->txMailboxCompleteCallback(hcan);
  can2_tx_mgr_ptr->txMailboxCompleteCallback(hcan);
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan) {
  HW_ASSERT(can1_tx_mgr_ptr != nullptr, "can1_tx_mgr_ptr is nullptr",
            can1_tx_mgr_ptr);
  HW_ASSERT(can2_tx_mgr_ptr != nullptr, "can2_tx_mgr_ptr is nullptr",
            can2_tx_mgr_ptr);
  can1_tx_mgr_ptr->txMailboxCompleteCallback(hcan);
  can2_tx_mgr_ptr->txMailboxCompleteCallback(hcan);
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan) {
  HW_ASSERT(can1_tx_mgr_ptr != nullptr, "can1_tx_mgr_ptr is nullptr",
            can1_tx_mgr_ptr);
  HW_ASSERT(can2_tx_mgr_ptr != nullptr, "can2_tx_mgr_ptr is nullptr",
            can2_tx_mgr_ptr);
  can1_tx_mgr_ptr->txMailboxCompleteCallback(hcan);
  can2_tx_mgr_ptr->txMailboxCompleteCallback(hcan);
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
  HW_ASSERT(can1_tx_mgr_ptr != nullptr, "can1_tx_mgr_ptr is nullptr",
            can1_tx_mgr_ptr);
  HW_ASSERT(can2_tx_mgr_ptr != nullptr, "can2_tx_mgr_ptr is nullptr",
            can2_tx_mgr_ptr);
  can1_tx_mgr_ptr->errorCallback(hcan);
  can2_tx_mgr_ptr->errorCallback(hcan);
}

void CommHardWareInit(void) {
  // CAN init
  HW_ASSERT(can1_rx_mgr_ptr != nullptr, "can1_rx_mgr_ptr is nullptr",
            can1_rx_mgr_ptr);
  can1_rx_mgr_ptr->filterInit();
  can1_rx_mgr_ptr->startReceive();
  HAL_CAN_Start(&hcan1);

  HW_ASSERT(can2_rx_mgr_ptr != nullptr, "can2_rx_mgr_ptr is nullptr",
            can2_rx_mgr_ptr);
  can2_rx_mgr_ptr->filterInit();
  can2_rx_mgr_ptr->startReceive();
  HAL_CAN_Start(&hcan2);

  // vision DMA init
  HW_ASSERT(vision_rx_mgr_ptr != nullptr, "vision_rx_mgr_ptr is nullptr",
            vision_rx_mgr_ptr);
  vision_rx_mgr_ptr->startReceive();
};

/**
 * @brief   UART接收回调函数
 * @param   none
 * @retval  none
 * @note    none
 **/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  // 视觉
  HW_ASSERT(vision_rx_mgr_ptr != nullptr, "vision_rx_mgr_ptr is nullptr",
            vision_rx_mgr_ptr);
  vision_rx_mgr_ptr->rxEventCallback(huart, Size);
}

/* Private function definitions ----------------------------------------------*/

static void PrivatePointerInit(void) {
  can1_rx_mgr_ptr = GetCan1RxMgr();
  can1_tx_mgr_ptr = GetCan1TxMgr();

  can2_rx_mgr_ptr = GetCan2RxMgr();
  can2_tx_mgr_ptr = GetCan2TxMgr();

  vision_rx_mgr_ptr = GetVisionRxMgr();
  vision_tx_mgr_ptr = GetVisionTxMgr();

  gc_comm_ptr = GetGimbalChassisComm();
};

static void CommAddReceiver(void) {
  HW_ASSERT(can1_rx_mgr_ptr != nullptr, "can1_rx_mgr_ptr is nullptr",
            can1_rx_mgr_ptr);
  can1_rx_mgr_ptr->addReceiver(GetGimbalChassisComm());
  can1_rx_mgr_ptr->addReceiver(GetMotorYaw());

  HW_ASSERT(can2_rx_mgr_ptr != nullptr, "can2_rx_mgr_ptr is nullptr",
            can2_rx_mgr_ptr);
  can2_rx_mgr_ptr->addReceiver(GetMotorFricLeft());
  can2_rx_mgr_ptr->addReceiver(GetMotorFricRight());
  can2_rx_mgr_ptr->addReceiver(GetMotorFeed());
  can2_rx_mgr_ptr->addReceiver(GetMotorPitch());

  HW_ASSERT(vision_rx_mgr_ptr != nullptr, "vision_rx_mgr_ptr is nullptr",
            vision_rx_mgr_ptr);
  vision_rx_mgr_ptr->addReceiver(GetVision());
};

static void CommAddTransmitter(void) {
  HW_ASSERT(can1_tx_mgr_ptr != nullptr, "can1_tx_mgr_ptr is nullptr",
            can1_tx_mgr_ptr);
  can1_tx_mgr_ptr->addTransmitter(GetGimbalChassisComm());
  can1_tx_mgr_ptr->addTransmitter(GetMotorYaw());

  HW_ASSERT(can2_tx_mgr_ptr != nullptr, "can2_tx_mgr_ptr is nullptr",
            can2_tx_mgr_ptr);
  can2_tx_mgr_ptr->addTransmitter(GetMotorFricLeft());
  can2_tx_mgr_ptr->addTransmitter(GetMotorFricRight());
  can2_tx_mgr_ptr->addTransmitter(GetMotorFeed());
  can2_tx_mgr_ptr->addTransmitter(GetMotorPitch());

  HW_ASSERT(vision_tx_mgr_ptr != nullptr, "vision_tx_mgr_ptr is nullptr",
            vision_tx_mgr_ptr);
  vision_tx_mgr_ptr->addTransmitter(GetVision());
};