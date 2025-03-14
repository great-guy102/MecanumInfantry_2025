/**
 *******************************************************************************
 * @file      :ins_fsm.cpp
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
#include "ins_all.hpp"
/* Private constants ---------------------------------------------------------*/
const robot::Chassis::Config kChassisConfig = {
    .normal_trans_vel = 3.6f,  ///< 正常平移速度
    .gyro_rot_spd = 10.0f,     ///< 小陀螺旋转速度; 14.0f
    .yaw_sensitivity = 2 * PI, ///< YAW 轴灵敏度(单位：rad/s)
    .max_trans_vel = 5.0f,     ///< 最大平移速度
    .max_rot_spd = 10.0f,      ///< 最大旋转速度; 14.0f
};
/* Private macro -------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static bool is_robot_inited = false;
static bool is_chassis_inited = false;

robot::Robot unique_robot = robot::Robot();
robot::Chassis unique_chassis = robot::Chassis(kChassisConfig);
robot::Gimbal unique_gimbal = robot::Gimbal();
robot::Shooter unique_shooter = robot::Shooter();

/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
robot::Chassis *GetChassis() {
  if (!is_chassis_inited) {
    // * 1. 无通信功能的组件指针
    // * - 底盘运动学解算器
    unique_chassis.registerIkSolver(GetChassisIkSolver());
    unique_chassis.registerFkSolver(GetChassisFkSolver());
    // * - 轮组pid
    unique_chassis.registerWheelPid(GetPidWheelLeftFront(),
                                    robot::Chassis::kWheelPidIdxLeftFront);
    unique_chassis.registerWheelPid(GetPidWheelLeftRear(),
                                    robot::Chassis::kWheelPidIdxLeftRear);
    unique_chassis.registerWheelPid(GetPidWheelRightRear(),
                                    robot::Chassis::kWheelPidIdxRightRear);
    unique_chassis.registerWheelPid(GetPidWheelRightFront(),
                                    robot::Chassis::kWheelPidIdxRightFront);
    // * - 轮组速度斜坡滤波器
    unique_chassis.registerWheelSpeedRamp(
        GetRampWheel1Speed(), robot::Chassis::kWheelSpeedRampIdxLeftFront);
    unique_chassis.registerWheelSpeedRamp(
        GetRampWheel2Speed(), robot::Chassis::kWheelSpeedRampIdxLeftRear);
    unique_chassis.registerWheelSpeedRamp(
        GetRampWheel3Speed(), robot::Chassis::kWheelSpeedRampIdxRightRear);
    unique_chassis.registerWheelSpeedRamp(
        GetRampWheel4Speed(), robot::Chassis::kWheelSpeedRampIdxRightFront);

    // 随动速度
    unique_chassis.registerFollowOmegaPid(GetPidFollowOmega());
    // * - 功率限制
    unique_chassis.registerPwrLimiter(GetPwrLimiter());

    // * 2. 只接收数据的组件指针
    // 云台和底盘通信
    unique_chassis.registerGimbalChassisComm(GetGimbalChassisComm());
    // YAW 轴电机
    unique_chassis.registerYawMotor(GetMotorYaw());
    // * 3. 接收、发送数据的组件指针
    // 超级电容
    unique_chassis.registerCap(GetCap());
    // 轮组电机
    unique_chassis.registerWheelMotor(GetMotorWheelLeftFront(),
                                      robot::Chassis::kWheelMotorIdxLeftFront);
    unique_chassis.registerWheelMotor(GetMotorWheelLeftRear(),
                                      robot::Chassis::kWheelMotorIdxLeftRear);
    unique_chassis.registerWheelMotor(GetMotorWheelRightRear(),
                                      robot::Chassis::kWheelMotorIdxRightRear);
    unique_chassis.registerWheelMotor(GetMotorWheelRightFront(),
                                      robot::Chassis::kWheelMotorIdxRightFront);
    is_chassis_inited = true;
  }
  return &unique_chassis;
};

robot::Gimbal *GetGimbal() { return &unique_gimbal; };
robot::Shooter *GetShooter() { return &unique_shooter; };
// TODO：整车移植
robot::Robot *GetRobot() {
  if (!is_robot_inited) {
    // main 组件指针注册
    // 主要模块状态机组件指针
    unique_robot.registerChassis(GetChassis());
    unique_robot.registerShooter(GetShooter());
    unique_robot.registerGimbal(GetGimbal());

    // 无通信功能的组件指针
    unique_robot.registerBuzzer(GetBuzzer());
    unique_robot.registerImu(GetImu());
    unique_robot.registerRampCmdVx(GetRampCmdVx());
    unique_robot.registerRampCmdVy(GetRampCmdVy());

    // 只接收数据的组件指针
    unique_robot.registerRc(GetRemoteControl());
    // 只发送数据的组件指针
    unique_robot.registerCap(GetCap(), GetCan1TxMgr());
    unique_robot.registerMotorYaw(GetMotorYaw(), GetCan1TxMgr());

    unique_robot.registerMotorWheels(GetMotorWheelLeftFront(),
                                     robot::Robot::kWheelMotorIdxLeftFront,
                                     GetCan2TxMgr());
    unique_robot.registerMotorWheels(GetMotorWheelLeftRear(),
                                     robot::Robot::kWheelMotorIdxLeftRear,
                                     GetCan2TxMgr());
    unique_robot.registerMotorWheels(GetMotorWheelRightRear(),
                                     robot::Robot::kWheelMotorIdxRightRear,
                                     GetCan2TxMgr());
    unique_robot.registerMotorWheels(GetMotorWheelRightFront(),
                                     robot::Robot::kWheelMotorIdxRightFront,
                                     GetCan2TxMgr());
    // 收发数据的组件指针
    unique_robot.registerGimbalChassisComm(GetGimbalChassisComm(),
                                           GetCan1TxMgr());
    unique_robot.registerReferee(GetReferee(), GetRfrTxMgr());

    unique_robot.registerPerformancePkg(GetRobotPerformancePackage());
    unique_robot.registerPowerHeatPkg(GetRobotPowerHeatPackage());
    unique_robot.registerShooterPkg(GetRobotShooterPackage());
    is_robot_inited = true;
  }
  return &unique_robot;
};

/* Private function definitions ----------------------------------------------*/
