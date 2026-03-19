/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       robot_param.h
  * @brief      这里是机器人参数配置文件，包括底盘参数，物理参数等
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Mar-31-2024     Penguin         1. done
  *  V1.0.1     Apr-16-2024     Penguin         1. 添加云台和发射机构类型
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#ifndef ROBOT_PARAM_H
#define ROBOT_PARAM_H

#include "robot_typedef.h"
#include "struct_typedef.h"

// 导入具体的机器人参数配置文件
#include "robot_param_balanced_infantry.h"

// 机器人速度限制参数
#define MAX_SPEED_VECTOR_VX (3.5f)
#define MAX_SPEED_VECTOR_VY (3.5f)
#define MAX_SPEED_VECTOR_WZ (6.0f)

#define MIN_SPEED_VECTOR_VX (-MAX_SPEED_VECTOR_VX)
#define MIN_SPEED_VECTOR_VY (-MAX_SPEED_VECTOR_VY)
#define MIN_SPEED_VECTOR_WZ (-MAX_SPEED_VECTOR_WZ)

// 关节电机相关参数
#define MAX_TORQUE_PROTECT (25.0f) // (Nm)最大扭矩保护
// DM控制参数
#define CALIBRATE_VEL_KD (4.0f)  // 校准MIT速度控制KD
#define ZERO_FORCE_VEL_KD (4.0f) // 无力MIT速度控制KD
#define NORMAL_POS_KP (20.0f)    // 正常MIT位置控制KP
#define NORMAL_POS_KD (1.0f)     // 正常MIT位置控制KD
// DM电机限位
#define MIN_J0_ANGLE (-0.6f) // (rad)关节角度下限
#define MIN_J1_ANGLE (-1.8f) // (rad)关节角度下限
#define MIN_J2_ANGLE (-1.8f) // (rad)关节角度下限
#define MIN_J3_ANGLE (0.0f)  // (rad)关节角度下限
// DM电机初始角度与水平线的关系
#define J0_ANGLE_OFFSET (-0.19163715f + M_PI) // (rad)关节0角度偏移量(电机0点到水平线的夹角)
#define J1_ANGLE_OFFSET (0.19163715f)         // (rad)关节1角度偏移量(电机0点到水平线的夹角)
#define J2_ANGLE_OFFSET (0.19163715f + M_PI)  // (rad)关节2角度偏移量(电机0点到水平线的夹角)
#define J3_ANGLE_OFFSET (-0.19163715f)        // (rad)关节3角度偏移量(电机0点到水平线的夹角)
// 电机旋转方向定义
#define J0_DIRECTION (-1)
#define J1_DIRECTION (-1)
#define J2_DIRECTION (1)
#define J3_DIRECTION (1)

#define W0_DIRECTION (1)
#define W1_DIRECTION (-1)

// 轮子相关参数
#define WHEEL_MASS (0.65f)     // (kg)轮子重量
#define WHEEL_RADIUS (0.0625f) // (m)轮子半径
#define WHEEL_BASE (0.51175f)  // (m)驱动轮轴距，即左右轮之间的默认距离

// 机器人物理参数
#define BODY_MASS (8.5f) // (kg)机身重量

// 底盘校准相关参数
#define ZERO_POS_THRESHOLD 0.001f     // 关节位置小于该阈值时认为已经校准到位
#define CALIBRATE_STOP_VELOCITY 0.05f // 关节速度小于该阈值时认为已经停止, rad/s
#define CALIBRATE_STOP_TIME 200       // 校准停止状态持续超过该时间时认为已经稳定, ms
#define CALIBRATE_VELOCITY 2.0f       // 校准时的关节速度, rad/s

// 底盘错误代码定义
#define JOINT_ERROR_OFFSET ((uint8_t)1 << 0) // 关节电机错误偏移量
#define WHEEL_ERROR_OFFSET ((uint8_t)1 << 1) // 驱动轮电机错误偏移量
#define DBUS_ERROR_OFFSET ((uint8_t)1 << 2)  // dbus错误偏移量
#define FLOATING_OFFSET ((uint8_t)1 << 3)    // 悬空状态偏移量

#endif /* ROBOT_PARAM_H */
