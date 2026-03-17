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

#endif /* ROBOT_PARAM_H */
