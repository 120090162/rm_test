#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H

#include "main.h"
#include "pid.h"
#include "INS_task.h"
#include "Motor.h"

#define ROLL_PID_KP 140.0f
#define ROLL_PID_KI 0.0f
#define ROLL_PID_KD 10.0f
#define ROLL_PID_MAX_OUT 100.0f
#define ROLL_PID_MAX_IOUT 0.0f

#define TP_PID_KP 30.0f
#define TP_PID_KI 0.0f
#define TP_PID_KD 1.0f
#define TP_PID_MAX_OUT 2.0f
#define TP_PID_MAX_IOUT 0.0f

#define TURN_PID_KP 2.5f
#define TURN_PID_KI 0.0f
#define TURN_PID_KD 0.3f
#define TURN_PID_MAX_OUT 1.0f // 轮毂电机的额定扭矩
#define TURN_PID_MAX_IOUT 0.0f

typedef struct
{
	/*左右两腿的公共参数，固定不变*/
	float l5; // AE长度 //单位为m
	float l1; // 单位为m
	float l2; // 单位为m
	float l3; // 单位为m
	float l4; // 单位为m

	float XB, YB; // B点的坐标
	float XD, YD; // D点的坐标

	float XC, YC;	// C点的直角坐标
	float L0, phi0; // C点的极坐标
	float alpha;
	float d_alpha;

	float lBD; // BD两点的距离

	float d_phi0;	 // 现在C点角度phi0的变换率
	float last_phi0; // 上一次C点角度，用于计算角度phi0的变换率d_phi0

	float A0, B0, C0; // 中间变量
	float phi2, phi3;
	float phi1, phi4;

	float j11, j12, j21, j22; // 笛卡尔空间力到关节空间的力的雅可比矩阵系数
	float torque_set[2];

	float F0;
	float Tp;
	float F02;

	float theta;
	float d_theta; // theta的一阶导数
	float last_d_theta;
	float dd_theta; // theta的二阶导数

	float d_L0;	 // L0的一阶导数
	float dd_L0; // L0的二阶导数
	float last_L0;
	float last_d_L0;

	float FN; // 支持力

	uint8_t first_flag;
	uint8_t leg_flag; // 腿长完成标志
} vmc_leg_t;

typedef struct
{
	DM_Motor_Info_Typedef *joint_motor[4];
	LK_Motor_Info_Typedef *wheel_motor[2];

	float v_set; // 期望速度，单位是m/s
	float x_set; // 期望位置，单位是m
	float target_v;

	float turn_set; // 期望yaw轴弧度
	float roll_set; // 期望roll轴弧度
	float roll_x;
	float phi_set;
	float theta_set;

	float leg_set; // 期望腿长，单位是m
	float last_leg_set;

	float v_filter; // 滤波后的车体速度，单位是m/s
	float x_filter; // 滤波后的车体位置，单位是m

	float myPithR;
	float myPithGyroR;
	float myPithL;
	float myPithGyroL;
	float roll;
	float total_yaw;
	float theta_err; // 两腿夹角误差

	float turn_T;  // yaw轴补偿
	float roll_f0; // roll轴补偿

	float leg_tp; // 防劈叉补偿

	uint8_t start_flag; // 启动标志

	uint8_t jump_flag;	// 右腿跳跃标志
	uint8_t jump_flag2; // 左腿跳跃标志

	uint8_t prejump_flag; // 预跳跃标志
	uint8_t recover_flag; // 一种情况下的倒地自起标志

} chassis_t;

extern chassis_t chassis_move;
extern vmc_leg_t left;
extern vmc_leg_t right;

extern uint32_t CHASS_TIME;

extern void mySaturate(float *in, float min, float max);

#endif