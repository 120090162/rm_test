#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H

#include "main.h"
#include "pid.h"
#include "INS_task.h"

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
#define TURN_PID_MAX_OUT 1.0f // 输出限幅，对应电机的额定扭矩
#define TURN_PID_MAX_IOUT 0.0f

typedef struct
{
	/*�������ȵĹ����������̶�����*/
	float l5; // AE���� //��λΪm
	float l1; // ��λΪm
	float l2; // ��λΪm
	float l3; // ��λΪm
	float l4; // ��λΪm

	float XB, YB; // B�������
	float XD, YD; // D�������

	float XC, YC;	// C���ֱ������
	float L0, phi0; // C��ļ�����
	float alpha;
	float d_alpha;

	float lBD; // BD����ľ���

	float d_phi0;	 // ����C��Ƕ�phi0�ı任��
	float last_phi0; // ��һ��C��Ƕȣ����ڼ���Ƕ�phi0�ı任��d_phi0

	float A0, B0, C0; // �м����
	float phi2, phi3;
	float phi1, phi4;

	float j11, j12, j21, j22; // �ѿ����ռ������ؽڿռ�������ſɱȾ���ϵ��
	float torque_set[2];

	float F0;
	float Tp;
	float F02;

	float theta;
	float d_theta; // theta��һ�׵���
	float last_d_theta;
	float dd_theta; // theta�Ķ��׵���

	float d_L0;	 // L0��һ�׵���
	float dd_L0; // L0�Ķ��׵���
	float last_L0;
	float last_d_L0;

	float FN; // ֧����

	uint8_t first_flag;
	uint8_t leg_flag; // �ȳ���ɱ�־
} vmc_leg_t;

typedef struct
{
	uint16_t id;
	uint16_t state;
	int p_int;
	int v_int;
	int t_int;
	int kp_int;
	int kd_int;
	float pos;
	float vel;
	float tor;
	float Kp;
	float Kd;
	float Tmos;
	float Tcoil;
} motor_fbpara_t;

typedef struct
{
	uint16_t mode;
	motor_fbpara_t para;
} Joint_Motor_t;

typedef struct
{
	uint16_t mode;
	float wheel_T; // ��챵�������Ť�أ���λΪN

	motor_fbpara_t para;
} Wheel_Motor_t;

typedef struct
{
	Joint_Motor_t joint_motor[4]; // 关节电机结构体 (控制腿部姿态)
	Wheel_Motor_t wheel_motor[2]; // 驱动轮电机结构体 (控制底盘移动)

	float v_set;	// 目标速度，单位：m/s
	float x_set;	// 目标位置，单位：m
	float target_v; // 过程目标速度

	float turn_set; // 目标 yaw 轴角度 (转向)
	float roll_set; // 目标 roll 轴角度 (横滚姿态)
	float roll_x;
	float phi_set;	 // 目标机体俯仰角 (Pitch)
	float theta_set; // 目标虚拟腿偏角

	float leg_set;		// 目标腿长，单位：m
	float last_leg_set; // 上一时刻的目标腿长

	float v_filter; // 滤波后的底盘速度，单位：m/s
	float x_filter; // 滤波后的底盘位置，单位：m

	float myPithR;	   // 右腿俯仰角
	float myPithGyroR; // 右腿俯仰角速度
	float myPithL;	   // 左腿俯仰角
	float myPithGyroL; // 左腿俯仰角速度
	float roll;		   // 机体横滚角
	float total_yaw;   // 连续的累计偏航角 (Yaw)
	float theta_err;   // 双腿角度误差 (用于双腿同步控制)

	float turn_T;  // yaw 轴力矩补偿
	float roll_f0; // roll 轴推力补偿

	float leg_tp; // 腿部推力/前馈补偿

	uint8_t start_flag; // 底盘启动标志位

	uint8_t jump_flag;	// 跳跃过程标志位 1 (离地/腾空)
	uint8_t jump_flag2; // 跳跃过程标志位 2 (落地/缓冲)

	uint8_t prejump_flag; // 预跳跃标志位 (下蹲蓄力阶段)
	uint8_t recover_flag; // 一定条件下的倒地自恢复标志位 (防翻车起立)

} chassis_t;

extern chassis_t chassis_move;
extern vmc_leg_t left;
extern vmc_leg_t right;

extern void mySaturate(float *in, float min, float max);

#endif