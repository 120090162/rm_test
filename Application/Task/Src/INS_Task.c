/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : INS_Task.c
 * @brief          : INS task
 * @author         : GrassFan Wang
 * @date           : 2025/01/22
 * @version        : v1.0
 ******************************************************************************
 * @attention      : None
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "INS_Task.h"

/**
 * @brief the structure that contains the information for the INS.
 */
INS_t INS;

struct MAHONY_FILTER_t mahony;
Axis3f Gyro, Accel;
float gravity[3] = {0, 0, 9.81f};

uint32_t INS_DWT_Count = 0;
float ins_dt = 0.0f;
float ins_time = 0.0f;
int stop_time = 0;

/**
 * @brief the Initialize data of Temperature Control PID.
 */
static float TemCtrl_PID_Param[PID_PARAMETER_NUM] = {1200, 20, 0, 0, 0, 0, 2000};

/**
 * @brief the structure that contains the Information of Temperature Control PID.
 */
PID_Info_TypeDef TempCtrl_PID;

/**
 * @brief Initializes the INS_Task.
 */
static void INS_Task_Init(void);

/**
 * @brief  Control the BMI088 temperature
 */
static void BMI088_Temp_Control(float temp);

/* USER CODE BEGIN Header_INS_Task */
/**
 * @brief  Function implementing the StartINSTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_INS_Task */
void INS_task(void)
{
	/* USER CODE BEGIN INS_Task */

	/* Initializes the INS_Task. */
	INS_Task_Init();

	/* Infinite loop */
	while (1)
	{
		ins_dt = DWT_GetDeltaT(&INS_DWT_Count);

		mahony.dt = ins_dt;

		/* Update the BMI088 measurement */
		BMI088_Read(&BMI088);

		INS.Accel[AXIS_X] = BMI088.Accel[AXIS_X];
		INS.Accel[AXIS_Y] = BMI088.Accel[AXIS_Y];
		INS.Accel[AXIS_Z] = BMI088.Accel[AXIS_Z];
		Accel.x = BMI088.Accel[AXIS_X];
		Accel.y = BMI088.Accel[AXIS_Y];
		Accel.z = BMI088.Accel[AXIS_Z];
		INS.Gyro[AXIS_X] = BMI088.Gyro[AXIS_X];
		INS.Gyro[AXIS_Y] = BMI088.Gyro[AXIS_Y];
		INS.Gyro[AXIS_Z] = BMI088.Gyro[AXIS_Z];
		Gyro.x = BMI088.Gyro[AXIS_X];
		Gyro.y = BMI088.Gyro[AXIS_Y];
		Gyro.z = BMI088.Gyro[AXIS_Z];

		mahony_input(&mahony, Gyro, Accel);
		mahony_update(&mahony);
		mahony_output(&mahony);
		RotationMatrix_update(&mahony);

		INS.q[0] = mahony.q0;
		INS.q[1] = mahony.q1;
		INS.q[2] = mahony.q2;
		INS.q[3] = mahony.q3;

		// 将重力从导航系(n系)转换到机体系(b系)，从加速度计中减去重力，得到机体的纯运动加速度
		float gravity_b[3];
		EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
		for (uint8_t i = 0; i < 3; i++) // 同时过一个一阶低通滤波
		{
			INS.MotionAccel_b[i] = (INS.Accel[i] - gravity_b[i]) * ins_dt / (INS.AccelLPF + ins_dt) + INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + ins_dt);
			//			INS.MotionAccel_b[i] = (INS.Accel[i] ) * dt / (INS.AccelLPF + dt)
			//														+ INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + dt);
		}
		BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q); // 再转换回导航系(n系)

		// 加速度死区处理 (滤除微小震动和零偏)
		if (fabsf(INS.MotionAccel_n[0]) < 0.02f)
		{
			INS.MotionAccel_n[0] = 0.0f; // x轴
		}
		if (fabsf(INS.MotionAccel_n[1]) < 0.02f)
		{
			INS.MotionAccel_n[1] = 0.0f; // y轴
		}
		if (fabsf(INS.MotionAccel_n[2]) < 0.04f)
		{
			INS.MotionAccel_n[2] = 0.0f; // z轴
			stop_time++;
		}
		//		if(stop_time>10)
		//		{ // 静止10ms
		//		  stop_time=0;
		//			INS.v_n=0.0f; // 速度清零
		//		}

		if (ins_time > 3000.0f)
		{
			INS.v_n = INS.v_n + INS.MotionAccel_n[1] * 0.001f;
			INS.x_n = INS.x_n + INS.v_n * 0.001f;
			INS.ins_flag = 1; // 滤波器收敛完成，加速度也已稳定，标志位置1，系统可以开始正常控制

			// 获取欧拉角(姿态角)
			INS.Roll = mahony.roll;
			INS.Pitch = mahony.pitch;
			INS.Yaw = mahony.yaw;

			// INS.YawTotalAngle=INS.YawTotalAngle+INS.Gyro[2]*0.001f;

			// 处理 Yaw 角过零点的问题 (将 -180~180 的角度转换为连续的累计角度)
			if (INS.Yaw - INS.YawAngleLast > 3.1415926f)
			{
				INS.YawRoundCount--;
			}
			else if (INS.Yaw - INS.YawAngleLast < -3.1415926f)
			{
				INS.YawRoundCount++;
			}
			INS.YawTotalAngle = 6.283f * INS.YawRoundCount + INS.Yaw;
			INS.YawAngleLast = INS.Yaw;
		}
		else
		{
			ins_time++; // 等待系统初始化和滤波收敛
		}

		if (INS_DWT_Count % 5 == 0)
		{
			BMI088_Temp_Control(BMI088.Temperature);
		}

		osDelay(1);
	}
	/* USER CODE END INS_Task */
}
//------------------------------------------------------------------------------
/**
 * @brief Initializes the INS_Task.
 */
static void INS_Task_Init(void)
{
	mahony_init(&mahony, 1.0f, 0.0f, 0.001f);
	INS.AccelLPF = 0.0089f;

	/* Initializes the Temperature Control PID  */
	PID_Init(&TempCtrl_PID, PID_POSITION, TemCtrl_PID_Param);
}
//------------------------------------------------------------------------------
/**
 * @brief  Control the BMI088 temperature
 * @param  temp  measure of the BMI088 temperature
 * @retval none
 */
static void BMI088_Temp_Control(float Temp)
{
	PID_Calculate(&TempCtrl_PID, 40.f, Temp);

	VAL_LIMIT(TempCtrl_PID.Output, -TempCtrl_PID.Param.LimitOutput, TempCtrl_PID.Param.LimitOutput);

	Heat_Power_Control((uint16_t)(TempCtrl_PID.Output));
}
//------------------------------------------------------------------------------

/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
	vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
					   (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
					   (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

	vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
					   (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
					   (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

	vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
					   (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
					   (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
	vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
					   (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
					   (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

	vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
					   (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
					   (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

	vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
					   (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
					   (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}