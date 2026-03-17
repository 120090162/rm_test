#include "Remote_Task.h"
#include "cmsis_os.h"

extern chassis_t chassis_move;
uint32_t REMOTE_TIME = 10; // 遥控器任务运行周期10ms

extern vmc_leg_t right;
extern vmc_leg_t left;

/**************************************************************************
Function: Sbus Remote
Input   : none
Output  : none
Auth    : DHY (qq:965849293)
Date	: 2024
**************************************************************************/
void Remote_task(void)
{
	HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rx_buff, BUFF_SIZE * 2);
	float dt = REMOTE_TIME / 1000.0f;
	static float last_jump_vrb = 0;
	static uint32_t vbat_low_count = 0;

	while (1)
	{
		if (remoter.online)
		{
			if (remoter.toggle.swd == 0)
			{
				// Power Off (遥控器SWD拨杆打下，关闭底盘)
				chassis_move.start_flag = 0;
				chassis_move.recover_flag = 0;

				vbat_low_count = 0;
			}
			else if (remoter.toggle.swd == 1)
			{
				//				if(chassis_move.vbus > VBAT_LOW_VAL)
				//				{
				//					//Power On
				//					chassis_move.start_flag=1;
				//				}
				//				else
				//				{
				//
				//					//Power Off
				//					chassis_move.start_flag=0;
				//					chassis_move.recover_flag=0;
				//				}

				if ((chassis_move.vbus < VBAT_LOW_VAL) && (chassis_move.start_flag == 1))
					vbat_low_count++;

				// 低电量保护逻辑，低电量持续一段时间后自动断电保护
				if (vbat_low_count > 100)
				{
					chassis_move.start_flag = 0;
					chassis_move.recover_flag = 0;
				}
				else
				{
					// Power On (遥控器SWD拨杆打上，启动底盘)
					chassis_move.start_flag = 1;
				}
			}

			// 倒地检测与自恢复逻辑 (当俯仰角过大判定为倒地时)
			if (chassis_move.recover_flag == 0 && ((chassis_move.myPithR < ((-3.1415926f) / 4.0f) && chassis_move.myPithR > ((-3.1415926f) / 2.0f)) || (chassis_move.myPithR > (3.1415926f / 4.0f) && chassis_move.myPithR < (3.1415926f / 2.0f))))
			{
				chassis_move.recover_flag = 1; // 需要恢复倒地
				chassis_move.leg_set = 0.08f;  // 恢复原始腿长
			}

			if (chassis_move.start_flag == 1)
			{
				if (chassis_move.vbus_mode == 1)
				{
					// 4s 电池供电时的速度映射
					chassis_move.v_set = ((float)remoter.joy.right_vert) * (-0.00097f * 0.5f); // 摇杆向前推为负
				}
				else if (chassis_move.vbus_mode == 2)
				{
					// 6s 电池供电时的速度映射 (动力更强，倍率更高)
					chassis_move.v_set = ((float)remoter.joy.right_vert) * (-0.00097f * 1.3f); // 摇杆向前推为负
				}
				chassis_move.x_set = chassis_move.x_set + chassis_move.v_set * dt;

				// 左摇杆水平方向控制偏航角 (Yaw转向)
				chassis_move.turn_set += ((float)remoter.joy.left_hori) * (-0.0000625f);

				// SWA 拨杆控制横滚角 (Roll姿态)
				if (remoter.toggle.swa == 0)
				{
					chassis_move.roll_set = -0.03f;
				}
				else
				{
					chassis_move.roll_set += ((float)remoter.joy.right_hori) * (0.000016f);
				}
				mySaturate(&chassis_move.roll_set, -0.40f, 0.40f);

				// SWC 拨杆结合左摇杆垂直方向控制腿长
				if (remoter.toggle.swc == 0)
				{
					chassis_move.leg_set = 0.08f;
				}
				else
				{
					chassis_move.leg_set = (remoter.joy.left_vert + 1024) * (0.00007f) + 0.072f;
				}

				mySaturate(&chassis_move.leg_set, 0.072f, 0.21f);

				if (fabsf(chassis_move.last_leg_set - chassis_move.leg_set) > 0.0001f)
				{						// 遥控器设置腿长在变化
					right.leg_flag = 1; // 为1标志位遥控器正在控制腿长，此时不进行离地检测，因为在腿长瞬间变短时离地检测会误判为离地
					left.leg_flag = 1;
				}
				chassis_move.last_leg_set = chassis_move.leg_set;

				// SWB 拨杆结合拨轮控制跳跃逻辑
				if (remoter.toggle.swb == 0)
				{
					// chassis_move.jump_flag=0;
					// chassis_move.jump_flag2=0;
				}
				else
				{
					// 触发跳跃条件：SWC未归零，拨轮拨动产生上升沿，且当前腿长允许跳跃
					if (remoter.toggle.swc != 0 && remoter.var.b < 500 && last_jump_vrb >= 500 && chassis_move.leg_set <= 0.16f)
					{
						if (chassis_move.vbus_mode == 2)
						{
							// 6s 电源模式下才允许跳跃（保证动力足够）
							chassis_move.jump_flag = 1;
							chassis_move.jump_flag2 = 1;
						}
					}
				}

				last_jump_vrb = remoter.var.b;
			}

			else
			{
				chassis_move.v_set = 0.0f;						// 速度清零
				chassis_move.x_set = chassis_move.x_filter;		// 保持位置
				chassis_move.turn_set = chassis_move.total_yaw; // 保持偏航
				chassis_move.leg_set = 0.08f;					// 原始腿长
				chassis_move.roll_set = -0.03f;
			}
		}
		else
		{
			// 遥控器离线 (失控保护)
			chassis_move.start_flag = 0;
			chassis_move.recover_flag = 0;
			vbat_low_count = 0;

			chassis_move.v_set = 0.0f;						// 速度清零
			chassis_move.x_set = chassis_move.x_filter;		// 保持位置
			chassis_move.turn_set = chassis_move.total_yaw; // 保持偏航
			chassis_move.leg_set = 0.08f;					// 原始腿长
			chassis_move.roll_set = -0.03f;
		}

		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rx_buff, BUFF_SIZE * 2); // 重新开启串口空闲中断DMA接收
		osDelay(REMOTE_TIME);
	}
}