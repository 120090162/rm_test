#include "ChassisL_Task.h"

void ChassisL_task(void)
{
    while (INS.ins_flag == 0)
    { // 等待加速度收敛
        osDelay(1);
    }
    ChassisL_init(); // 初始化左边两个关节电机和左边轮毂电机的id和控制模式、初始化腿部

    while (1)
    {
        osDelay(CHASS_TIME * 3);
    }
}

void ChassisL_init(void)
{
    chassis_move.joint_motor[0] = &DM_8009_Motor[0];
    chassis_move.joint_motor[1] = &DM_8009_Motor[1];

    chassis_move.wheel_motor[0] = &LK_9025_Motor[0];

    for (int j = 0; j < 10; j++)
    {
        DM_Motor_Command(&FDCAN3_TxFrame, chassis_move.joint_motor[0], DM_Motor_Enable);
        osDelay(1);
    }
    for (int j = 0; j < 10; j++)
    {
        DM_Motor_Command(&FDCAN3_TxFrame, chassis_move.joint_motor[1], DM_Motor_Enable);
        osDelay(1);
    }
    for (int j = 0; j < 10; j++)
    {
        LK_Motor_Command(&FDCAN3_TxFrame, chassis_move.wheel_motor[0], LK_Motor_Enable); // 左边轮毂电机
        osDelay(1);
    }
}