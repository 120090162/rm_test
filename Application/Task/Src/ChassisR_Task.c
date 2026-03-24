#include "ChassisR_Task.h"

void ChassisR_task(void)
{
    while (INS.ins_flag == 0)
    { // 等待加速度收敛
        osDelay(1);
    }

    ChassisR_init(); // 初始化右边两个关节电机和右边轮毂电机的id和控制模式、初始化腿部

    while (1)
    {
        if (chassis_move.start_flag == 1)
        {
            switch (chassis_move.mode)
            {
            case CHASSIS_STAND_UP:
            {
                DM_Motor_CAN_TxMessage(&FDCAN3_TxFrame, chassis_move.joint_motor[2], right.position_set[0], 0, NORMAL_POS_KP, NORMAL_POS_KD, 0);
                osDelay(CHASS_TIME);
                DM_Motor_CAN_TxMessage(&FDCAN3_TxFrame, chassis_move.joint_motor[3], right.position_set[1], 0, NORMAL_POS_KP, NORMAL_POS_KD, 0);
                osDelay(CHASS_TIME);
                LK_Motor_CAN_TxMessage(&FDCAN3_TxFrame, chassis_move.wheel_motor[1], 0, right.wheel_T);
                osDelay(CHASS_TIME);
            }
            break;
            case CHASSIS_CALIBRATE:
            {
                DM_Motor_CAN_TxMessage(&FDCAN3_TxFrame, chassis_move.joint_motor[2], 0, right.velocity_set[0], 0, CALIBRATE_VEL_KD, 0);
                osDelay(CHASS_TIME);
                DM_Motor_CAN_TxMessage(&FDCAN3_TxFrame, chassis_move.joint_motor[3], 0, right.velocity_set[1], 0, CALIBRATE_VEL_KD, 0);
                osDelay(CHASS_TIME);
                LK_Motor_CAN_TxMessage(&FDCAN3_TxFrame, chassis_move.wheel_motor[1], 0, right.wheel_T);
                osDelay(CHASS_TIME);

                if (CALIBRATE.reached[2] && CALIBRATE.reached[3])
                {
                    DM_Motor_Command(&FDCAN3_TxFrame, chassis_move.joint_motor[2], DM_Motor_Save_Zero_Position);
                    osDelay(CHASS_TIME);
                    DM_Motor_Command(&FDCAN3_TxFrame, chassis_move.joint_motor[3], DM_Motor_Save_Zero_Position);
                    osDelay(CHASS_TIME * 2);
                }
            }
            break;
            case CHASSIS_SAFE:
            default:
            {
                DM_Motor_CAN_TxMessage(&FDCAN3_TxFrame, chassis_move.joint_motor[2], 0, 0, 0, ZERO_FORCE_VEL_KD, 0);
                osDelay(CHASS_TIME);
                DM_Motor_CAN_TxMessage(&FDCAN3_TxFrame, chassis_move.joint_motor[3], 0, 0, 0, ZERO_FORCE_VEL_KD, 0);
                osDelay(CHASS_TIME);
                LK_Motor_CAN_TxMessage(&FDCAN3_TxFrame, chassis_move.wheel_motor[1], 0, 1);
                osDelay(CHASS_TIME);
            }
            }
        }
        else
        {
            // 失能状态，电机停止
            DM_Motor_CAN_TxMessage(&FDCAN3_TxFrame, chassis_move.joint_motor[2], 0, 0, 0, 0, 0);
            osDelay(CHASS_TIME);
            DM_Motor_CAN_TxMessage(&FDCAN3_TxFrame, chassis_move.joint_motor[3], 0, 0, 0, 0, 0);
            osDelay(CHASS_TIME);
            LK_Motor_CAN_TxMessage(&FDCAN3_TxFrame, chassis_move.wheel_motor[1], 0, 0);
            osDelay(CHASS_TIME);
        }
    }
}

void ChassisR_init(void)
{
    chassis_move.joint_motor[2] = &DM_8009_Motor[2];
    chassis_move.joint_motor[3] = &DM_8009_Motor[3];

    chassis_move.wheel_motor[1] = &LK_9025_Motor[1];

    for (int j = 0; j < 10; j++)
    {
        DM_Motor_Command(&FDCAN3_TxFrame, chassis_move.joint_motor[2], DM_Motor_Enable);
        osDelay(1);
    }
    for (int j = 0; j < 10; j++)
    {
        DM_Motor_Command(&FDCAN3_TxFrame, chassis_move.joint_motor[3], DM_Motor_Enable);
        osDelay(1);
    }
    for (int j = 0; j < 10; j++)
    {
        LK_Motor_Command(&FDCAN3_TxFrame, chassis_move.wheel_motor[1], LK_Motor_Enable); // 右边轮毂电机
        osDelay(1);
    }
}