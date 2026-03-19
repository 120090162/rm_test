#include "Chassis_Task.h"
#include "User_Lib.h"

chassis_t chassis_move = {
    .mode = CHASSIS_OFF,
    .error_code = 0,
};
vmc_leg_t left;
vmc_leg_t right;

uint32_t CHASS_TIME = 1;

Calibrate_s CALIBRATE = {
    .velocity = {0.0f, 0.0f, 0.0f, 0.0f},
    .stop_time = {0, 0, 0, 0},
    .reached = {false, false, false, false},
    .calibrated = false,
};

PID_Info_TypeDef stand_up_pid;

void mySaturate(float *in, float min, float max)
{
    if (*in < min)
    {
        *in = min;
    }
    else if (*in > max)
    {
        *in = max;
    }
}

/**
 * @brief  Console function for chassis calibration mode, 计算校准时的电机控制量
 */
void ConsoleCalibrate(void)
{
    left.velocity_set[0] = -CALIBRATE_VELOCITY;
    left.velocity_set[1] = CALIBRATE_VELOCITY;
    right.velocity_set[0] = CALIBRATE_VELOCITY;
    right.velocity_set[1] = -CALIBRATE_VELOCITY;

    left.wheel_T = 0;
    right.wheel_T = 0;
}

/**
 * @brief  Console function for chassis stand-up mode, 计算初始站起时的电机控制量
 */
void ConsoleStandUp(void)
{
    // ===腿部位置控制===

    LocomotionController();

    float phi1_phi4_l[2], phi1_phi4_r[2];
    CalcPhi1AndPhi4(CHASSIS.ref.rod_Angle[0], CHASSIS.ref.rod_L0[0], phi1_phi4_l);
    CalcPhi1AndPhi4(CHASSIS.ref.rod_Angle[1], CHASSIS.ref.rod_L0[1], phi1_phi4_r);

    // 当解算出的角度正常时，设置目标角度
    if (!(isnan(phi1_phi4_l[0]) || isnan(phi1_phi4_l[1]) || isnan(phi1_phi4_r[0]) ||
          isnan(phi1_phi4_r[1])))
    {
        left.position_set[0] =
            theta_transform(phi1_phi4_l[1], -J0_ANGLE_OFFSET, J0_DIRECTION, 1);
        left.position_set[1] =
            theta_transform(phi1_phi4_l[0], -J1_ANGLE_OFFSET, J1_DIRECTION, 1);
        right.position_set[0] =
            theta_transform(phi1_phi4_r[1], -J2_ANGLE_OFFSET, J2_DIRECTION, 1);
        right.position_set[1] =
            theta_transform(phi1_phi4_r[0], -J3_ANGLE_OFFSET, J3_DIRECTION, 1);
    }
    // 检测设定角度是否超过电机角度限制
    left.position_set[0] =
        fp32_constrain(left.position_set[0], MIN_J0_ANGLE, MAX_J0_ANGLE);
    left.position_set[1] =
        fp32_constrain(left.position_set[1], MIN_J1_ANGLE, MAX_J1_ANGLE);
    right.position_set[0] =
        fp32_constrain(right.position_set[0], MIN_J2_ANGLE, MAX_J2_ANGLE);
    right.position_set[1] =
        fp32_constrain(right.position_set[1], MIN_J3_ANGLE, MAX_J3_ANGLE);

    // ===驱动轮pid控制===
    float feedforward = -220;
    PID_Calculate(&stand_up_pid, 0, INS.Pitch);
    left.wheel_T = (feedforward + stand_up_pid.Output) * W0_DIRECTION;
    right.wheel_T = (feedforward + stand_up_pid.Output) * W1_DIRECTION;
}