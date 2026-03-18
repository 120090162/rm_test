#include "Chassis_Task.h"

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