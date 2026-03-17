#include "Chassis_Task.h"

chassis_t chassis_move;
vmc_leg_t left;
vmc_leg_t right;

uint32_t CHASS_TIME = 1;

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