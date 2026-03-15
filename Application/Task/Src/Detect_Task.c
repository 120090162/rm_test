/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : Detect_Task.h
 * @brief          : Detect task
 * @author         : GrassFan Wang
 * @date           : 2025/01/22
 * @version        : v1.0
 ******************************************************************************
 * @attention      : None
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "Detect_Task.h"
// #include "Control_Task.h"
#include "Remote_Control.h"
#include "bsp_gpio.h"

float VBAT_WARNNING_VAL;
float VBAT_LOW_VAL;
float vbus;
bool vbus_low_warning;

/**
 * @note turn on:  800
 *       turn off: 4150
 */

/* USER CODE BEGIN Header_Detect_Task */
/**
 * @brief Function implementing the StartDetectTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Detect_Task */
void Detect_Task(void)
{
    /* USER CODE BEGIN Detect_Task */
    //  TickType_t systick = 0;
    VBAT_WARNNING_VAL = VBAT_WARNNING_VAL_6S;
    VBAT_LOW_VAL = VBAT_LOW_VAL_6S;
    vbus_low_warning = false;
    /* Infinite loop */
    for (;;)
    {
        // 监测电压
        vbus = (adc_val[0] * 3.3f / 65535) * 11.0f;
        if (vbus < VBAT_WARNNING_VAL)
        {

            vbus_low_warning = true;
        }
        else
        {
            vbus_low_warning = false;
        }

        Remote_Message_Moniter(&remote_ctrl); // 监测遥控器在线状态

        osDelay(100);
    }
    /* USER CODE END Detect_Task */
}
