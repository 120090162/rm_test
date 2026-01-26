/**
 ******************************************************************************
 * @file           : bsp_dwt.c
 * @brief          : bsp DWT functions
 * @author         : GrassFan Wang
 * @date           : 2025/02/17
 * @version        : v1.0
 ******************************************************************************
 * @attention      : none
 ******************************************************************************
 */
/* USER CODE END Header */

/* Include ---------------------------------------------------------*/
#include "bsp_dwt.h"
#include "stm32h723xx.h"

DWT_Time_Typedef SysTime;
uint64_t CYCCNT64;
/* Private variables ---------------------------------------------------------*/
static uint32_t CPU_FREQ_Hz;
static uint32_t CPU_FREQ_Hz_ms;
static uint32_t CPU_FREQ_Hz_us;
static uint32_t CYCCNT_RountCount;
static uint32_t CYCCNT_LAST;
static void BSP_DWT_CNT_Update(void);

void BSP_DWT_Init(uint32_t CPU_Freq_mHz)
{

	/* Initializes the DWT peripheral */
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

	/* clear the DWT CYCCNT register */
	DWT->CYCCNT = (uint32_t)0u;

	/* Enable Cortex-M DWT CYCCNT register */
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	/* update the clock frequence */
	CPU_FREQ_Hz = CPU_Freq_mHz * 1000000;
	CPU_FREQ_Hz_ms = CPU_FREQ_Hz / 1000;
	CPU_FREQ_Hz_us = CPU_FREQ_Hz / 1000000;
	CYCCNT_RountCount = 0;
}

float BSP_DWT_GetDeltaT(uint32_t *cnt_last)
{
	volatile uint32_t cnt_now = DWT->CYCCNT;
	float dt = ((uint32_t)(cnt_now - *cnt_last)) / ((float)(CPU_FREQ_Hz));
	*cnt_last = cnt_now;

	BSP_DWT_CNT_Update();

	return dt;
}

double BSP_DWT_GetDeltaT64(uint32_t *cnt_last)
{
	volatile uint32_t cnt_now = DWT->CYCCNT;
	double dt = ((uint32_t)(cnt_now - *cnt_last)) / ((double)(CPU_FREQ_Hz));
	*cnt_last = cnt_now;

	BSP_DWT_CNT_Update();

	return dt;
}

void BSP_DWT_SysTimeUpdate(void)
{
	volatile uint32_t cnt_now = DWT->CYCCNT;
	static uint64_t CNT_TEMP1, CNT_TEMP2, CNT_TEMP3;

	BSP_DWT_CNT_Update();

	CYCCNT64 = (uint64_t)CYCCNT_RountCount * (uint64_t)UINT32_MAX + (uint64_t)cnt_now;
	CNT_TEMP1 = CYCCNT64 / CPU_FREQ_Hz;
	CNT_TEMP2 = CYCCNT64 - CNT_TEMP1 * CPU_FREQ_Hz;
	SysTime.s = CNT_TEMP1;
	SysTime.ms = CNT_TEMP2 / CPU_FREQ_Hz_ms;
	CNT_TEMP3 = CNT_TEMP2 - SysTime.ms * CPU_FREQ_Hz_ms;
	SysTime.us = CNT_TEMP3 / CPU_FREQ_Hz_us;
}

float BSP_DWT_GetTimeline_s(void)
{
	BSP_DWT_SysTimeUpdate();

	float DWT_Timelinef32 = SysTime.s + SysTime.ms * 0.001f + SysTime.us * 0.000001f;

	return DWT_Timelinef32;
}

float BSP_DWT_GetTimeline_ms(void)
{
	BSP_DWT_SysTimeUpdate();

	float DWT_Timelinef32 = SysTime.s * 1000 + SysTime.ms + SysTime.us * 0.001f;

	return DWT_Timelinef32;
}

uint64_t BSP_DWT_GetTimeline_us(void)
{
	BSP_DWT_SysTimeUpdate();

	uint64_t DWT_Timelinef32 = SysTime.s * 1000000 + SysTime.ms * 1000 + SysTime.us;

	return DWT_Timelinef32;
}

static void BSP_DWT_CNT_Update(void)
{
	volatile uint32_t cnt_now = DWT->CYCCNT;

	if (cnt_now < CYCCNT_LAST)
		CYCCNT_RountCount++; // 进位

	CYCCNT_LAST = cnt_now;
}

void BSP_DWT_Delay(volatile float Delay)
{
	uint32_t tickstart = DWT->CYCCNT;
	float wait = Delay;

	while ((DWT->CYCCNT - tickstart) < wait * (float)CPU_FREQ_Hz)
	{
	}
}
