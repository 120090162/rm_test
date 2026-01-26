#ifndef BSP_DWT_H
#define BSP_DWT_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"

/* Exported types ------------------------------------------------------------*/
/**
 * @brief typedef structure that contains the information for the DWT timer.
 */
typedef struct
{
  uint32_t s;
  uint16_t ms;
  uint16_t us;
} DWT_Time_Typedef;

extern void BSP_DWT_Init(uint32_t CPU_Freq_mHz);
float BSP_DWT_GetDeltaT(uint32_t *cnt_last);
double BSP_DWT_GetDeltaT64(uint32_t *cnt_last);
float BSP_DWT_GetTimeline_s(void);
float BSP_DWT_GetTimeline_ms(void);
uint64_t BSP_DWT_GetTimeline_us(void);
void BSP_DWT_Delay(float Delay);
void BSP_DWT_SysTimeUpdate(void);

#endif