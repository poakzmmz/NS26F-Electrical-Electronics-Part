/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file   fatfs.c
  * @brief  Code for fatfs applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
#include "fatfs.h"

uint8_t retSD;    /* Return value for SD */
char SDPath[4];   /* SD logical drive path */
FATFS SDFatFS;    /* File system object for SD logical drive */
FIL SDFile;       /* File object for SD */

/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

void MX_FATFS_Init(void)
{
  /*## FatFS: Link the SD driver ###########################*/
  retSD = FATFS_LinkDriver(&SD_Driver, SDPath);

  /* USER CODE BEGIN Init */
  /* additional user code for init */
  /* USER CODE END Init */
}

/**
  * @brief  Gets Time from RTC (overridden with GPS time when available)
  * @param  None
  * @retval Time in DWORD (FAT timestamp format)
  */
extern volatile uint32_t g_gps_fat_time;

DWORD get_fattime(void)
{
  /* USER CODE BEGIN get_fattime */
  if (g_gps_fat_time != 0U) {
    return g_gps_fat_time;
  }
  /* GPS 미연동 시 구분하기 쉽도록 기준일인 2000-01-01 00:00:00 반환
   * (Year = 2000 - 1980 = 20, Month = 1, Day = 1) */
  return ((20UL << 25) | (1UL << 21) | (1UL << 16) | (0UL << 11) | (0UL << 5) | 0UL);
  /* USER CODE END get_fattime */
}

/* USER CODE BEGIN Application */

/* USER CODE END Application */
