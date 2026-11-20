#pragma once

#include <stm32f7xx.h>
#include "flash_S25FL127.h"

// Unused by the original SWN, including sphere storage and hardware tests.
// Dedicated SMN resume journal; never reduce MAX_PRESETS or move other areas.
#define SMN_RESUME_SECTOR_A 0
#define SMN_RESUME_SECTOR_B 1
#define 	STARTUP_PRESET_SETTING_SECTOR 14
// Former sphere sectors stay reserved; removing their code must not move presets.
#define 	WT_SECTOR_START 		16
#define		PRESET_SECTOR_START		217
#define		PRESETS_PER_SECTOR		2

#define		MAX_WT_IN_FLASH  		(PRESET_SECTOR_START - WT_SECTOR_START - 1)									/* 200 */
#define		MAX_PRESETS  			((sFLASH_SPI_NUM_SECTORS - PRESET_SECTOR_START) * PRESETS_PER_SECTOR)		/* 108 */
