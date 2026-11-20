#pragma once
#include <stdint.h>

// Cooperative access to external sectors 0/1 only. Buffers remain alive until
// poll returns DONE. No waiting for a DMA transfer or flash program/erase.
enum SmnFlashResult { SMN_FLASH_BUSY, SMN_FLASH_DONE, SMN_FLASH_FAILED };
uint8_t smn_flash_read(uint32_t address, uint8_t *data, uint16_t size);
uint8_t smn_flash_program(uint32_t address, uint8_t *data, uint16_t size);
uint8_t smn_flash_erase(uint32_t address);
enum SmnFlashResult smn_flash_poll(uint32_t ticks);
uint8_t smn_flash_idle(void);
