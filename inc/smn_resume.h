#pragma once
#include <stdint.h>

// Init/poll/availability: main-loop only. No IRQ masking or timer pauses.
void smn_resume_init(void);
void smn_resume_poll(uint32_t ticks);
void smn_resume_request(void); // ISR-safe flag: recall/undo or brightness release; no IO.
uint8_t smn_resume_flash_available(void); // Gate existing synchronous preset calls.
