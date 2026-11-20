// Six SMN chord memories. No processing in the audio callback.
#pragma once
#include <stdint.h>

void smn_chords_reset(void);
void smn_chord_store(uint8_t memory);
void smn_chord_recall(uint8_t memory);
void smn_chord_recall_cv(uint8_t memory);
int8_t smn_chord_cv_feedback(uint32_t ticks);
void smn_chords_poll_cv(uint32_t ticks);
uint8_t spread_cv_is_global_gate(void);

// Preset gestures live in params_update.c. Interrupt-side shortcuts may cancel
// them, but only the main loop can store/recall a chord.
void cancel_chord_gesture(void);
uint8_t chord_buttons_green(void);
int8_t chord_saved_feedback(uint32_t ticks);
