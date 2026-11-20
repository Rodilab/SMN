// SMN model/dispersion metadata uses reserved bytes; preset sizes stay fixed.
#pragma once
#include "params_update.h"
#include "params_lfo.h"

// Four independent targeted locks; global channel locks remain separate.
enum SmnParameterLock { SMN_LOCK_HARMONICS, SMN_LOCK_TIMBRE, SMN_LOCK_MORPH,
                        SMN_LOCK_PITCH, SMN_LOCK_NONE };
// RAM metadata in previously unused PADDING[88..97], tag + six masks.
void smn_parameter_locks_init(o_params *state);
void smn_parameter_locks_restore(o_params *state);
static inline uint8_t smn_parameter_locked(const o_params *state, uint8_t voice, uint8_t parameter)
{
    return parameter < SMN_LOCK_NONE && (state->PADDING[92 + voice] & (1u << parameter));
}
static inline void smn_parameter_lock_set(o_params *state, uint8_t voice, uint8_t mask)
{
    state->PADDING[92 + voice] = mask & 15u;
}

void smn_model_selection_init(o_params *state);
void smn_model_selection_capture(o_params *state);
void smn_model_selection_restore(o_params *state);

// Global encoder intensity for each axis (H/T/M), bounded 0..1. H uses the
// historical field; T/M have an independent version tag in reserved bytes.
void smn_dispersion_init(o_params *state);
float smn_dispersion_get(const o_params *state, uint8_t dimension);
void smn_dispersion_set(o_params *state, uint8_t dimension, float value);
// Normalize the current RAM metadata; no flash I/O or old-format conversion.
void smn_dispersion_restore(o_params *state);

// PADDING[24..87]: BRH/version 2, then A-F x gain/H/T/M/pitch uint16 LE (0..480).
// Initialization is RAM-only. Capture/restore live doses only for active state;
// inactive preset/undo objects carry their own metadata without touching audio.
void smn_breathe_init(o_params *state);
void smn_breathe_capture(o_params *state);
void smn_breathe_restore(const o_params *state, const o_lfos *lfos);
