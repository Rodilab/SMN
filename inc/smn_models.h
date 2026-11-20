// Stable model identities are independent of their order in the selector.
#pragma once
#include <stdint.h>

// Plaits banks: green 0..15, red 16..31, orange 32..47.
// In each bank: MAIN = base + 2 * engine position (0..7), AUX = MAIN + 1.
// These are identities, not selector positions; unimplemented IDs stay reserved.
enum SmnModelId {
    SMN_MODEL_VIRTUAL_ANALOG_MAIN = 0,
    SMN_MODEL_VIRTUAL_ANALOG_AUX = 1,
    SMN_MODEL_WAVESHAPING_MAIN = 2,
    SMN_MODEL_WAVESHAPING_AUX = 3,
    SMN_MODEL_FM_MAIN = 4,
    SMN_MODEL_FM_AUX = 5,
    SMN_MODEL_FORMANT_MAIN = 6,
    SMN_MODEL_FORMANT_AUX = 7,
    SMN_MODEL_ADDITIVE_MAIN = 8,
    SMN_MODEL_ADDITIVE_AUX = 9,
    SMN_MODEL_WAVETABLE_MAIN = 10,
    SMN_MODEL_WAVETABLE_AUX = 11,
    SMN_MODEL_SPEECH_MAIN = 14,
    SMN_MODEL_SPEECH_AUX = 15,
    SMN_MODEL_SWARM_MAIN = 16,
    SMN_MODEL_SWARM_AUX = 17,
    SMN_MODEL_NOISE_MAIN = 18,
    SMN_MODEL_NOISE_AUX = 19,
    SMN_MODEL_PARTICLE_MAIN = 20,
    SMN_MODEL_PARTICLE_AUX = 21,
    SMN_MODEL_STRING_MAIN = 22,
    SMN_MODEL_MODAL_MAIN = 24,
    SMN_MODEL_MODAL_AUX = 25,
    SMN_MODEL_VIRTUAL_ANALOG_VCF_MAIN = 32,
    SMN_MODEL_VIRTUAL_ANALOG_VCF_AUX = 33,
    SMN_MODEL_PHASE_DISTORTION_MAIN = 34,
    SMN_MODEL_PHASE_DISTORTION_AUX = 35,
    SMN_MODEL_TERRAIN_MAIN = 42,
    SMN_MODEL_TERRAIN_AUX = 43,
    // SMN-specific variants; never take a reserved Plaits MAIN/AUX identity.
    SMN_MODEL_STRING_AUDIO_IN = 48,
    SMN_MODEL_MODAL_AUDIO_IN = 49,
    SMN_MODEL_ID_COUNT = 50 // ID space, not the implemented count.
};

// Four display pages, six MAIN/AUX pairs per page. Slots are zero-based
// and independent of stable IDs and the compact list of available models.
enum {
    SMN_MODEL_PAGE_COUNT = 4,
    SMN_MODELS_PER_PAGE = 12,
    SMN_MODEL_SLOT_COUNT = SMN_MODEL_PAGE_COUNT * SMN_MODELS_PER_PAGE
};

#ifdef __cplusplus
extern "C" {
#endif
uint8_t smn_model_count(void);
uint8_t smn_model_wrap(int32_t position);
uint8_t smn_model_id_at(uint8_t index);
uint8_t smn_model_index(uint8_t id);
uint8_t smn_model_valid(uint8_t id);
// Palette index / reserved display slot (0..47); unknown IDs use the first model.
uint8_t smn_model_color(uint8_t id);
// Zero-based ring position: 1,2,4,5,...,16,17 (user LEDs 2,3,5,6,...,17,18).
uint8_t smn_model_led(uint8_t id);
float smn_model_gain(uint8_t id);
#ifdef __cplusplus
}
#endif
