#include "smn_models.h"
#include <stddef.h>

typedef struct {
    uint8_t color; // Reserved display slot 0..47; its palette is prepared in led_cont.c.
    uint8_t available; // Zero-filled reserved entries are not selectable.
    float gain;
} SmnModel;

// Indexed by stable ID, never by selector position. Audio reads these directly
// in constant time; reordering the UI must not change the cost of a model.
// Gains match each engine's OUT/AUX post-processing settings in Plaits.
static const SmnModel models[SMN_MODEL_ID_COUNT] = {
    [SMN_MODEL_WAVESHAPING_MAIN] = {4, 1, 0.7f},
    [SMN_MODEL_WAVESHAPING_AUX] = {5, 1, 0.6f},
    [SMN_MODEL_VIRTUAL_ANALOG_VCF_MAIN] = {0, 1, 1.0f},
    [SMN_MODEL_VIRTUAL_ANALOG_VCF_AUX] = {1, 1, 1.0f},
    [SMN_MODEL_VIRTUAL_ANALOG_MAIN] = {2, 1, 0.8f},
    [SMN_MODEL_VIRTUAL_ANALOG_AUX] = {3, 1, 0.8f},
    [SMN_MODEL_PHASE_DISTORTION_MAIN] = {14, 1, 0.7f},
    [SMN_MODEL_PHASE_DISTORTION_AUX] = {15, 1, 0.7f},
    [SMN_MODEL_ADDITIVE_MAIN] = {16, 1, 0.8f},
    [SMN_MODEL_ADDITIVE_AUX] = {17, 1, 0.8f},
    // Noise's Plaits limiter is applied by the adapter; no second fixed gain.
    [SMN_MODEL_NOISE_MAIN] = {30, 1, 1.0f},
    [SMN_MODEL_NOISE_AUX] = {31, 1, 1.0f},
    // Particle MAIN limiter includes pre-gain 2; AUX has unity gain.
    [SMN_MODEL_PARTICLE_MAIN] = {32, 1, 1.0f},
    [SMN_MODEL_PARTICLE_AUX] = {33, 1, 1.0f},
    // Modal MAIN uses its Plaits limiter in the adapter; AUX is the exciter.
    [SMN_MODEL_MODAL_MAIN] = {25, 1, 1.0f},
    [SMN_MODEL_MODAL_AUX] = {28, 1, 0.8f},
    [SMN_MODEL_STRING_AUDIO_IN] = {26, 1, 1.0f},
    [SMN_MODEL_MODAL_AUDIO_IN] = {27, 1, 1.0f},
    [SMN_MODEL_FORMANT_MAIN] = {6, 1, 0.7f},
    [SMN_MODEL_FORMANT_AUX] = {7, 1, 0.6f},
    // MAIN limiter includes pre-gain 0.7, AUX fixed post-gain 0.8.
    [SMN_MODEL_SPEECH_MAIN] = {22, 1, 1.0f},
    [SMN_MODEL_SPEECH_AUX] = {23, 1, 0.8f},
    // Swarm MAIN limiter applies pre-gain 3; AUX has unity gain.
    [SMN_MODEL_SWARM_MAIN] = {8, 1, 1.0f},
    [SMN_MODEL_SWARM_AUX] = {9, 1, 1.0f},
    [SMN_MODEL_WAVETABLE_MAIN] = {18, 1, 0.6f},
    [SMN_MODEL_WAVETABLE_AUX] = {19, 1, 0.6f},
    [SMN_MODEL_TERRAIN_MAIN] = {20, 1, 0.7f},
    [SMN_MODEL_TERRAIN_AUX] = {21, 1, 0.7f},
    [SMN_MODEL_FM_MAIN] = {12, 1, 0.6f},
    [SMN_MODEL_FM_AUX] = {13, 1, 0.6f},
    // String MAIN uses its Plaits limiter in the adapter; AUX ID 23 stays reserved.
    [SMN_MODEL_STRING_MAIN] = {24, 1, 1.0f}
};

// Only available models, in Browse/CV order. Keep each ID exactly once.
// Unassigned display slots stay empty and are skipped by Browse/CV.
static const uint8_t model_order[] = {
    SMN_MODEL_VIRTUAL_ANALOG_VCF_MAIN,
    SMN_MODEL_VIRTUAL_ANALOG_VCF_AUX,
    SMN_MODEL_VIRTUAL_ANALOG_MAIN,
    SMN_MODEL_VIRTUAL_ANALOG_AUX,
    SMN_MODEL_WAVESHAPING_MAIN,
    SMN_MODEL_WAVESHAPING_AUX,
    SMN_MODEL_FORMANT_MAIN,
    SMN_MODEL_FORMANT_AUX,
    SMN_MODEL_SWARM_MAIN,
    SMN_MODEL_SWARM_AUX,
    SMN_MODEL_FM_MAIN,
    SMN_MODEL_FM_AUX,
    SMN_MODEL_PHASE_DISTORTION_MAIN,
    SMN_MODEL_PHASE_DISTORTION_AUX,
    SMN_MODEL_ADDITIVE_MAIN,
    SMN_MODEL_ADDITIVE_AUX,
    SMN_MODEL_WAVETABLE_MAIN,
    SMN_MODEL_WAVETABLE_AUX,
    SMN_MODEL_TERRAIN_MAIN,
    SMN_MODEL_TERRAIN_AUX,
    SMN_MODEL_SPEECH_MAIN,
    SMN_MODEL_SPEECH_AUX,
    SMN_MODEL_STRING_MAIN,
    SMN_MODEL_MODAL_MAIN,
    SMN_MODEL_STRING_AUDIO_IN,
    SMN_MODEL_MODAL_AUDIO_IN,
    SMN_MODEL_MODAL_AUX,
    SMN_MODEL_NOISE_MAIN,
    SMN_MODEL_NOISE_AUX,
    SMN_MODEL_PARTICLE_MAIN,
    SMN_MODEL_PARTICLE_AUX
};
_Static_assert(sizeof(model_order) <= SMN_MODEL_ID_COUNT, "Selector exceeds reserved IDs");
_Static_assert(sizeof(model_order) <= 127, "Selector storage limit");

uint8_t smn_model_count(void) { return sizeof(model_order); }

uint8_t smn_model_wrap(int32_t position)
{
    int32_t wrapped = position % smn_model_count();
    return wrapped < 0 ? wrapped + smn_model_count() : wrapped;
}

uint8_t smn_model_id_at(uint8_t index)
{
    return index < smn_model_count() ? model_order[index] : model_order[0];
}

uint8_t smn_model_index(uint8_t id)
{
    if (!smn_model_valid(id)) return 0;
    // Only UI and preset recall need this reverse lookup, never audio.
    for (uint8_t i = 0; i < smn_model_count(); ++i)
        if (model_order[i] == id) return i;
    return 0; // Unreachable with a complete selector list.
}

uint8_t smn_model_valid(uint8_t id)
{
    return id < SMN_MODEL_ID_COUNT && models[id].available;
}

uint8_t smn_model_color(uint8_t id)
{
    return models[smn_model_valid(id) ? id : model_order[0]].color;
}

uint8_t smn_model_led(uint8_t id)
{
    const uint8_t slot = smn_model_color(id) % SMN_MODELS_PER_PAGE;
    return 3 * (slot / 2) + 1 + slot % 2;
}

float smn_model_gain(uint8_t id)
{
    return models[smn_model_valid(id) ? id : model_order[0]].gain;
}
