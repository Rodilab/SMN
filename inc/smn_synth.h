// C interface to the C++ synthesis engines; these types are not storage formats.
#pragma once

#include <stddef.h>
#include <stdint.h>
#include "smn_models.h"

#define SMN_SYNTH_MAX_VOICES 6u
#define SMN_SYNTH_BLOCK_SIZE 16u
#define SMN_SYNTH_SAMPLE_RATE_HZ 44100u

// Effective engine range, not a change to SWN pitch calculations.
// Finite frequencies above the maximum are rendered at this maximum.
#define SMN_SYNTH_MIN_FREQUENCY_HZ 0.1f
#define SMN_SYNTH_MAX_FREQUENCY_HZ 11025.0f

typedef struct {
    float frequency_hz;
    float harmonics;
    float timbre;
    float morph;
    uint8_t model_id; // Stable ID, not a selector position. Zero = Virtual Analog MAIN.
} SmnSynthParameters;

#ifdef __cplusplus
extern "C" {
#endif

// Call before audio starts, never concurrently with render. Reinitializes state.
// Returns 1 for 1..6 voices; otherwise disables rendering and returns 0.
int smn_synth_init(uint8_t voice_count);

// Non-reentrant. Parameters must be finite, timbre controls in [0,1], and
// frequency at least the minimum above (high values are capped). frames = 16.
// main_output receives the selected MAIN or AUX variant and must point to 16
// writable floats, not overlapping parameters. Unknown model IDs are rejected.
// Returns 1 on success. On failure, silences a non-null output only if frames
// is 16; otherwise returns 0 without writing. The selected output is at its
// native floating-point scale. Noise/Particle/Modal/Speech/Swarm include
// Plaits limiting/clipping. The caller applies model gain and SWN gain/pan.
int smn_synth_render(uint8_t voice,
                     const SmnSynthParameters *parameters,
                     float *main_output,
                     size_t frames);

// Same contract, with a read-only mono input of 16 finite samples in [-1,1].
// NULL means silence. Only the two SMN Audio In variants consume this input.
// Input and output must not overlap. No input buffer is retained after return.
int smn_synth_render_with_input(uint8_t voice,
                     const SmnSynthParameters *parameters,
                     const float *audio_input, float *main_output, size_t frames);

#ifdef __cplusplus
}
#endif
