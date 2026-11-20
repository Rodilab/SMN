#include "smn_model_selection.h"
#include "smn_models.h"
#include <math.h>
#include <string.h>

// Four-byte tag/version followed by six model IDs for the manual choices,
// before the CV offset is applied.
static const uint8_t tag[] = {'S', 'M', 'N', 1};
_Static_assert(sizeof(((o_params *)0)->PADDING) >= sizeof(tag) + NUM_CHANNELS,
               "Model metadata must fit reserved preset storage");

void smn_model_selection_capture(o_params *state)
{
    memcpy(state->PADDING, tag, sizeof(tag));
    for (uint8_t v = 0; v < NUM_CHANNELS; ++v) {
        const uint8_t index = smn_model_wrap(state->wtsel_enc[v]);
        state->PADDING[sizeof(tag) + v] = smn_model_id_at(index);
    }
}

void smn_model_selection_init(o_params *state)
{
    state->wtsel_cv = 0;
    state->wtsel_spread_cv = 0;
    for (uint8_t v = 0; v < NUM_CHANNELS; ++v) {
        state->wtsel_enc[v] = 0; // First available model, independent of its stable ID.
        state->wtsel_spread_enc[v] = 0;
    }
    smn_model_selection_capture(state);
}

void smn_model_selection_restore(o_params *state)
{
    if (memcmp(state->PADDING, tag, sizeof(tag))) {
        smn_model_selection_init(state);
        return;
    }
    for (uint8_t v = 0; v < NUM_CHANNELS; ++v) {
        uint8_t id = state->PADDING[sizeof(tag) + v];
        // The saved ID includes the former WT Spread offset. Use that
        // combined choice when restoring to the current selector.
        state->wtsel_enc[v] = smn_model_index(id);
        state->wtsel_spread_enc[v] = 0;
    }
    state->wtsel_spread_cv = 0;
    smn_model_selection_capture(state);
}

// Separate versioned metadata preserves the existing SMN model ID tag. H uses
// the historical float; T/M use reserved bytes. memcpy avoids aliasing casts.
enum { DISPERSION_TAG_OFFSET = 12, DISPERSION_VALUES_OFFSET = 16 };
static const uint8_t dispersion_tag[] = {'D', 'S', 'P', 1};
_Static_assert(sizeof(float) == 4, "Preset float size");
_Static_assert(sizeof(((o_params *)0)->PADDING) >= DISPERSION_VALUES_OFFSET + 2 * sizeof(float),
               "Dispersion metadata must fit reserved preset storage");

static float bounded_intensity(float value)
{
    if (!isfinite(value) || value < 0.0f) return 0.0f;
    return value > 1.0f ? 1.0f : value;
}

static uint8_t has_dispersion(const o_params *state)
{
    return !memcmp(state->PADDING + DISPERSION_TAG_OFFSET,
                    dispersion_tag, sizeof(dispersion_tag));
}

static void init_dispersion_metadata(o_params *state)
{
    const float zero[2] = {0};
    memcpy(state->PADDING + DISPERSION_VALUES_OFFSET, zero, sizeof(zero));
    memcpy(state->PADDING + DISPERSION_TAG_OFFSET, dispersion_tag, sizeof(dispersion_tag));
}

void smn_dispersion_init(o_params *state)
{
    state->dispersion_enc = 0.0f;
    init_dispersion_metadata(state);
}

float smn_dispersion_get(const o_params *state, uint8_t dimension)
{
    float value = 0.0f;
    if (dimension == 0)
        value = state->dispersion_enc;
    else if (dimension < 3 && has_dispersion(state))
        memcpy(&value, state->PADDING + DISPERSION_VALUES_OFFSET
                        + (dimension - 1) * sizeof(float), sizeof(value));
    return bounded_intensity(value);
}

void smn_dispersion_set(o_params *state, uint8_t dimension, float value)
{
    if (dimension >= 3) return;
    value = bounded_intensity(value);
    if (!has_dispersion(state)) init_dispersion_metadata(state);
    if (dimension == 0)
        state->dispersion_enc = value;
    else
        memcpy(state->PADDING + DISPERSION_VALUES_OFFSET
                + (dimension - 1) * sizeof(float), &value, sizeof(value));
}

void smn_dispersion_restore(o_params *state)
{
    if (!has_dispersion(state)) {
        // Older saves used a 0..2 encoder range; fold it back to a 0..1 amount.
        float old = state->dispersion_enc;
        if (!isfinite(old) || old < 0.0f) old = 0.0f;
        if (old > 2.0f) old = 2.0f;
        state->dispersion_enc = old > 1.0f ? 2.0f - old : old;
        init_dispersion_metadata(state); // Start the added T/M amounts at zero without rewriting the save.
    }
    for (uint8_t dimension = 0; dimension < 3; ++dimension)
        smn_dispersion_set(state, dimension, smn_dispersion_get(state, dimension));
}

enum { BREATHE_TAG_OFFSET = 24, BREATHE_VALUES_OFFSET = 28 };
// Version 2: all destinations use 0..480 Fine ticks. Version 1 is ignored.
static const uint8_t breathe_tag[] = {'B', 'R', 'H', 2};
_Static_assert(NUM_CHANNELS == 6 && SMN_MOD_DESTINATIONS == 6,
               "Five legacy destinations plus separate pan extension");
_Static_assert(SMN_MOD_GAIN == 0 && SMN_MOD_HARMONICS == 1 && SMN_MOD_TIMBRE == 2 &&
               SMN_MOD_MORPH == 3 && SMN_MOD_PITCH == 4,
               "Preserve the Breathe preset destination order");
_Static_assert(sizeof(((o_params *)0)->PADDING) >= BREATHE_VALUES_OFFSET + 60,
               "Breathe counters must fit reserved preset storage");
_Static_assert(sizeof(((o_params *)0)->PADDING) >= 116, "Pan extension fits RAM metadata");

void smn_breathe_init(o_params *state)
{
    memcpy(state->PADDING + BREATHE_TAG_OFFSET, breathe_tag, sizeof(breathe_tag));
    memset(state->PADDING + BREATHE_VALUES_OFFSET, 0, 60);
    memcpy(state->PADDING + 100, "PAN\1", 4);
    memset(state->PADDING + 104, 0, 12);
}

void smn_breathe_capture(o_params *state)
{
    memcpy(state->PADDING + BREATHE_TAG_OFFSET, breathe_tag, sizeof(breathe_tag));
    memcpy(state->PADDING + 100, "PAN\1", 4);
    for (unsigned v = 0; v < NUM_CHANNELS; ++v) {
        uint16_t steps[SMN_MOD_DESTINATIONS];
        smn_modulation_capture_steps(v, steps);
        state->PADDING[104 + 2*v] = (uint8_t)steps[SMN_MOD_PAN];
        state->PADDING[105 + 2*v] = (uint8_t)(steps[SMN_MOD_PAN] >> 8);
        for (unsigned d = 0; d < 5; ++d) {
            uint8_t *dst = state->PADDING + BREATHE_VALUES_OFFSET
                + 2 * (v * 5 + d);
            dst[0] = (uint8_t)steps[d];
            dst[1] = (uint8_t)(steps[d] >> 8);
        }
    }
}

void smn_breathe_restore(const o_params *state, const o_lfos *lfos)
{
    const uint8_t present = !memcmp(state->PADDING + BREATHE_TAG_OFFSET,
                                   breathe_tag, sizeof(breathe_tag));
    for (unsigned v = 0; v < NUM_CHANNELS; ++v) {
        uint16_t steps[SMN_MOD_DESTINATIONS] = {0};
        if (present) {
            for (unsigned d = 0; d < 5; ++d) {
                const uint8_t *src = state->PADDING + BREATHE_VALUES_OFFSET
                    + 2 * (v * 5 + d);
                steps[d] = (uint16_t)src[0] | ((uint16_t)src[1] << 8);
            }
        }
        if (!memcmp(state->PADDING + 100, "PAN\1", 4))
            steps[SMN_MOD_PAN] = state->PADDING[104 + 2*v] |
                ((uint16_t)state->PADDING[105 + 2*v] << 8);
        // Same gain exclusion as read_level_and_pan(); other doses survive.
        if (state->key_sw[v] != ksw_MUTE || lfos->to_vca[v])
            steps[SMN_MOD_GAIN] = 0;
        smn_modulation_restore_steps(v, steps);
    }
}

_Static_assert(sizeof(((o_params *)0)->PADDING) >= 98, "Targeted locks fit reserved RAM metadata");
void smn_parameter_locks_init(o_params *state)
{
    memcpy(state->PADDING + 88, "LCK\1", 4);
    memset(state->PADDING + 92, 0, NUM_CHANNELS);
}
void smn_parameter_locks_restore(o_params *state)
{
    if (memcmp(state->PADDING + 88, "LCK\1", 4)) smn_parameter_locks_init(state);
    for (unsigned v = 0; v < NUM_CHANNELS; ++v) state->PADDING[92 + v] &= 15u;
}
