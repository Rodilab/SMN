#include "smn_patch.h"
#include "smn_model_selection.h"
#include "smn_models.h"
#include "quantz_scales.h"
#include "params_lfo_period.h"
#include <stddef.h>
#include <string.h>
#include <math.h>
extern o_params params;
_Static_assert(NUM_CHANNELS == 6 && SMN_MOD_DESTINATIONS == 6, "SMN2 voice/destination counts");
_Static_assert(sizeof(float) == 4, "SMN2 float32 fields");
_Static_assert(offsetof(SmnSettings, polyrotate) == 503, "Use former zero padding only");
_Static_assert(sizeof(SmnSettings) == 504, "Preset v3 / resume v5 settings");
_Static_assert(sizeof(SmnPatchExtra) == 216, "Six chords and six glide fields");
_Static_assert(offsetof(o_params, smn_extra) == SMN_LEGACY_PARAMS_BYTES, "Legacy params prefix");
_Static_assert(sizeof(SmnPatchState) == 732, "SMN2 state layout");
_Static_assert(offsetof(SmnPatchState, pan_breathe) == 720, "Keep v3/v5 prefix intact");
_Static_assert(sizeof(SmnPresetPlayback) == 44, "SMN2 playback layout");
_Static_assert(sizeof(SmnPresetRecord) == SMN_PRESET_BYTES, "SWN neighbor copies exactly 1024 bytes");
_Static_assert(offsetof(SmnPresetRecord, state) == 16, "SMN2 payload offset");
_Static_assert(offsetof(SmnPresetRecord, reserved) == 792, "SMN2 reserve offset");

// Reuse the actual Spread intervals: A is unison, B-F are Spread 1-5.
extern const int8_t CHORD_LIST[NUM_CHORDS][NUM_CHANNELS];
void smn_patch_chords_init(SmnPatchExtra *e)
{
    for (unsigned m = 0; m < SMN_CHORD_MEMORIES; ++m) {
        e->chord_scale[m] = sclm_NONE;
        e->chord_root[m] = 0;
        for (unsigned v = 0; v < NUM_CHANNELS; ++v) {
            e->chord_pitch[m][v] = CHORD_LIST[m][v];
            e->chord_octave[m][v] = INIT_OCT;
        }
    }
}

void smn_patch_extra_init(SmnPatchExtra *e)
{
    memset(e, 0, sizeof(*e));
    smn_patch_chords_init(e);
}
_Static_assert(offsetof(SmnPatchExtra, chord_scale) == SMN_PATCH_EXTRA_V1_BYTES,
               "Keep old chord/octave/glide prefix readable");
void smn_patch_extra_sanitize(SmnPatchExtra *e)
{
    for (unsigned v = 0; v < SMN_PATCH_VOICES; ++v)
        if (e->glide_ms[v] > SMN_GLIDE_MAX_MS) e->glide_ms[v] = 0;
    for (unsigned m = 0; m < 6; ++m) {
        if (e->chord_scale[m] >= NUM_QTZ_SCALES || e->chord_root[m] >= 12) {
            e->chord_scale[m] = sclm_NONE;
            e->chord_root[m] = 0;
        }
    }
    for (unsigned m = 0; m < 6; ++m) for (unsigned v = 0; v < 6; ++v) {
        if (e->chord_pitch[m][v] < -512 || e->chord_pitch[m][v] > 512 ||
            e->chord_octave[m][v] < MIN_OCT || e->chord_octave[m][v] > MAX_OCT) {
            e->chord_pitch[m][v] = 0;
            e->chord_octave[m][v] = INIT_OCT;
        }
    }
}

uint32_t smn_storage_crc(const uint8_t *data, unsigned size, unsigned crc_offset)
{
    // CRC32/ISO-HDLC. Nibble table keeps one-off work and flash footprint small.
    static const uint32_t table[16] = {
        0,0x1db71064,0x3b6e20c8,0x26d930ac,0x76dc4190,0x6b6b51f4,0x4db26158,0x5005713c,
        0xedb88320,0xf00f9344,0xd6d6a3e8,0xcb61b38c,0x9b64c2b0,0x86d3d2d4,0xa00ae278,0xbdbdf21c
    };
    uint32_t crc = UINT32_MAX;
    for (unsigned i = 0; i < size; ++i) {
        crc ^= i >= crc_offset && i < crc_offset + 4 ? 0 : data[i]; // CRC field is logically zero.
        crc = (crc >> 4) ^ table[crc & 15];
        crc = (crc >> 4) ^ table[crc & 15];
    }
    return ~crc;
}

static uint8_t in_range(float x, float low, float high)
{
    return isfinite(x) && x >= low && x <= high;
}

uint8_t smn_settings_valid(const SmnSettings *s)
{
    // Unknown LFO IDs are allowed here and normalized per voice on restore.
    // CRC and all other semantic checks still decide record validity.
    if (s->phase_switch > 1 || !scale_config_valid(s->scale, s->scale_buf)) return 0;
    for (unsigned d = 0; d < 3; ++d) {
        if (!in_range(s->dispersion[d], 0, 1)) return 0;
        for (unsigned v = 0; v < 6; ++v) if (!isfinite(s->nav[d][v])) return 0;
    }
    for (unsigned v = 0; v < 6; ++v) {
        if (!smn_model_valid(s->model[v]) || !in_range(s->pan[v], 0, 1) ||
            s->octave[v] < -MAX_OCT || s->octave[v] > MAX_OCT * 2 ||
            s->fine[v] < MIN_FINETUNE_WRAP || s->fine[v] > MAX_FINETUNE_WRAP ||
            s->keymode[v] >= NUM_MUTE_NOTE_KEY_STATES || s->voct[v] >= NUM_VOCT_VCA_STATES ||
            s->note[v] > 1 || s->note_buf[v] > 1 || s->locked[v] > 1 ||
            s->mode[v] >= NUM_LFO_MODES || s->mode_buf[v] >= NUM_LFO_MODES ||
            s->to_vca[v] > 1 || s->to_vca_buf[v] > 1 ||
            !in_range(s->gain[v], 0, 1) || !in_range(s->gain_buf[v], 0, 1) ||
            !in_range(s->phase[v], 0, LFO_PHASE_TABLELEN) ||
            s->phase_buf[v] < 0 || s->phase_buf[v] >= LFO_PHASE_TABLELEN ||
            !in_range(s->speed[v], LFO_MIN_DIVMULT_ID, LFO_MAX_DIVMULT_ID) ||
            !in_range(s->speed_buf[v], LFO_MIN_DIVMULT_ID, LFO_MAX_DIVMULT_ID) ||
            !in_range(s->locked_speed[v], LFO_MIN_DIVMULT_ID, LFO_MAX_DIVMULT_ID + NUM_DIVMULTS)) return 0;
        // Position-lock byte: bit 0 legacy global CV lock, bits 1..4 H/T/M/Pitch.
        if (s->locks[0][v] > 1 || s->locks[1][v] > 31 || s->locks[2][v] > 1) return 0;
        for (unsigned d = 0; d < 5; ++d)
            if (s->breathe[v][d] > SMN_MOD_FULL_STEPS)
                return 0;
    }
    return in_range(s->speed[GLO_CLK], LFO_MIN_DIVMULT_ID, LFO_MAX_DIVMULT_ID);
}

void smn_settings_capture(SmnSettings *s, const o_params *p, const o_lfos *l)
{
    __asm__ volatile("" ::: "memory");
    memset(s, 0, sizeof(*s)); // deterministic structure padding, too.
    memcpy(s->nav, p->wt_nav_enc, sizeof(s->nav));
    for (unsigned d = 0; d < 3; ++d) s->dispersion[d] = smn_dispersion_get(p, d);
    memcpy(s->pan, p->pan, sizeof(s->pan));
    memcpy(s->speed, l->divmult_id, sizeof(s->speed)); // exclude measured REF_CLK
    memcpy(s->phase, l->phase_id, sizeof(s->phase));
    memcpy(s->gain, l->gain, sizeof(s->gain));
    memcpy(s->speed_buf, l->divmult_id_buf, sizeof(s->speed_buf));
    memcpy(s->gain_buf, l->gain_buf, sizeof(s->gain_buf));
    memcpy(s->transpose, p->transpose_enc, sizeof(s->transpose));
    memcpy(s->spread, p->spread_enc, sizeof(s->spread));
    memcpy(s->fine, p->finetune, sizeof(s->fine));
    memcpy(s->octave, p->oct, sizeof(s->octave));
    memcpy(s->scale, p->indiv_scale, sizeof(s->scale));
    memcpy(s->scale_buf, p->indiv_scale_buf, sizeof(s->scale_buf));
    memcpy(s->shape, l->shape, sizeof(s->shape));
    memcpy(s->shape_buf, l->shape_buf, sizeof(s->shape_buf));
    memcpy(s->phase_buf, l->phase_id_buf, sizeof(s->phase_buf));
    memcpy(s->note_buf, p->note_on_buf, sizeof(s->note_buf));
    s->phase_switch = l->phase_switch;
    s->polyrotate = smn_polyrotate_enabled(p);
    for (unsigned v = 0; v < 6; ++v) {
        s->model[v] = smn_model_id_at(smn_model_wrap(p->wtsel_enc[v]));
        s->keymode[v] = p->key_sw[v];
        // In envelope modes note_on is a live gate, not a saved mute setting.
        s->note[v] = p->key_sw[v] == ksw_MUTE ? p->note_on[v] : 0;
        s->voct[v] = p->voct_switch_state[v];
        s->locks[0][v] = p->osc_param_lock[v];
        s->locks[1][v] = p->wt_pos_lock[v] | ((p->PADDING[92 + v] & 15u) << 1);
        s->locks[2][v] = p->wtsel_lock[v];
        s->mode[v] = l->mode[v];
        s->to_vca[v] = l->to_vca[v];
        s->locked[v] = l->locked[v];
        s->mode_buf[v] = l->mode_buf[v];
        s->to_vca_buf[v] = l->to_vca_buf[v];
        // Unlocked value follows CV continuously; it must not cause writes.
        s->locked_speed[v] = l->locked[v] ? l->divmult_id_global_locked[v] : 0;
        if (p == &params) {
            uint16_t steps[SMN_MOD_DESTINATIONS];
            smn_modulation_peek_steps(v, steps);
            memcpy(s->breathe[v], steps, sizeof(s->breathe[v]));
        }
        else {
            const uint8_t present = !memcmp(p->PADDING + 24, "BRH\2", 4);
            for (unsigned d = 0; d < 5; ++d) {
                const uint8_t *src = p->PADDING + 28 + 2 * (v * 5 + d);
                s->breathe[v][d] = present ? (uint16_t)(src[0] | (src[1] << 8)) : 0;
            }
        }
    }
    __asm__ volatile("" ::: "memory");
}

void smn_settings_restore(const SmnSettings *s, o_params *p, o_lfos *l)
{
    smn_parameter_locks_init(p);
    memcpy(p->wt_nav_enc, s->nav, sizeof(s->nav));
    for (unsigned d = 0; d < 3; ++d) smn_dispersion_set(p, d, s->dispersion[d]);
    memcpy(p->pan, s->pan, sizeof(s->pan));
    memcpy(l->divmult_id, s->speed, sizeof(s->speed));
    memcpy(l->phase_id, s->phase, sizeof(s->phase));
    memcpy(l->gain, s->gain, sizeof(s->gain));
    memcpy(l->divmult_id_buf, s->speed_buf, sizeof(s->speed_buf));
    memcpy(l->gain_buf, s->gain_buf, sizeof(s->gain_buf));
    memcpy(p->transpose_enc, s->transpose, sizeof(s->transpose));
    memcpy(p->spread_enc, s->spread, sizeof(s->spread));
    memcpy(p->finetune, s->fine, sizeof(s->fine));
    memcpy(p->oct, s->octave, sizeof(s->octave));
    memcpy(p->indiv_scale, s->scale, sizeof(s->scale));
    memcpy(p->indiv_scale_buf, s->scale_buf, sizeof(s->scale_buf));
    scale_set(p, p->indiv_scale[0], scale_root(p));
    p->PADDING[SMN_POLYROTATE_BYTE] = s->polyrotate == 1 && p->indiv_scale[0] != sclm_NONE;
    memcpy(l->shape, s->shape, sizeof(s->shape));
    memcpy(l->shape_buf, s->shape_buf, sizeof(s->shape_buf));
    memcpy(l->phase_id_buf, s->phase_buf, sizeof(s->phase_buf));
    memcpy(p->note_on_buf, s->note_buf, sizeof(s->note_buf));
    l->phase_switch = s->phase_switch;
    for (unsigned v = 0; v < 6; ++v) {
        if (l->shape[v] >= NUM_LFO_SHAPES) l->shape[v] = LFO_INIT_SHAPE;
        if (l->shape_buf[v] >= NUM_LFO_SHAPES) l->shape_buf[v] = LFO_INIT_SHAPE;
        p->wtsel_enc[v] = smn_model_index(s->model[v]);
        p->key_sw[v] = s->keymode[v];
        p->note_on[v] = s->note[v];
        p->voct_switch_state[v] = s->voct[v];
        p->osc_param_lock[v] = s->locks[0][v];
        p->wt_pos_lock[v] = s->locks[1][v] & 1u;
        smn_parameter_lock_set(p, v, s->locks[1][v] >> 1);
        p->wtsel_lock[v] = s->locks[2][v];
        l->mode[v] = s->mode[v];
        l->to_vca[v] = s->to_vca[v];
        l->locked[v] = s->locked[v];
        l->mode_buf[v] = s->mode_buf[v];
        l->to_vca_buf[v] = s->to_vca_buf[v];
        l->divmult_id_global_locked[v] = s->locked[v] ? s->locked_speed[v] : s->speed[GLO_CLK];
        l->phase[v] = calc_lfo_phase(s->phase[v]);
        for (unsigned d = 0; d < 5; ++d) {
            uint8_t *dst = p->PADDING + 28 + 2 * (v * 5 + d);
            dst[0] = (uint8_t)s->breathe[v][d];
            dst[1] = (uint8_t)(s->breathe[v][d] >> 8);
        }
    }
    smn_model_selection_capture(p);
    memcpy(p->PADDING + 24, "BRH\2", 4);
}


void smn_pan_capture(uint16_t steps[6], const o_params *p)
{
    for (unsigned v = 0; v < 6; ++v) {
        if (p == &params) {
            uint16_t all[SMN_MOD_DESTINATIONS];
            smn_modulation_peek_steps(v, all);
            steps[v] = all[SMN_MOD_PAN];
        } else steps[v] = !memcmp(p->PADDING + 100, "PAN\1", 4) ?
            p->PADDING[104 + 2*v] | ((uint16_t)p->PADDING[105 + 2*v] << 8) : 0;
    }
}
void smn_pan_restore(const uint16_t steps[6], o_params *p)
{
    memcpy(p->PADDING + 100, "PAN\1", 4);
    for (unsigned v = 0; v < 6; ++v) {
        const uint16_t value = steps[v] <= SMN_MOD_FULL_STEPS ? steps[v] : 0;
        p->PADDING[104 + 2*v] = (uint8_t)value;
        p->PADDING[105 + 2*v] = (uint8_t)(value >> 8);
    }
}

void smn_preset_encode(SmnPresetRecord *r, const o_params *p, const o_lfos *l)
{
    memset(r, 0, sizeof(*r));
    memcpy(r->signature, "SMN2", 4);
    r->version = 4;
    r->payload_size = sizeof(r->state) + sizeof(r->playback);
    smn_settings_capture(&r->state.settings, p, l);
    // Presets retain their envelope gates and cycle positions; resume does not.
    memcpy(r->playback.note, p->note_on, sizeof(r->playback.note));
    memcpy(r->playback.cycle, l->cycle_pos, sizeof(r->playback.cycle));
    memcpy(r->playback.divisions, l->div_cnt, sizeof(r->playback.divisions));
    smn_pan_capture(r->state.pan_breathe, p);
    r->state.extra = p->smn_extra;
    smn_patch_extra_sanitize(&r->state.extra);
    r->crc = smn_storage_crc((const uint8_t *)r, sizeof(*r), offsetof(SmnPresetRecord, crc));
}
static SmnPresetPlayback preset_playback(const SmnPresetRecord *r)
{
    SmnPresetPlayback result;
    memcpy(&result, (const uint8_t *)r + (r->version == 3 ? 736 : offsetof(SmnPresetRecord, playback)), sizeof(result));
    return result;
}
uint8_t smn_preset_valid(const SmnPresetRecord *r)
{
    // Unknown model IDs fall back in presets; resume retains its stricter rule.
    SmnSettings checked = r->state.settings;
    for (unsigned v = 0; v < 6; ++v)
        if (!smn_model_valid(checked.model[v])) checked.model[v] = smn_model_id_at(0);
    if (memcmp(r->signature, "SMN2", 4) || (r->version != 3 && r->version != 4) ||
        r->payload_size != (r->version == 3 ? 764 : sizeof(r->state) + sizeof(r->playback)) ||
        r->crc != smn_storage_crc((const uint8_t *)r, sizeof(*r), offsetof(SmnPresetRecord, crc)) ||
        !smn_settings_valid(&checked)) return 0;
    const SmnPresetPlayback playback = preset_playback(r);
    for (unsigned v = 0; v < 8; ++v)
        if (!in_range(playback.cycle[v], 0, 1)) return 0;
    for (unsigned v = 0; v < 6; ++v) if (playback.note[v] > 1) return 0;
    return 1;
}
uint8_t smn_preset_decode(const SmnPresetRecord *r, o_params *p, o_lfos *l)
{
    if (!smn_preset_valid(r)) return 0;
    // A numbered preset must not restore a historical measurement of the
    // external clock. Keep the current live measurement on an active recall.
    const float reference_period = p == &params ? l->period[REF_CLK] : 0;
    const uint8_t use_external_clock = p == &params ? l->use_ext_clock : 0;
    memset(p, 0, sizeof(*p)); memset(l, 0, sizeof(*l));
    init_param_object(p); init_lfo_object(l);
    if (p == &params && isfinite(reference_period) && reference_period > 0) {
        l->period[REF_CLK] = reference_period;
        l->use_ext_clock = use_external_clock;
    }
    smn_settings_restore(&r->state.settings, p, l);
    const uint16_t zero_pan[6] = {0};
    smn_pan_restore(r->version == 3 ? zero_pan : r->state.pan_breathe, p);
    p->smn_extra = r->state.extra;
    smn_patch_extra_sanitize(&p->smn_extra);
    const SmnPresetPlayback playback = preset_playback(r);
    memcpy(p->note_on, playback.note, sizeof(playback.note));
    memcpy(l->cycle_pos, playback.cycle, sizeof(playback.cycle));
    memcpy(l->div_cnt, playback.divisions, sizeof(playback.divisions));
    return 1;
}
