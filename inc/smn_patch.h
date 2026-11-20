// Versioned storage structures, separate from live audio and control state.
#pragma once
#include "params_update.h"
#include "params_lfo.h"
typedef struct {
    float nav[3][6], dispersion[3], pan[6];
    float speed[7], phase[6], gain[6];
    float speed_buf[6], gain_buf[6], locked_speed[6];
    int32_t transpose[6], spread[6];
    int16_t fine[6];
    uint16_t breathe[6][5];
    int8_t octave[6], scale[6], scale_buf[6];
    // LFO IDs bank * 33 + position, including duplicate curves across banks.
    uint16_t shape[6], shape_buf[6];
    int8_t phase_buf[6];
    uint8_t model[6], note[6], keymode[6], voct[6], locks[3][6];
    uint8_t mode[6], to_vca[6], locked[6], mode_buf[6], to_vca_buf[6], note_buf[6];
    uint8_t phase_switch;
    uint8_t polyrotate; // Former zero padding byte 503; sizes/offsets unchanged.
} SmnSettings;
// Shared by the public preset v4 and automatic-resume v6 formats.
typedef struct {
    SmnSettings settings;
    SmnPatchExtra extra;
    uint16_t pan_breathe[6]; // Six pan modulation amounts, after settings/chords.
} SmnPatchState;
// Preserve useful preset playback position, but never journal these live values.
typedef struct {
    float cycle[8];
    uint8_t note[6], divisions[6];
} SmnPresetPlayback;
typedef struct {
    uint8_t signature[4];
    uint16_t version, payload_size;
    uint32_t crc, reserved_header;
    SmnPatchState state;
    SmnPresetPlayback playback;
    uint8_t reserved[232];
} SmnPresetRecord;
#define SMN_PRESET_BYTES 1024u
#define SMN_LEGACY_PARAMS_BYTES 504u
void smn_settings_capture(SmnSettings *s, const o_params *p, const o_lfos *l);
void smn_settings_restore(const SmnSettings *s, o_params *p, o_lfos *l);
uint8_t smn_settings_valid(const SmnSettings *s);
uint32_t smn_storage_crc(const uint8_t *data, unsigned size, unsigned crc_offset);
void smn_preset_encode(SmnPresetRecord *r, const o_params *p, const o_lfos *l);
uint8_t smn_preset_valid(const SmnPresetRecord *r);
uint8_t smn_preset_decode(const SmnPresetRecord *r, o_params *p, o_lfos *l);

void smn_pan_capture(uint16_t steps[6], const o_params *p);
void smn_pan_restore(const uint16_t steps[6], o_params *p);

// Reset only the six memories; leave glide durations unchanged.
void smn_patch_chords_init(SmnPatchExtra *e);
