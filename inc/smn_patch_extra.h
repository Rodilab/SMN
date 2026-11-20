// SMN-only settings, independent of the legacy SWN byte layout.
#pragma once
#include <stdint.h>
#define SMN_CHORD_MEMORIES 6
#define SMN_PATCH_VOICES 6
#define SMN_PATCH_EXTRA_V1_BYTES 204u
// Pitch is an integer semitone offset with the saved octave removed.
// Glide durations in milliseconds; transition states are never persisted.
typedef struct {
    int32_t chord_pitch[SMN_CHORD_MEMORIES][SMN_PATCH_VOICES];
    int8_t chord_octave[SMN_CHORD_MEMORIES][SMN_PATCH_VOICES];
    uint32_t glide_ms[SMN_PATCH_VOICES];
    // Global Scale/root at capture; NONE means no saved scale.
    uint8_t chord_scale[SMN_CHORD_MEMORIES];
    uint8_t chord_root[SMN_CHORD_MEMORIES];
} SmnPatchExtra;
void smn_patch_extra_init(SmnPatchExtra *extra);
void smn_patch_extra_sanitize(SmnPatchExtra *extra);
