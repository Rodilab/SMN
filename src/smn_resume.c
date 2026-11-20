// Automatic state recall uses its own journal, separate from numbered presets.
// The main loop services it without pausing timers or masking interrupts.
// It uses fixed buffers and does not run in the audio callback.
#include "smn_resume.h"
#include "smn_patch.h"
#include "drivers/smn_flash_async.h"
#include "drivers/flashram_spidma.h"
#include "external_flash_layout.h"
#include "params_update.h"
#include "params_lfo.h"
#include "params_lfo_period.h"
#include "preset_manager.h"
#include "preset_manager_undo.h"
#include "smn_models.h"
#include "smn_model_selection.h"
#include "quantz_scales.h"
#include "ui_modes.h"
#include "system_settings.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

extern o_params params;
extern o_lfos lfos;
extern enum UI_Modes ui_mode;
extern o_systemSettings system_settings;

typedef SmnPatchState ResumeState;

enum { DATA_SIZE = 768, RECORD_SIZE = 1024, SLOT_COUNT = 4096 / RECORD_SIZE };
#define RESUME_MAGIC 0x52534d53u // "SMSR", distinct from every preset signature.
// Public v6 schema: version in the upper half, state size in the lower half.
#define RESUME_SCHEMA (0x00060000u | (uint32_t)sizeof(ResumeState))
#define RESUME_COMMIT 0x434d5352u

typedef union {
    struct {
        uint32_t magic, schema, sequence, crc;
        ResumeState state;
        // V6 brightness follows the 732-byte state.
        float brightness;
    } value;
    uint8_t bytes[DATA_SIZE];
} ResumeRecord;
_Static_assert(sizeof(ResumeState) == 732, "Change the journal version if its layout changes");
_Static_assert(sizeof(ResumeRecord) == DATA_SIZE, "Resume settings fit three pages");
_Static_assert(offsetof(ResumeRecord, value.brightness) == 748, "Resume v6 brightness offset");
_Static_assert(NUM_CHANNELS == 6 && SMN_MOD_DESTINATIONS == 6, "Resume voice/destination counts");
_Static_assert(SMN_RESUME_SECTOR_A == 0 && SMN_RESUME_SECTOR_B == 1,
               "Resume journal must stay outside SWN data");

static ResumeState observed, candidate;
static ResumeRecord writing;
static uint8_t scratch[DATA_SIZE];
static uint32_t commit[4];
static uint32_t sequence, last_scan, last_io, last_change;
static int8_t latest_bank;
static uint8_t next_slot[2], target_bank, dirty, enabled;
static volatile uint8_t forced; // Also set by the brightness gesture's ISR.
static uint16_t offset;
static uint32_t write_address;
enum Worker { IDLE, ERASED, BLANK, PROGRAMMED, VERIFIED, COMMITTED, CONFIRMED, FAILED };
static enum Worker worker;

static uint32_t bank_address(uint8_t bank)
{
    return sFLASH_get_sector_addr(bank ? SMN_RESUME_SECTOR_B : SMN_RESUME_SECTOR_A);
}

static uint32_t checksum(const uint8_t *data)
{
    return smn_storage_crc(data, DATA_SIZE, 12);
}
static uint8_t in_range(float x, float low, float high)
{
    return isfinite(x) && x >= low && x <= high;
}
static uint8_t valid_settings(const ResumeState *s)
{
    return smn_settings_valid(&s->settings);
}
static void capture(ResumeState *s)
{
    __asm__ volatile("" ::: "memory");
    smn_settings_capture(&s->settings, &params, &lfos);
    smn_pan_capture(s->pan_breathe, &params);
    s->extra = params.smn_extra;
    smn_patch_extra_sanitize(&s->extra);
    __asm__ volatile("" ::: "memory");
}
static void restore(const ResumeState *s)
{
    smn_settings_restore(&s->settings, &params, &lfos);
    smn_pan_restore(s->pan_breathe, &params);
    params.smn_extra = s->extra;
    smn_patch_extra_sanitize(&params.smn_extra);
    recalc_active_params();
}
static float record_brightness(const ResumeRecord *r)
{
    float result;
    memcpy(&result, r->bytes + (r->value.schema == 0x000502D0u ? 736 : 748), 4);
    return result;
}
static uint8_t valid_record(const ResumeRecord *r, const uint32_t *c)
{
    return r->value.magic == RESUME_MAGIC && (r->value.schema == RESUME_SCHEMA || r->value.schema == 0x000502D0u) &&
        c[0] == RESUME_COMMIT && c[1] == r->value.sequence &&
        c[2] == r->value.crc && c[3] == ~RESUME_COMMIT &&
        checksum(r->bytes) == r->value.crc && valid_settings(&r->value.state) &&
        (record_brightness(r) == 0 || in_range(record_brightness(r), 1.f / 1000.f, 2.f));
}

void smn_resume_init(void)
{
    float brightness = 0;
    enabled = dirty = forced = 0;
    worker = IDLE;
    latest_bank = -1;
    sequence = 0;
    // Always re-erase the other bank before its first write after boot: an
    // interrupted erase/program can leave unstable cells even if they read FF.
    next_slot[0] = next_slot[1] = SLOT_COUNT;
    // Read v5 and public v6 records without writing or erasing at boot.
    for (unsigned b = 0; b < 2; ++b) for (unsigned slot = 0; slot < SLOT_COUNT; ++slot) {
        const uint32_t addr = bank_address(b) + slot * RECORD_SIZE;
        sFLASH_read_buffer(writing.bytes, addr, DATA_SIZE);
        sFLASH_read_buffer((uint8_t *)commit, addr + DATA_SIZE, sizeof(commit));
        if (!valid_record(&writing, commit)) continue;
        const uint32_t found_sequence = writing.value.sequence;
        if (latest_bank >= 0 && (int32_t)(found_sequence - sequence) <= 0) continue;
        observed = writing.value.state;
        if (writing.value.schema == 0x000502D0u) memset(observed.pan_breathe, 0, sizeof(observed.pan_breathe));
        brightness = record_brightness(&writing);
        sequence = found_sequence;
        latest_bank = b;
    }
    // With no extension, keep the original SWN internal setting loaded at boot.
    if (brightness != 0) system_settings.global_brightness = brightness;
    // Boot only, before audio and the oscillator/LFO update timers are started.
    memset(&params, 0, sizeof(params));
    memset(&lfos, 0, sizeof(lfos));
    init_param_object(&params);
    init_lfo_object(&lfos);
    // Keep init_param_object() H/T/M defaults: raw zero gives midpoint 0.5.
    // First catalog entry, no dispersion or internal modulation without a valid journal.
    for (unsigned v = 0; v < 6; ++v) {
        params.wtsel_enc[v] = 0;
    }
    smn_model_selection_capture(&params);
    if (latest_bank >= 0) restore(&observed);
    else recalc_active_params();
    stash_active_into_undo_buffer();
    capture(&observed);
    last_scan = last_io = last_change = HAL_GetTick();
    forced = 0;
    enabled = 1;
}

void smn_resume_request(void)
{
    if (enabled) forced = 1;
}

uint8_t smn_resume_flash_available(void)
{
    return (worker == IDLE || worker == FAILED) && smn_flash_idle();
}

static uint8_t blank(const uint8_t *p, unsigned size)
{
    for (unsigned i = 0; i < size; ++i) if (p[i] != 255) return 0;
    return 1;
}

static void fail(void) { worker = FAILED; }

static void program_page(void)
{
    if (!smn_flash_program(write_address + offset, writing.bytes + offset, 256)) fail();
    else worker = PROGRAMMED;
}

static void start_write(void)
{
    // Clear before capturing. A brightness release during or after capture
    // must leave its request pending for the next delayed write.
    dirty = forced = 0;
    __asm__ volatile("" ::: "memory");
    memset(&writing, 0, sizeof(writing));
    writing.value.magic = RESUME_MAGIC;
    writing.value.schema = RESUME_SCHEMA;
    writing.value.sequence = sequence + 1;
    writing.value.state = observed;
    writing.value.brightness = system_settings.global_brightness;
    writing.value.crc = checksum(writing.bytes);
    commit[0] = RESUME_COMMIT;
    commit[1] = writing.value.sequence;
    commit[2] = writing.value.crc;
    commit[3] = ~RESUME_COMMIT;
    target_bank = latest_bank == 0 ? 1 : 0;
    offset = 0;
    if (next_slot[target_bank] >= SLOT_COUNT) {
        if (!smn_flash_erase(bank_address(target_bank))) fail();
        else worker = ERASED;
    } else {
        write_address = bank_address(target_bank) + next_slot[target_bank]++ * RECORD_SIZE;
        program_page();
    }
}

static void step_write(uint32_t ticks)
{
    const enum SmnFlashResult result = smn_flash_poll(ticks);
    if (result == SMN_FLASH_BUSY) return;
    if (result == SMN_FLASH_FAILED) { fail(); return; }
    switch (worker) {
    case ERASED:
        offset = 0;
        if (!smn_flash_read(bank_address(target_bank), scratch, 256)) fail();
        else worker = BLANK;
        break;
    case BLANK:
        if (!blank(scratch, 256)) { fail(); break; }
        offset += 256;
        if (offset < 4096) {
            if (!smn_flash_read(bank_address(target_bank) + offset, scratch, 256)) fail();
        } else {
            next_slot[target_bank] = 1;
            write_address = bank_address(target_bank);
            offset = 0;
            program_page();
        }
        break;
    case PROGRAMMED:
        offset += 256;
        if (offset < DATA_SIZE) program_page();
        else if (!smn_flash_read(write_address, scratch, DATA_SIZE)) fail();
        else worker = VERIFIED;
        break;
    case VERIFIED:
        if (memcmp(scratch, writing.bytes, DATA_SIZE)) { fail(); break; }
        // Commit has its own previously untouched page/ECC region.
        if (!smn_flash_program(write_address + DATA_SIZE, (uint8_t *)commit, sizeof(commit))) fail();
        else worker = COMMITTED;
        break;
    case COMMITTED:
        if (!smn_flash_read(write_address + DATA_SIZE, scratch, sizeof(commit))) fail();
        else worker = CONFIRMED;
        break;
    case CONFIRMED:
        if (memcmp(scratch, commit, sizeof(commit))) { fail(); break; }
        latest_bank = target_bank;
        sequence = writing.value.sequence;
        worker = IDLE;
        break;
    default: break;
    }
}

void smn_resume_poll(uint32_t ticks)
{
    if (!enabled || worker == FAILED || (uint32_t)(ticks - last_io) < TICKS_PER_MS) return;
    last_io = ticks;
    // 50 Hz for settings, at most 1 kHz while SPI work is in progress. These
    // periods use the project's 8 ticks/ms, not an assumed HAL 1 ms tick.
    if ((uint32_t)(ticks - last_scan) >= 20u * TICKS_PER_MS) {
        last_scan = ticks;
        if (ui_mode == PLAY || ui_mode == SELECT_PARAMS) {
            capture(&candidate);
            // Keep the second copy on the stack; DMA buffers belong to the worker.
            ResumeState second;
            capture(&second);
            if (!memcmp(&candidate, &second, sizeof(candidate)) && valid_settings(&candidate)) {
                if (forced || memcmp(&candidate, &observed, sizeof(candidate))) {
                    observed = candidate;
                    last_change = ticks;
                    dirty = 1;
                    forced = 0;
                }
            }
        } else last_change = ticks; // Do not persist calibration's temporary state.
    }
    if (worker != IDLE) step_write(ticks);
    else if (dirty && (ui_mode == PLAY || ui_mode == SELECT_PARAMS) &&
               (uint32_t)(ticks - last_change) >= 1000u * TICKS_PER_MS &&
               smn_flash_idle()) start_write();
}
