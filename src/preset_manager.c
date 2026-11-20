/*
 * presets.c - Saving and recalling presets
 *
 * Author: Dan Green (danngreen1@gmail.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * See http://creativecommons.org/licenses/MIT/ for more information.
 *
 * -----------------------------------------------------------------------------
 */

#include "preset_manager.h"
#include "quantz_scales.h"
#include "UI_conditioning.h"
#include "drivers/flashram_spidma.h"
#include "globals.h"
#include "gpio_pins.h"
#include "hardware_controls.h"
#include "math_util.h"
#include "params_lfo_period.h"
#include "preset_manager_undo.h"
#include "preset_manager_UI.h"
#include "timekeeper.h"
#include "startup_preset_storage.h"
#include "smn_model_selection.h"
#include "smn_resume.h"
#include "smn_patch.h"
#include <stddef.h>
#include <string.h>

extern o_params params;
extern o_lfos lfos;

o_preset_manager preset_mgr;

char	preset_signature_v1_0[4] = {'P', 'R', '9', '\0'};
char	preset_signature_v1_2[4] = {'P', 'R', 'A', '\0'};
char	preset_signature_v2_0[4] = {'P', 'R', 'B', '\0'};
char	preset_signature_legacy_smn[4] = {'P', 'R', 'C', '\0'};
// Recognize older signatures for display; decode supported SMN2 versions.
char preset_signature_smn1[4] = {'S', 'M', 'N', '1'};
char	preset_signature_vLatest[4] = {'S', 'M', 'N', '2'};

_Static_assert(offsetof(o_params, smn_extra) == 504 && sizeof(o_lfos) == 524,
               "Runtime LFO IDs widened; persistent record stays 1024 bytes");
_Static_assert(offsetof(o_params, PADDING) == 348 &&
               sizeof(((o_params *)0)->PADDING) == 153,
               "Preserve the reserved preset byte layout");

static uint8_t cached_preset[SMN_PRESET_BYTES];
static SmnPresetRecord io_record;
static uint8_t animation_enabled = 1;
static char read_data[4];

static inline void wait_for_flash_ready(void)
{
	while (get_flash_state() != sFLASH_NOTBUSY) {}
}

void init_preset_manager(void)
{
	char dummy;

	preset_mgr.hover_num = 0;
	preset_mgr.mode = PM_INACTIVE;
	preset_mgr.last_action = PM_INACTIVE;
	preset_mgr.animation_ctr = 0;

	uint16_t i;
	for (i = 0; i < MAX_PRESETS; i++) {
        preset_mgr.is_swn[i] = 0;
		if (check_preset_filled(i, &dummy))
			preset_mgr.filled[i] = 1;
		else
			preset_mgr.filled[i] = 0;
	}

	init_startup_preset_storage();
	// Keep the SWN startup-slot storage for rollback compatibility, but SMN
	// boots from its separate checked journal in main(), never a numbered slot.
	stash_active_into_undo_buffer();

}

void exit_preset_manager(void)
{
	preset_mgr.mode = PM_INACTIVE;
	preset_mgr.activity_tmr = 0;
	preset_mgr.animation_ctr = 0;
}

void store_preset_from_active(uint32_t preset_num)
{
	preset_mgr.hover_num = preset_num;
	preset_start_save_animation();
	store_preset(preset_num, &params, &lfos);
	set_startup_preset(preset_num);
}

void recall_preset_into_active(uint32_t preset_num)
{
	preset_mgr.hover_num = preset_num;
	if (animation_enabled)
		preset_start_load_animation();
	stash_active_into_undo_buffer();
	recall_preset(preset_num, &params, &lfos);
	recalc_active_params();
}

void store_preset(uint32_t preset_num, o_params *t_params, o_lfos *t_lfos)
{
	uint32_t addr = get_preset_addr(preset_num);
	uint32_t other_preset_addr;
	char dummy;

	pause_timer_IRQ(OSC_TIM_number);
	pause_timer_IRQ(PWM_OUTS_TIM_number);
	smn_parameter_locks_restore(t_params);
	smn_dispersion_restore(t_params);
	smn_model_selection_capture(t_params);
	// Undo carries its own modulation counters, separate from the live values.
	if (t_params == &params) smn_breathe_capture(t_params);
    // Encode and validate before erasing either preset in the shared sector.
    smn_preset_encode(&io_record, t_params, t_lfos);
    if (!smn_preset_valid(&io_record)) {
        resume_timer_IRQ(OSC_TIM_number);
        resume_timer_IRQ(PWM_OUTS_TIM_number);
        return;
    }
	wait_for_flash_ready();

	//store other half of sector in a temp variable
	other_preset_addr = get_preset_addr((preset_num & 1) ? preset_num - 1 : preset_num + 1);
	sFLASH_read_buffer(cached_preset, other_preset_addr, get_preset_size());

	sFLASH_erase_sector(addr);

    sFLASH_write_buffer((uint8_t *)&io_record, addr, SMN_PRESET_BYTES);

	sFLASH_write_buffer(cached_preset, other_preset_addr, get_preset_size());

	//Verify sector was written (could use a checksum to be more rigorous)
	preset_mgr.filled[preset_num] = check_preset_filled(preset_num, &dummy);

	resume_timer_IRQ(OSC_TIM_number);
	resume_timer_IRQ(PWM_OUTS_TIM_number);
}

void recall_preset(uint32_t preset_num, o_params *t_params, o_lfos *t_lfos)
{
	uint32_t addr = get_preset_addr(preset_num);
	uint8_t preset_is_filled;
	char version;

	pause_timer_IRQ(OSC_TIM_number);
	pause_timer_IRQ(PWM_OUTS_TIM_number);

	while (get_flash_state() != sFLASH_NOTBUSY) {}

	//Manually checking is not necessary, but we want to make sure this sector is readable, and we already have control of FLASH
	preset_is_filled = check_preset_filled(preset_num, &version);
	if (preset_num < MAX_PRESETS && preset_mgr.filled[preset_num] && preset_is_filled && !preset_mgr.is_swn[preset_num]) {
        if (version == '2') {
            sFLASH_read_buffer((uint8_t *)&io_record, addr, SMN_PRESET_BYTES);
            if (!smn_preset_decode(&io_record, t_params, t_lfos)) {
                init_param_object(t_params); init_lfo_object(t_lfos);
            }
        } else {
            // Unsupported development format: defaults in RAM, no write to flash.
            init_param_object(t_params); init_lfo_object(t_lfos);
        }

		smn_model_selection_restore(t_params);
		smn_parameter_locks_restore(t_params);
		smn_dispersion_restore(t_params);
	}
	else {
		//Loading an unfilled preset re-initializes params
		init_param_object(t_params);
		init_lfo_object(t_lfos);
	}

    // Also guard runtime copies before their table-indexed readers.
    for (unsigned v = 0; v < NUM_CHANNELS; ++v) {
        if (t_lfos->shape[v] >= NUM_LFO_SHAPES) t_lfos->shape[v] = LFO_INIT_SHAPE;
        if (t_lfos->shape_buf[v] >= NUM_LFO_SHAPES) t_lfos->shape_buf[v] = LFO_INIT_SHAPE;
    }

	if (t_params == &params) smn_breathe_restore(t_params, t_lfos);

	resume_timer_IRQ(OSC_TIM_number);
	resume_timer_IRQ(PWM_OUTS_TIM_number);
}


void clear_preset(uint32_t preset_num)
{
	//Write over the preset
	uint32_t addr = get_preset_addr(preset_num);
	uint32_t other_preset_addr;

	wait_for_flash_ready();

	//store other half of sector in a temp variable
	other_preset_addr = get_preset_addr((preset_num & 1) ? preset_num - 1 : preset_num + 1);
	sFLASH_read_buffer(cached_preset, other_preset_addr, get_preset_size());

	sFLASH_erase_sector(addr);

	sFLASH_write_buffer(cached_preset, other_preset_addr, get_preset_size());

	preset_mgr.filled[preset_num] = 0;
    preset_mgr.is_swn[preset_num] = 0;

}

void reset_active_params(void)
{
    // Same timer protection and reconstruction as a preset recall, without I/O.
    pause_timer_IRQ(OSC_TIM_number);
    pause_timer_IRQ(PWM_OUTS_TIM_number);
    // Reset only the requested controls. Scale/root, bank, Fine, pan, voice
    // locks and physical switch state stay in their existing objects.
    params.PADDING[SMN_POLYROTATE_BYTE] = 0;
    smn_parameter_locks_init(&params); // Global sound reset also clears targeted locks.
    smn_dispersion_init(&params);
    smn_breathe_init(&params);
    uint8_t lfo_locked[NUM_CHANNELS];
    memcpy(lfo_locked, lfos.locked, sizeof(lfo_locked));
    init_lfo_object(&lfos);
    memcpy(lfos.locked, lfo_locked, sizeof(lfo_locked));
    for (uint8_t v = 0; v < NUM_CHANNELS; ++v) {
        params.wtsel_enc[v] = 0;
        for (uint8_t d = 0; d < 3; ++d) params.wt_nav_enc[d][v] = 0;
        params.transpose_enc[v] = 0;
        params.spread_enc[v] = 0;
        params.oct[v] = INIT_OCT;
        params.smn_extra.glide_ms[v] = 0;
        lfos.fine_phase_buf[v] = 0;
        const enum MuteNoteKeyStates mode = params.key_sw[v];
        if (mode != ksw_MUTE) {
            // Reuse the normal mode entry: reset the cached Mute LFO too,
            // then restore the envelope speed/routing required by this mode.
            const uint8_t mute_note = params.note_on_buf[v];
            params.key_sw[v] = ksw_MUTE;
            apply_keymode(v, mode);
            params.note_on_buf[v] = mute_note;
        } else {
            cache_uncache_keys_params_and_lfos(v, CACHE);
        }
    }
    smn_model_selection_capture(&params);
    recalc_active_params();
    resume_timer_IRQ(OSC_TIM_number);
    resume_timer_IRQ(PWM_OUTS_TIM_number);
}

void recalc_active_params(void)
{
	scale_restore(&params);
	init_calc_params();
	smn_model_selection_restore(&params);
	smn_parameter_locks_restore(&params);
	smn_dispersion_restore(&params);
	smn_breathe_restore(&params, &lfos);
	update_wtsel();
	update_all_smn_controls();
    // Stored settings omit derived periods; rebuild locked voices as well.
    for (uint8_t i = 0; i < NUM_CHANNELS + 2; ++i) flag_lfo_recalc(i);
	for (int i=0; i<NUM_CHANNELS; i++)
		compute_tuning(i);
	smn_resume_request();
}

//Writes over preset version in all sectors
//Much faster than erasing sectors
void clear_all_presets(void)
{
	uint8_t preset_num;
	uint32_t addr;
	uint8_t sz;

	wait_for_flash_ready();

	for (preset_num = 0; preset_num < MAX_PRESETS; preset_num++) {
		addr = get_preset_addr(preset_num);
		sz = 4;
		char version;
		if (check_preset_filled(preset_num, &version))
		{
			sz = 4;
			read_data[0] = 0x00;
			read_data[1] = 0x00;
			read_data[2] = 0x00;
			read_data[3] = 0x00;
			sFLASH_write_buffer((uint8_t *)read_data, addr, sz);
		}

		preset_mgr.filled[preset_num] = 0;
		preset_mgr.is_swn[preset_num] = 0;
	}

}

uint8_t check_preset_filled(uint32_t preset_num, char *version)
{
	uint32_t addr = get_preset_addr(preset_num);
	uint32_t sz;

	sz = 4;
	wait_for_flash_ready();

	sFLASH_read_buffer((uint8_t *)read_data, addr, sz);

    if (preset_num < MAX_PRESETS) preset_mgr.is_swn[preset_num] = 0;
	if (!memcmp(read_data, preset_signature_vLatest, sizeof(read_data)) ||
        !memcmp(read_data, preset_signature_smn1, sizeof(read_data)))
	{
		*version = read_data[3];
		return 1;
	}
	if (   read_data[0] == 'P'
		&& read_data[1] == 'R'
		&& (read_data[2] == preset_signature_legacy_smn[2] \
			|| read_data[2] == preset_signature_v2_0[2] \
			|| read_data[2] == preset_signature_v1_0[2] \
			|| read_data[2] == preset_signature_v1_2[2])
		&& read_data[3] == '\0' )
	{
        if (preset_num < MAX_PRESETS)
            preset_mgr.is_swn[preset_num] = read_data[2] != preset_signature_legacy_smn[2];
		*version = read_data[2];
		return 1;
	}
	else
		return 0;
}

uint32_t get_preset_size(void)
{
	return SMN_PRESET_BYTES;
}

uint32_t get_preset_addr(uint32_t preset_num)
{
	uint32_t preset_sector_offset = 0;

	if (preset_num >= MAX_PRESETS)
		preset_num = (MAX_PRESETS - 1);

	uint32_t sector_num = PRESET_SECTOR_START + (preset_num >> 1);
	uint32_t sector_start = sFLASH_get_sector_addr(sector_num);

	//odd numbered presets start in middle of a sector
	if (preset_num & 1) preset_sector_offset = sFLASH_get_sector_size(sector_num) / 2;

	return sector_start +  preset_sector_offset;
}
