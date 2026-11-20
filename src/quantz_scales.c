/*
 * quantz_scales.c
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

#include "quantz_scales.h"

extern o_params params;

static const uint8_t scale_order[NUM_QTZ_SCALES] = {
    sclm_NONE,
    sclm_OCTAVES, sclm_ROOT_FIFTH, sclm_ROOT_FOURTH,
    sclm_MAJOR_TRIAD, sclm_MINOR_TRIAD, sclm_SUS2_TRIAD,
    sclm_SEMITONES, sclm_MAJOR_PENTATONIC, sclm_MINOR_PENTATONIC,
    sclm_MAJOR, sclm_LYDIAN, sclm_MIXOLYDIAN,
    sclm_AEOLIAN, sclm_DORIAN, sclm_PHRYGIAN,
    sclm_MINOR, sclm_MELODIC_MINOR, sclm_HARMONIC_MAJOR,
    sclm_BLUES, sclm_WHOLE_TONE, sclm_DIM_HALF_WHOLE,
    sclm_DIM_WHOLE_HALF, sclm_DOUBLE_HARMONIC_MAJOR, sclm_QUARTERTONES
};

// Pitch classes in semitones above the root, not a frequency calculation.
#define N(n) (1u << (n))
static const uint16_t scale_masks[NUM_QTZ_SCALES] = {
    0,
    N(0)|N(2)|N(4)|N(5)|N(7)|N(9)|N(11), // Ionian
    N(0)|N(2)|N(3)|N(5)|N(7)|N(8)|N(11), // Harmonic minor (old MINOR)
    0xfff, 0xfff, // Chromatic / quarter tones
    N(0)|N(2)|N(4)|N(6)|N(7)|N(9)|N(11),
    N(0)|N(2)|N(4)|N(5)|N(7)|N(9)|N(10),
    N(0)|N(2)|N(3)|N(5)|N(7)|N(8)|N(10),
    N(0)|N(2)|N(3)|N(5)|N(7)|N(9)|N(10),
    N(0)|N(1)|N(3)|N(5)|N(7)|N(8)|N(10),
    N(0)|N(2)|N(3)|N(5)|N(7)|N(9)|N(11),
    N(0)|N(2)|N(4)|N(5)|N(7)|N(8)|N(11),
    N(0)|N(2)|N(4)|N(7)|N(9),
    N(0)|N(3)|N(5)|N(7)|N(10),
    N(0)|N(3)|N(5)|N(6)|N(7)|N(10),
    N(0)|N(2)|N(4)|N(6)|N(8)|N(10),
    N(0)|N(1)|N(3)|N(4)|N(6)|N(7)|N(9)|N(10),
    N(0)|N(2)|N(3)|N(5)|N(6)|N(8)|N(9)|N(11),
    N(0), N(0)|N(7), N(0)|N(5),
    N(0)|N(4)|N(7), N(0)|N(3)|N(7), N(0)|N(2)|N(7),
    N(0)|N(1)|N(4)|N(5)|N(7)|N(8)|N(11)
};
#undef N

// Historical SWN semitones unchanged; odd entries add their exact 50-cent
// midpoint. Constants only: no pow/log or note/frequency conversion at runtime.
static const float quarter_frequencies[24] = {
    16.351601562500f, 16.830740060984f,
    17.323906250000f, 17.831535450527f,
    18.354023437500f, 18.891837375626f,
    19.445429687500f, 20.015224269838f,
    20.601718750000f, 21.205395188075f,
    21.826757812500f, 22.466330635082f,
    23.124648437500f, 23.802252358313f,
    24.499726562500f, 25.217623347935f,
    25.956562500000f, 26.717147836827f,
    27.500000000000f, 28.305811507696f,
    29.135234375000f, 29.988961907320f,
    30.867695312500f, 31.772187825186f,
};

typedef struct {
    float frequency[24], midpoint[24];
    uint8_t note[24], count;
} ScaleTable;

// Build the inactive table in the main loop, publish one aligned pointer.
// The parameter ISR sees a complete old or new table, never a partial update.
volatile uint8_t scale_quantization_enabled;
volatile uint32_t smn_scale_generation;
static ScaleTable tables[2];
static const ScaleTable * volatile active = &tables[0];

uint8_t scale_position(uint8_t scale)
{
    for (uint8_t i = 0; i < NUM_QTZ_SCALES; ++i)
        if (scale_order[i] == scale) return i;
    return 0;
}

uint8_t scale_at_position(uint8_t position)
{
    return position < NUM_QTZ_SCALES ? scale_order[position] : sclm_NONE;
}

static uint8_t tagged(const int8_t config[6])
{
    return config[0] == 'S' && config[1] == 'C' && config[2] == 1 &&
        config[3] >= 0 && config[3] < 12 && config[4] == 0 && config[5] == 0;
}

uint8_t scale_root(const o_params *p)
{
    return tagged(p->indiv_scale_buf) ? (uint8_t)p->indiv_scale_buf[3] : 0;
}

uint8_t scale_config_valid(const int8_t scales[6], const int8_t config[6])
{
    if (tagged(config)) {
        // Unknown IDs remain loadable; scale_restore() maps them to NONE.
        for (uint8_t i = 1; i < 6; ++i)
            if (scales[i] != scales[0]) return 0;
    } else {
        // Old records only used IDs 0..3, including their per-channel cache.
        for (uint8_t i = 0; i < 6; ++i)
            if (scales[i] < 0 || scales[i] > sclm_SEMITONES ||
                config[i] < 0 || config[i] > sclm_SEMITONES) return 0;
    }
    return 1;
}

void scale_set(o_params *p, uint8_t scale, uint8_t root)
{
    if (scale >= NUM_QTZ_SCALES) scale = sclm_NONE;
    for (uint8_t i = 0; i < NUM_CHANNELS; ++i) p->indiv_scale[i] = scale;
    // Reuse the six obsolete per-channel scale-cache bytes. Preset and resume
    // layouts stay fixed. The tag distinguishes old caches from a saved root.
    p->indiv_scale_buf[0] = 'S'; p->indiv_scale_buf[1] = 'C';
    p->indiv_scale_buf[2] = 1; p->indiv_scale_buf[3] = root % 12;
    p->indiv_scale_buf[4] = p->indiv_scale_buf[5] = 0;
}

void scale_restore(o_params *p)
{
    scale_set(p, p->indiv_scale[0], scale_root(p));
    if (p != &params) return;
    ScaleTable *t = active == &tables[0] ? &tables[1] : &tables[0];
    t->count = 0;
    const uint8_t scale = p->indiv_scale[0], root = scale_root(p);
    const uint16_t mask = scale_masks[scale];
    for (uint8_t step = 0; step < 24; ++step) {
        const uint8_t relative = (step / 2 + 12 - root) % 12;
        if ((mask & (1u << relative)) &&
            (!(step & 1) || scale == sclm_QUARTERTONES)) {
            const uint8_t n = t->count++;
            t->frequency[n] = quarter_frequencies[step];
            t->note[n] = step;
        }
    }
    for (uint8_t i = 0; i < t->count; ++i) {
        const float next = i + 1 < t->count ? t->frequency[i+1] : t->frequency[0] * 2.f;
        t->midpoint[i] = (t->frequency[i] + next) * .5f;
    }
    __asm__ volatile("" ::: "memory");
    active = t;
    __asm__ volatile("" ::: "memory");
    scale_quantization_enabled = t->count != 0;
    ++smn_scale_generation;
    if (!t->count) p->PADDING[SMN_POLYROTATE_BYTE] = 0;
}

void init_quantz_scales(void) { scale_restore(&params); }

float quantize_to_scale(float frequency, uint8_t *note, int8_t *octave)
{
    const ScaleTable *t = active;
    *note = UINT8_MAX; // Also defined for bypass and out-of-range inputs.
    *octave = 0;
    if (!t->count || !(frequency > 0.f)) return frequency;

    int8_t oct = 0;
    float normalized = frequency;
    const float high = t->midpoint[t->count - 1];
    while (normalized >= high && oct < MAX_OCT) { normalized *= .5f; ++oct; }
    while (normalized < high * .5f && oct > -MAX_OCT) { normalized *= 2.f; --oct; }
    if (oct == MAX_OCT || oct == -MAX_OCT) return frequency;

    // At most five threshold comparisons for any scale.
    uint8_t low = 0, end = t->count - 1;
    while (low < end) {
        const uint8_t mid = (low + end) / 2;
        if (normalized < t->midpoint[mid]) end = mid;
        else low = mid + 1;
    }
    // Always return a note, including near a midpoint. Holding the old pitch
    // here would also hold its octave: octave moves keep the normalized gap.
    *note = t->note[low];
    *octave = oct;
    return oct < 0 ? t->frequency[low] / (float)(1u << -oct)
                   : t->frequency[low] * (float)(1u << oct);
}
