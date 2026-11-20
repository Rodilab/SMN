# SMN — Spherical Macro Navigator

## 1. Introduction

SMN is an unofficial alternative firmware for the **4ms Spherical Wavetable Navigator (SWN)**. It combines the SWN's six-voice interface with synthesis engines adapted from Mutable Instruments Plaits. Each voice has its own model, pitch, timbre parameters and modulation settings.

The original sphere-based audio engine is replaced by **15 Plaits engines and 31 selectable variants**, including two audio-input resonators. SMN also adds internal modulation, chord memories, quantization glide, rotating pitch allocation and automatic state recall after power-off.

This README describes the stable **beta 0.1** sources. It focuses on changes from the original SWN and assumes familiarity with that module. The target hardware is the SWN with an **STM32F765**; audio remains at **44.1 kHz**. SMN is neither firmware for the Plaits module nor an official release from 4ms or Mutable Instruments.

## 2. Downloads, manual and project news

**For normal use, get the firmware and illustrated PDF manual from [Payhip](https://payhip.com/b/SXuoW).** This is the main distribution page for the latest stable release.

The download is **free** and delivered through Payhip's secure download service. The installer is an audio WAV file played into the module; no executable installer runs on your computer. The email address requested for the download is used **only to notify you of firmware updates**, and for nothing else.

This GitHub repository is intended for developers and technically experienced users who want the source code, implementation details or repository builds. This branch prepares the sources of the publicly distributed beta 0.1; its firmware behaviour is preserved.

For demonstrations and project updates, see
[YouTube](https://youtu.be/mxIhumODbAY) and
[Instagram](https://www.instagram.com/sol.modular/).

## 3. Installation and returning to SWN

Installation uses **the same audio-update procedure as the original SWN**, through **Waveform In** and the existing bootloader. The WAV and current PDF manual on [Payhip](https://payhip.com/b/SXuoW) provide the release package and detailed installation instructions. No hardware modification, factory reset or recalibration is required.

The repository also provides the beta 0.1 application in [firmware/](firmware/): [main.wav](firmware/main.wav) for audio installation and [main.bin](firmware/main.bin) for SWD programming. These files keep their normal names; the Payhip package uses release-specific filenames. Check which revision an artifact represents: rebuilding the application does not regenerate its WAV automatically.

**Returning safely to the original SWN firmware is supported** through the same audio-update mechanism, using an appropriate original application WAV. SMN preserves the bootloader, calibrations, system settings and stored spheres. An audio round trip from SMN to SWN and back has been reported successful on the development module, with the original spheres available again under SWN.

Stored data and preset compatibility are separate matters. Original SWN presets remain stored unless you overwrite their slots, but current SMN loads them as defaults. SWN cannot read presets saved in SMN's format. Keep original presets in slots you do not reuse if you intend to return to SWN.

Advanced users can also install the application binary with SWD. See the programming constraints in section 14.

The remaining chapters cover:

- [Interface changes](#4-interface-changes-at-a-glance) and [synthesis models](#5-synthesis-models).
- [Timbre, dispersion and CV](#6-timbre-dispersion-and-cv-reassignment) and [Internal modulation](#7-internal-modulation).
- [Pitch, scales, chords and Polyrotate](#8-pitch-scales-chord-memories-and-polyrotate).
- [LFOs and gates](#9-lfos-envelopes-and-gate-routing), [levels and stereo placement](#10-levels-and-stereo-placement).
- [Locks and storage](#11-locks-presets-and-automatic-resume), [resets](#12-reset-shortcuts-and-defaults) and [display changes](#13-display-and-smaller-behaviour-changes).
- [Building and code structure](#14-building-and-navigating-the-code), [current limits](#15-current-limits-and-reporting-problems) and [licences](#16-credits-and-licences).

## 4. Interface changes at a glance

**Browse** always means the panel’s Browse button/encoder. Its internal modulation function is described in section 7. **Scale** refers to the pressed Octave encoder. **Preset** means the Load Preset encoder. H/T/M abbreviates HARMONICS, TIMBRE and MORPH.

| Original control | SMN function |
| --- | --- |
| Depth, Latitude, Longitude | HARMONICS, TIMBRE, MORPH |
| Press and turn Depth / Latitude / Longitude | Global dispersion amount for H / T / M |
| Turn Browse | Adjust internal modulation amounts already active |
| Press and turn Browse | Select the synthesis model |
| Hold a destination control and turn Browse | Set that destination's internal modulation amount |
| Hold Scale and turn Browse | Set quantization glide time per voice |
| Hold Scale and turn Transpose without pressing it | Set the global scale root |
| Hold Transpose alone while inserting Transpose CV | Enable Polyrotate |
| Hold Preset and use A–F outside the preset menu | Recall or capture a chord memory |

For model, timbre and modulation edits, no A–F held means all eligible unlocked voices. Holding A–F explicitly selects those voices, including locked voices. **Dispersion amounts and scale/root remain global.** Fine provides smaller steps where supported, including all six internal modulation destinations.

The sphere audio engine, sphere selection/editing/recording/export and its associated effects have been removed. Sphere storage remains reserved and untouched. The Plaits Wavetable model uses its own built-in tables and does not restore the SWN sphere workflow. Select Bus preset control and its UART handling have also been removed; Preset + B now accesses chord memory B.

## 5. Synthesis models

### Selection, display and persistent IDs

MAIN and AUX are separate choices for each voice. A choice produces **one mono voice** routed into the existing stereo mix; MAIN/AUX do not add physical outputs.

Press and turn Browse to move through the catalogue, holding A–F for individual voices. WT CV adds an offset to the manual selection, wrapping through **31 active choices**. The CV uses the compact catalogue order, skipping empty display slots. Adding or reordering models can therefore change what a given voltage selects, while stored model IDs remain stable.

The selector reserves four pages of twelve slots. Each page uses six groups of three outer-ring LEDs: separator off, first variant, second variant. With LEDs numbered clockwise from the lower-left LED as 1, selectable positions are **2/3, 5/6, 8/9, 11/12, 14/15 and 17/18**. Rotation crosses pages automatically and skips unassigned slots; the fourth page is currently empty. Colours belong to slots, not to Plaits banks or stored IDs. Voices occupying the same LED on different pages alternate their colours.

The table follows the actual encoder/CV order. A slash separates two consecutive variants.

| Page | Outer-ring LED(s) | Model choice(s) | Stored ID(s) |
| --- | --- | --- | --- |
| 1 | 2 / 3 | VCF Lowpass MAIN / Highpass AUX | 32 / 33 |
| 1 | 5 / 6 | Virtual Analog MAIN / Hard Sync AUX | 0 / 1 |
| 1 | 8 / 9 | Waveshaping Soft MAIN / Hard AUX | 2 / 3 |
| 1 | 11 / 12 | Formant MAIN / Windowed Sines AUX | 6 / 7 |
| 1 | 14 / 15 | Swarm Saw MAIN / Sine AUX | 16 / 17 |
| 2 | 2 / 3 | FM MAIN / FM Sub AUX | 4 / 5 |
| 2 | 5 / 6 | Phase Distortion MAIN / Free-running Carrier AUX | 34 / 35 |
| 2 | 8 / 9 | Additive Bumps MAIN / Organ AUX | 8 / 9 |
| 2 | 11 / 12 | Wavetable MAIN / AUX | 10 / 11 |
| 2 | 14 / 15 | Terrain MAIN / AUX | 42 / 43 |
| 2 | 17 / 18 | Speech Vowel MAIN / Unfiltered AUX | 14 / 15 |
| 3 | 2 / 3 | String Inharmonic MAIN / Modal MAIN | 22 / 24 |
| 3 | 5 / 6 | String Audio In / Modal Audio In | 48 / 49 |
| 3 | 8 | Raw Exciter (Modal AUX) | 25 |
| 3 | 11 / 12 | Filtered Noise MAIN / Double Band-pass AUX | 18 / 19 |
| 3 | 14 / 15 | Particle Noise MAIN / AUX | 20 / 21 |

Native IDs reserve Plaits green-bank variants at 0–15, red at 16–31 and orange at 32–47, counting MAIN then AUX for each engine. Unimplemented IDs stay reserved, including String AUX 23. SMN's external-excitation variants use 48 and 49. These IDs are independent of display order. An unknown model ID in a decoded preset falls back to the first current entry, **VCF MAIN**. Resume records undergo stricter whole-record validation and may be rejected if they contain an unavailable model.

### Parameter reference

| Engine | HARMONICS | TIMBRE | MORPH |
| --- | --- | --- | --- |
| Virtual Analog VCF | Resonance and filter character | Pitch-relative cutoff | Oscillator shape, pulse width and sub-oscillator contribution |
| Virtual Analog | Detuning and oscillator intervals | Pulse width / sync character | Wave shape and oscillator balance, depending on variant |
| Waveshaping | Waveshaper character | Wavefolding amount | Source-wave asymmetry |
| Formant | Formant spacing / AUX character | Formant frequency | Formant shape |
| Swarm | Pitch spread | Grain rate | Grain duration through continuous sound and glissandi |
| FM, two operators | Modulator/carrier ratio | Modulation index | Feedback |
| Phase Distortion | Modulator ratio | Phase distortion amount | Distortion asymmetry |
| Additive | Spectral bump structure | Spectral centre | Spectral width and slope |
| Wavetable | Interpolated bank selection | Grid X position | Grid Y position |
| Terrain | Blend between terrains | Path size | Path position and flattening |
| Speech | Naive / SAM / LPC vowel character | Formant register | Vowel selection |
| String | Nonlinearity and inharmonicity | Excitation density/brightness | Damping and decay |
| Modal | Resonance structure | Excitation density/brightness | Resonance duration |
| Filtered Noise | Filter blend / AUX filter spacing | Noise update rate | Resonance |
| Particle Noise | Random resonator pitch spread on MAIN | Impulse density | Diffusion, then increasing resonance on MAIN |

### Adaptations from Plaits

SMN adapts the synthesis engines to six independent voices at 44.1 kHz. It uses the SWN's pitch, envelopes, gain controls and outputs. Plaits' global envelope, LPG and dedicated FM input are not imported. Native Plaits Trigger is left unpatched; SMN note/gate events articulate the existing voice envelope rather than triggering a native Plaits strike.

The following differences matter when comparing SMN with Plaits:

- **Wavetable:** 192 fixed tables are included in application flash, with the original rearranged fourth bank. H maps to the interpolated first half of Plaits' range; quantized bank selection and its progressive interpolation reduction are removed. No user-table loading is exposed. AUX retains its reduced-resolution character.
- **Speech:** continuous vowels only. H maps to half of Plaits' H range, covering naive, SAM and LPC synthesis; the final third of the SMN control stays on LPC. Word banks, consonant triggering and prosody are omitted. AUX supplies the excitation signal.
- **String:** one continuously excited string per SMN voice. The unused native three-string strike rotation is removed; String AUX is unavailable. The string's dispersion, nonlinearities, delay interpolation and low-frequency sample-rate conversion remain.
- **Modal:** MAIN and Audio In use **12 resonances**, retaining original modes **1–8, 10, 12, 14 and 16**, rather than all 24. Their original amplitudes and damping are retained without level compensation. Raw Exciter is the filtered excitation alone; in continuous mode, H and M do not shape that output.
- **Particle:** MAIN uses **four particles**, with compensated event density and impulse amplitude. Its seven-line diffuser runs at **22.05 kHz**, with filtered conversion and a 14-sample dry-path alignment delay. AUX keeps six raw impulse generators and is mainly controlled by TIMBRE. MAIN texture and high-frequency diffusion differ intentionally from Plaits.
- **Terrain:** the eight factory terrains remain, including three based on the built-in wavetable data. Tiny endpoint contributions are omitted near the end of H. AUX evaluates each surface at the midpoint of the path's two sampling positions and uses a dense lookup for its final sine; its sound can differ substantially from the original AUX. MAIN retains its two-point path calculation.
- **FM:** four-times oversampling is retained. AUX's final sine uses a dense, non-interpolated lookup with a small numerical difference; the emitted MAIN signal, phase and feedback calculations retain their previous behaviour.
- **MAIN/AUX switching:** Virtual Analog, Phase Distortion, Formant and Swarm suspend unused oscillators and resume their stored state when selected again. Particle suspends MAIN filters, diffuser and limiter during AUX. Returning from Raw Exciter to either Modal resonator variant clears the resonator and its limiter, including after another model family. These are not continuously running, phase-matched crossfades.

Other optimizations include direct normalized-frequency inputs, shared interpolation divisors, precomputed immutable tables and local filter states. Model-specific gains, limiters and clipping remain where required. Six voices share constant tables but have independent mutable synthesis state and random generators.

### External excitation through Waveform In

**String Audio In** and **Modal Audio In** replace internal noise excitation with the mono signal from Waveform In. All six voices can process that input with independent pitches, resonator parameters, input gains, levels and pans. Other models ignore the audio input.

HARMONICS and MORPH keep their resonator functions. **TIMBRE becomes input gain: `4 × TIMBRE²`**, giving ×0, ×1 and ×4 at minimum, midpoint and maximum. The input is softly clipped digitally before direct resonator injection; this cannot repair clipping already occurring in the analog input. Internal brightness is fixed at the noise variant's midpoint setting.

Noise and Audio In variants of each resonator share its vibration state. Changing between those sources does not clear the ringing; closing input gain stops new excitation but does not instantly silence an existing vibration. The Raw Exciter reset rule still applies to Modal.

## 6. Timbre, dispersion and CV reassignment

### Continuous controls with a reflected range

H/T/M are bounded 0–1 parameters controlled by endless encoders. Turning continuously traverses **minimum → maximum → minimum**, repeating without a value jump at the ends. Direct CV and modulation enter this reflected path too.

Manual position and manual dispersion offsets share **18 ramps of approximately 300 ms**, one per voice and parameter. CV remains responsive through the existing analog filters and does not restart those ramps. Continuous timbre and dispersion CV uses the filtered values, retaining fractional resolution instead of the discrete-change thresholds used for selections and triggers. Encoder smoothing is fixed-duration, not speed-adaptive.

### Three independent dispersions

Press and turn Depth, Latitude or Longitude to set the corresponding global dispersion amount from 0 to 1. Fine provides smaller steps. Each dimension uses its own fixed, irregular, zero-mean pattern across A–F; no random pattern is generated during performance.

At maximum amount, offsets in internal units are:

| Voice | H | T | M |
| --- | ---: | ---: | ---: |
| A | −0.54 | +0.99 | −1.15 |
| B | +1.38 | −0.57 | +0.60 |
| C | −0.89 | +1.47 | +0.15 |
| D | +0.28 | −1.28 | +1.48 |
| E | −1.26 | +0.23 | −0.34 |
| F | +1.03 | −0.84 | −0.74 |

Three internal units span a parameter's full 0–1 range; six units complete its reflected cycle. Offsets are added before reflection, so perceived voice spacing also depends on the base position. Dispersion CV adds to its manual amount, capped at 100%. The old selectable dispersion patterns and model WT Spread function are removed; pitch Spread remains separate.

### Changed input assignments

| Panel input | SMN role |
| --- | --- |
| Depth | Direct HARMONICS CV |
| Latitude | Direct TIMBRE CV |
| WT Spread | Direct MORPH CV |
| Dispersion | HARMONICS dispersion CV |
| Dispersion Pat | TIMBRE dispersion CV |
| Browse | MORPH dispersion CV; independent of internal modulation |
| WT | Model-selection offset |
| Spread / Chord | Chord-memory selection, except when assigned to global gate |
| Transpose | Common pitch CV, or rotating allocation with Polyrotate |
| Waveform In | External resonator excitation; firmware updates in bootloader mode |

Gate sharing takes priority over continuous CV functions, as described in section 9. The six individual V/oct/VCA inputs, Clock input and configurable LFO CV retain their relevant hardware functions. Electrical input/output ranges are unchanged.

## 7. Internal modulation

Each voice has **six independent modulation amounts**: gain, H, T, M, pitch and pan, for **36 amounts total**. The source is that voice's LFO, or its envelope in note modes, **before the CV-output level control**.

Turn Browse without pressing it while holding a destination:

| Held control | Destination |
| --- | --- |
| LFO→VCA | Gain |
| Depth / Latitude / Longitude | H / T / M |
| Transpose | Pitch |
| LFO/Gate/Trig | Pan |
| None | Adjust only amounts whose targets are already above zero |

Destinations can be combined, except that both LFO buttons together retain mode-selection priority. Add A–F for individual voices. Global edits respect voice and applicable parameter locks. Rotating Browse consumes the LFO-button click so it does not also change routing or output mode when released.

Amounts are bounded at 0–1 and never wrap. Fine gives five-times smaller steps. Internally they are stored as 0–480 counters, with normal steps of 5 and Fine steps of 1. Amount changes ramp over about 300 ms; the modulation source itself is not given that extra smoothing. There is no dedicated amount display.

Browse alone adds or subtracts the same amount from existing positive targets. It preserves their differences until a boundary is reached. Once an amount reaches zero, turning Browse alone cannot reactivate it; select that destination explicitly.

- **H/T/M:** positive offsets before reflection. A full amount traverses a parameter's complete range; the reflected result may move up or down.
- **Pitch:** up to ±2 semitones for sine, triangle and complex shapes in Mute mode; 0 to +2 semitones for the two ramp banks and envelope modes. It is applied after quantization and Fine, so it can move between scale notes. V/oct calibration temporarily neutralizes it without erasing its amount.
- **Gain:** adds opening to the fader, so a closed fader can become audible. Mute and zero external VCA CV still close the voice. This destination is available only in Mute with LFO→VCA disabled. Enabling LFO→VCA or entering a note mode clears the gain amount; it does not return automatically afterward.
- **Pan:** uses `2 × source − 1` for every shape and play mode. It moves around the stored manual pan, with up to ±1 at full amount and clipping at the left/right endpoints. Half-height source returns to manual pan.

The original **LFO→VCA** route remains multiplicative: it cannot open a closed fader. The CV-output gain affects neither internal modulation depth nor LFO→VCA depth.

## 8. Pitch, scales, chord memories and Polyrotate

### Global scale and root

Press and turn Octave/Scale to select one scale for all six voices. Hold Scale and turn Transpose **without pressing it** to choose a common root from C to B. This gesture takes priority over Fine; pressing Octave and Transpose together retains the V/oct calibration gesture. Play-mode changes no longer select another scale.

There are **25 choices including NONE**, in this order:

| Display | Choices, in order |
| --- | --- |
| Both rings off | NONE, no quantization |
| Inner ring, white | Octaves; Root & Fifth; Root & Fourth; Major Triad; Minor Triad; Sus2 Triad |
| Outer ring, white | Chromatic; Major Pentatonic; Minor Pentatonic |
| Outer ring, gold | Major; Lydian; Mixolydian |
| Outer ring, blue | Natural Minor; Dorian; Phrygian |
| Outer ring, pink | Harmonic Minor; ascending Melodic Minor; Harmonic Major |
| Outer ring, green | Minor Blues; Whole Tone; Half-Whole Diminished |
| Outer ring, purple | Whole-Half Diminished; Double Harmonic Major; Quarter-Tone Microtonal |

The root shifts the allowed note set; it does not directly transpose the voices. Fine tuning and internal pitch modulation remain after quantization. Pitch sent to the engines is limited to **0.1–11,025 Hz**.

Hold Fine and turn Transpose to adjust eligible voices by **1 cent per step**. Hold Fine, press Transpose and turn it (Fine + Spread) to add **−3, −2, −1, +1, +2 and +3 cents** to voices A–F per positive step. Reverse rotation subtracts the same pattern. A–F selection and Transpose locks follow the normal edit rules.

Fine is stored in whole cents, with limits of **±2,160 cents**. The Fine reset sets eligible voices to zero; the sound reset preserves Fine. The ring shows white at zero, blue below and red above, with faster pulses as the offset grows and a 50 ms minimum period.

### Six chord memories

Outside the preset menu, hold Preset and use **one** A–F button:

- Release A–F before one second to recall its six-note chord.
- Hold both controls for one second to capture the current six pitches. The selected button turns red for one second.
- Release Preset first, or select several A–F together, to cancel the gesture.

Capture records post-scale, pre-Fine/pre-modulation pitches, rounded to semitones for storage. During glide it records the destination. Recall restores pitch and octave and clears manual pitch Spread on eligible voices. Existing CV still adds afterward, so repeated capture/recall with pitch CV present can accumulate its offset.

Each memory also stores the global scale and root. These are recalled only if both the current and stored scales are enabled; a chord recall never switches quantization on or off. Voice locks and Transpose locks protect the corresponding pitches.

Defaults are **A = unison, B–F = Spread positions 1–5**, based on C3, with no stored scale. **Preset + Fine + any A–F** resets all six chord memories without recalling a chord or changing the current scale/glide. Fine + Preset alone retains undo.

Spread/Chord CV selects six approximately equal segments over 0–5 V. Recall occurs on segment changes, with hysteresis, not continuously; stable CV leaves manual recalls usable. There is no initial recall at boot. Disconnecting from a segment other than A recalls A. A CV recall flashes its button green for 300 ms. Global-gate use overrides chord selection.

Manual recalls retrigger Keyboard, CV/Gate and Sustain envelopes. CV recalls normally change their pitches without triggering. In Note mode, both recall methods change the pitch target; only the normal stable-note detector can request an automatic attack. A Transpose-locked voice keeps its pitch, but chord recalls can still retrigger its envelope; a globally locked voice stays excluded.

### Quantization glide

Hold Scale and turn **unpressed Browse** to set glide time, globally or with A–F. Fine gives five subdivisions per main interval. There are **91 durations from 0 to 30 seconds**, stored independently per voice. The outer ring displays the six durations; Fine positions blend between neighbouring LEDs.

Glide requires an active scale and follows a straight trajectory in note space, passing through intermediate pitches. A new target starts from the reached position. Changing duration affects the next transition.

- **Mute:** quantized target changes glide, including while muted.
- **Note:** a newly detected stable quantized note can glide and trigger even from rest. Direct button attacks glide only when overlapping an active envelope.
- **Keyboard:** overlapping attacks can glide; isolated attacks take their pitch directly.
- **CV/Gate and Sustain:** pitch changes while the envelope is positive can glide. An isolated attack takes its pitch directly, with a short protection window of about 5 ms for incoming pitch/gate timing differences.

Note and Keyboard hold their pitch destination during a cycle; later changes wait for the next accepted attack or cycle end. Fine and internal modulation remain live. Glide continues during silence. Disabling Scale cancels the current glide but retains its configured time; preset recall and undo start directly at the restored pitch.

### Polyrotate

Hold **Transpose alone**, then insert a cable into **Transpose CV**. The first stable note establishes a reference; each subsequent changed quantized note updates the next voice's pitch offset, cycling **A → B → C → D → E → F**. Repeated notes do not advance it. Other voices retain their offsets. This allocates pitch only; it does not generate a global trigger.

If Scale is NONE, activation selects Chromatic. Disconnecting the cable or selecting NONE disables Polyrotate. Pressing Transpose with an already connected cable does not activate it; reinsert while holding Transpose. Locks still apply without removing voices from the rotation. Activation/deactivation uses a 500 ms white animation, and assignments flash white for 150 ms.

Only the enabled flag is saved. Offsets and allocation position are rebuilt after recall or restart; an enabled recall without the cable becomes disabled. Chord recall does not reset the rotation.

## 9. LFOs, envelopes and gate routing

### Eight banks of 33 shapes

SMN replaces the original LFO shape set with **264 table selections**, arranged as eight banks:

| Bank | Shape family | Colour |
| --- | --- | --- |
| 1 | Ramp-down | Blue |
| 2 | Ramp-up | Purple |
| 3 | Sine Wavefold | Yellow |
| 4 | Sine Symmetry | Orange |
| 5 | Triangle Size | Green |
| 6 | Triangle Log/Exp 2 | Cyan |
| 7 | Complex Soft | Magenta |
| 8 | Complex Hard | Red |

Turn Shape to change bank with wraparound. **Fine + Shape** selects positions 1–33 with end stops. Changing bank keeps the position. Saturation increases across a bank. The default is **bank 4, position 17**, the central sine (stored index 115).

The selected bank/position now survives every play-mode change, including returning to Mute after editing an envelope. No blue ramp is imposed on entry to Note/Keyboard. Shapes change directly without a crossfade or waiting for the next cycle. In envelope modes, selection shows its colour for one second even when the envelope is at zero. Sustain pauses at the midpoint for the Wavefold and triangle banks, and at the first global maximum for other banks.

Speed, phase, output level, audio-rate operation, output LFO/Gate/Trig selection and internal LFO→VCA routing retain their existing roles. External clock timing now uses a hardware time counter and gradual phase correction, preserving divided-cycle progress instead of repeatedly resetting it. This improves resilience to delayed software updates; missed input edges cannot be reconstructed.

### Changed play behaviour

The five-mode cycle remains Mute, Note, Keyboard, CV/Gate and CV/Gate/Sustain. Use the two LFO buttons together, optionally with A–F, as on the SWN. Normal voice colours are now **gold, pink, light purple, white and pale yellow**, respectively.

In **Note and Keyboard**, a new button press restarts a complete envelope. Holding does not repeat it; releasing does not truncate it. Both modes hold pre-Fine pitch during the cycle, including CV, octave, transpose and spread. Pending values become available afterward; Fine and internal modulation remain active.

**Note** additionally detects changed quantized notes when available, requiring six stable pitch observations, roughly 3–4 ms including attack dispatch. NONE disables automatic note-change attacks. Holding A–F does not mask automatic Note attacks or incoming CV/Gate edges. Keyboard has no automatic note-change attack.

### Shared gate inputs

Both external-gate modes use the following assignments when the corresponding individual V/oct input is connected:

| Voice | Gate input |
| --- | --- |
| A | WT |
| B | Dispersion |
| C | Depth |
| D | Dispersion Pat |
| E | Latitude |
| F | WT Spread |

The shared input's usual timbre, dispersion or model-selection role is suppressed while it serves that voice's gate. **Gate A no longer uses Waveform In**, leaving the audio input available in all modes.

When at least one voice uses CV/Gate or Sustain **and Transpose CV is connected**, Spread/Chord CV becomes a **global gate** instead of selecting chord memories. Its rising edge retriggers all six voices; its held level sustains voices in Sustain mode. Mixed-mode patches must account for this priority. Global and individual sustain levels are combined before publication to avoid a false release between updates.

## 10. Levels and stereo placement

The default stereo assignment is now **A/C/E fully left and B/D/F fully right**. Both outputs are needed to hear all six voices in that initial configuration. Saved pans are retained on recall. MAIN/AUX selection does not change a voice's pan.

Hold a voice button and move its fader to edit pan while retaining its previous level. After release, return the fader to that previous level to resume volume control. Internal pan modulation moves around this manual setting without overwriting it.

## 11. Locks, presets and automatic resume

### Parameter locks

The existing voice lock remains available with **Fine + A–F**, without turning a control. Explicit A–F edits and individual V/oct remain available.

SMN adds independent **H, T, M and Transpose locks per voice**. Hold **Fine + A–F + the relevant parameter encoder**, then release all involved controls without rotation. The lock toggles once after complete release. Multiple voices/parameters may be selected. Any rotation cancels the pending toggle and performs its normal edit.

Locked H/T/M ignores global manual changes, direct CV and dispersion. Its reached dispersion offset is folded into the stored base when locking, then removed against the current global offset when unlocking. The direct CV contribution is removed while locked. Existing internal modulation continues; explicit A–F edits remain possible.

Transpose lock protects manual pitch, octave, Fine, shared Transpose CV, pitch Spread and chord pitch recall. Individual V/oct stays active. Scale/root remain global. A full preset recall restores that preset's values and locks. Parameter LEDs blink to indicate targeted locks; the sound reset clears these locks but retains global voice locks.

### Numbered presets

The original **108 slots** remain at their original addresses. Rotate Preset to enter the menu and select a slot; rotation does not load it. A press begun outside the menu belongs to the chord/undo shortcuts instead.

- **Short press and release:** load immediately, with a green animation.
- **Hold about 2–6 seconds, then release:** save immediately, with a red animation.
- **Hold beyond about 6 seconds:** request deletion; the multicolour confirmation step remains.
- **Fine + Preset:** undo the last supported preset operation. Undo is temporary and does not survive power-off.

Loading and saving no longer require a second confirmation click. SMN slots are shown in yellow; recognized SWN signatures are shown in pink. Loading an empty or unsupported slot creates defaults in RAM without erasing stored data. Saving explicitly replaces the selected slot.

Presets include model IDs, manual H/T/M, dispersions, all modulation amounts, pitch settings, chord memories, glide times, pan, play modes, locks, LFO settings, Polyrotate. Physical fader positions and live voltages remain effective after recall.

### Automatic power-cycle recall

After about **one second without a change to a memorized setting**, SMN schedules an independent resume snapshot. Allow a few seconds for writing before power-off. Startup restores the latest valid snapshot; it does not implicitly load a numbered preset.

Resume includes manual settings and brightness, but excludes live CV, fader levels, LFO phases, active gates, Polyrotate offsets and intermediate ramps. Restarting does not resume an exact waveform phase or a held note.

Without valid resume data, defaults are VCF MAIN, H/T/M at 0.5, zero dispersions and internal modulation amounts, no quantization, zero glide, Polyrotate off, the central sine LFO and alternating hard pans. Numbered presets and spheres are not erased.

The journal uses external-flash sectors **0 and 1**, unused by the original SWN. It alternates records with CRC32 checks and a separate final commit. Automatic writing uses cooperative flash transfers without stopping audio/LFO timers. Brightness is also stored here, avoiding an internal-flash erase when adjusting it. Original SWN retains its own internal brightness setting.

### Public-format compatibility

Beta 0.1 writes **SMN2 preset v4** and **resume v6**. Presets remain 1,024 bytes each. The stable reference also retains its older development readers:

| Record | Read behaviour |
| --- | --- |
| SMN2 v4 | Restore all beta 0.1 preset fields, including pan modulation |
| Resume v6 | Restore all beta 0.1 resume fields |
| SMN2 v3 / resume v5 (development formats) | Restore saved settings with pan modulation set to zero |
| SWN and unsupported SMN presets | Recognize where applicable, but load defaults without rewriting the slot |
| Unsupported or invalid resume data | Ignore and look for another valid copy; otherwise use defaults |

Other unsupported development formats are not converted. Startup does not rewrite or erase those records. A later explicit preset save writes the current format; a real setting change can produce a new resume record. Fine values remain in whole cents in the public preset and resume formats.

Future SMN releases must preserve reading of prior public SMN formats, beginning with beta 0.1. This source revision retains the v3/v5 development readers to reproduce the stable beta 0.1 binary exactly; their presence creates no future compatibility obligation. It does not promise that beta 0.1 can read formats written by later development firmware. Returning from a newer firmware may therefore restore older compatible settings or defaults while leaving unsupported records stored until an explicit save or a normal auto-save after a setting change.

## 12. Reset shortcuts and defaults

Hold Preset in play mode and add the listed control. These are performance resets, not a factory reset. Live CV remains active afterward.

| Added control | Result |
| --- | --- |
| Depth / Latitude / Longitude | Corresponding H/T/M at midpoint and its global dispersion at zero; applicable locks respected |
| Browse, pressed | All six manual model choices set to the first catalogue entry, including locked voices |
| Transpose | Reset transpose and pitch Spread on eligible voices |
| Fine + Transpose | Reset Fine tuning on eligible voices |
| Octave/Scale | Reset eligible octaves |
| Fine + Octave/Scale | Global Scale NONE and root C; octaves retained |
| LFO Shape | Default central sine on eligible LFOs |
| LFO Speed | Default LFO speeds |
| LFO→VCA | Disable internal LFO→VCA on all six voices, including their cached settings |
| LFO/Gate/Trig | Set all six outputs to LFO, including cached settings |
| Both LFO buttons | Sound reset, described below |
| Fine + any A–F, outside the menu | Reset all six chord memories |

The octave/scale reset executes after both encoders are released. Fine held during their common press selects the scale/root reset instead of the octave reset. The two individual LFO-button resets and sound reset execute after both LFO buttons are released; the sound reset takes priority.

The **sound reset** restores first model, H/T/M midpoint, zero dispersions and internal modulation, initial LFOs, zero transpose/Spread, initial octave, zero glide, Polyrotate off. It clears targeted H/T/M/Transpose locks. It preserves play modes, scale/root, chord memories, Fine tuning, pans and global voice/LFO locks. It erases no numbered presets, spheres, calibrations or system settings. Auto-save subsequently records the changed current state.

Individual H/T/M resets do not clear their internal modulation amounts. The two individual LFO-button resets also leave modulation amounts unchanged.

## 13. Display and smaller behaviour changes

- **Normal outer ring:** six H/T/M triplets, A at LEDs 1–3 through F at 16–18. Each LED fades from off at minimum, through its model-slot colour at midpoint, to white at maximum. It follows the value sent to the engine, including reflection, CV and internal modulation.
- **Dispersion encoder LEDs:** the global manual-plus-CV amount, capped at 100%, runs from attenuated white through gold at 50% to red at 100%. Pressing preserves that gradient. Fine blue and Transpose/Spread pink retain priority. This shows the global amount, not per-voice lock exceptions.
- **Model colours:** fixed slot palette, approximately yellow/red on page 1, blue/green on page 2 and pink/purple on page 3. LED intensities are corrected from the supplied sRGB palette.
- **Scale/root display:** both rings show the scale layout above. Root uses the Transpose positions, with natural notes white and sharps gold. Scale remains visible while held and for about 700 ms afterward; root and octave movements have their own brief views. Holding A–F does not extend these timeouts.
- **Button brightness:** the six voice buttons and both LFO buttons use twice the global brightness factor, capped at full intensity. Other LEDs keep the normal global factor. Preset + unpressed Browse still adjusts brightness.
- **Calibration:** each encoder has its own hold counter. Holding Browse for more than two seconds correctly saves V/oct calibration and exits. Pitch quantization no longer retains a stale frequency/octave in the old threshold dead band.
- **Startup version:** the public SMN version display is separate from the retained internal SWN system/calibration version. Beta 0.1 leaves the inner version ring off and uses the outer-ring minor-version indication.

## 14. Building and navigating the code

### Build the application and audio update

Required tools are GNU Make and an ARM toolchain providing `arm-none-eabi-gcc`, `arm-none-eabi-g++` and matching binutils/libraries. The adapter uses **C++17**, Cortex-M7 hard-float settings and `-O3`, without exceptions or RTTI. The current code has been built with GCC 15; older SWN toolchain instructions do not establish compatibility with these sources.

From the repository root:

```sh
make -j4 all
```

The ready-to-install beta 0.1 files are provided in `firmware/`:

| Artifact | Purpose |
| --- | --- |
| [firmware/main.bin](firmware/main.bin) | Raw application for `0x08010000` |
| [firmware/main.wav](firmware/main.wav) | Audio update through Waveform In |

The Makefile generates local compilation outputs separately; it does not update the files in `firmware/` automatically. For a new release, copy the matching generated BIN and WAV into `firmware/` and keep their source revision and identity together. Debug symbols, link maps and intermediate files are not part of this distribution folder.

Application compilation does not regenerate `main.wav`. WAV generation needs Python and NumPy. With `python` selecting the intended environment, run:

```sh
make wav
```

Generate the WAV from the matching newly compiled BIN before copying both files into `firmware/`. No bootloader rebuild is needed for normal application development. The legacy `release` target uses SWN naming and local archive paths; it is not the public SMN packaging procedure.

### Hardware programming

Developers using SWD must program **only the application at `0x08010000`**. Preserve the bootloader below `0x08008000` and calibrations/system settings beginning at `0x08008000`. Make a fresh, verified full internal-flash backup before writing, then read back and compare the complete flash afterward. An internal-flash backup does not include external presets or spheres. Do not use mass erase, combined factory images or the unguarded `make flash` target for a normal update. With the compatible ST-LINK V2 used during development, reads and writes require `--freq 1000` (950 kHz effective); programming at 100 kHz previously failed after erasure. Identify your own probe's pinout and connect it with power off; do not power the SWN from the probe.

### Source map and real-time constraints

| Location | Responsibility |
| --- | --- |
| `src/`, `inc/` | Application, controls, state, modulation, LFOs, presets and hardware interfaces |
| `src/main.c`, `src/oscillator.c` | Startup and codec callback, voice processing and stereo mix |
| `src/params_update.c`, `src/params_changes.c` | Control gestures, live parameters and lock-aware edits |
| `src/params_lfo*.c`, `src/lfo_wavetable_bank.c`, `src/led_cont.c` | LFO timing/shapes and UI display |
| `src/smn_models.c`, `inc/smn_models.h` | Catalogue order, stable IDs, slot mapping and gains |
| `src/smn_synth.cc` | C-facing adapter and private Plaits engine state for six voices |
| `src/smn_controls.c` | Parameter reflection, ramps, internal modulation and glide |
| `src/smn_chords.c`, `src/quantz_scales.c` | Chord memories and quantization |
| `src/smn_patch.c`, `src/smn_resume.c` | Preset serialization, version conversion and resume journal |
| `vendor/plaits/`, `vendor/stmlib/` | Adapted DSP sources and constant resources; provenance in `vendor/SOURCES.json` |
| `stm32/` | MCU support, startup, peripherals and linker script |
| `stm_audio_bootloader/` | Application WAV encoder |
| `bootloader/` | Separately built audio receiver; preserved during updates |

The C application sends coherent parameter snapshots to the C++ adapter. The codec callback renders **16 samples per voice**, about every **362.8 µs**, then applies voice processing and the stereo mix. Audio, analog acquisition, parameter updates, LFO/PWM and UI run in separate interrupt contexts. The main loop services preset actions and cooperative auto-save.

The audio path performs no heap allocation or storage access. Shared resources are immutable; oscillators, filters, delays, interpolation history and random state are private to each voice. String delays occupy 30,720 bytes of DTCM. Particle diffuser buffers occupy 98,304 bytes of SRAM1 and are explicitly initialized before audio starts.

The application links through `stm32/device/STM32F765ZGTx_FLASH.ld`, loads at `0x08010000` and executes through the ITCM alias at `0x00210000`. External sectors 0/1 hold resume data; sphere space begins at sector 16 and presets at sector 217. Changing memory layout or shared drivers requires checking the audio bootloader as well as the application.

The adapted synthesis sources used by the build are included in `vendor/`, with their provenance recorded in `vendor/SOURCES.json`.

### Verification

Development checks have covered DSP comparisons, independent voices, simulated callbacks, controls, LEDs, play modes and storage compatibility. These host-side checks do not measure STM32 execution time or validate electrical behaviour. A successful ARM build does not establish the safety of an arbitrary flash image or prove an audio update succeeded.

## 15. Current limits and reporting problems

SMN remains a beta. Six independent voices are implemented, but **every model combination is not guaranteed to run without clicks at every pitch and modulation setting**. Heavy combinations involving Wavetable, Terrain, Particle, resonators or FM AUX have exposed limits during development. Subsequent optimizations improved several cases, but hardware qualification remains incomplete.

Model switches are not guaranteed click-free. Some suspended-state policies and deliberate DSP reductions are audible differences from Plaits. Terrain can produce a noisy texture near the upper end of H; its precise relation to reported clicks has not been established.

Host tests, flash readback and listening results establish different things. Beta 0.1 was programmed and fully read back on the development module. The maintainer selected it as the stable public baseline after comparing it with later development versions. This does not establish a CPU margin for every combination of models and settings. Earlier audio-update round-trip results do not certify every later build. Physical power interruption during auto-save and all storage failure scenarios have not been exhaustively qualified.

For a useful issue report, include the exact version or source revision and artifact, models per voice, approximate pitches, H/T/M settings, play modes, modulation and connected CV. State whether the problem follows startup, a preset recall, a model switch or a specific gesture, and give reproducible steps.

## 16. Credits and licences

The project idea, conception, design, testing and validation are human-led. AI tools have been used to assist with co-writing the code. The scope and limits of current validation are described above.

SMN builds on **Dan Green and Hugo Paris / 4ms Company's SWN firmware**, **Émilie Gillet / Mutable Instruments' Plaits and stmlib**, and the existing STM audio bootloader and STM32 support code. The project is independently developed and is not endorsed by those authors or companies.

SMN-specific contributions are distributed under the **MIT License**, copyright (c) 2026 Rodrigo Diaz. See [LICENSE](LICENSE) for the licence text and [THIRD_PARTY_LICENSES.md](THIRD_PARTY_LICENSES.md) for the SWN, Plaits/stmlib, audio bootloader and ARM/ST notices. Third-party files retain their own copyrights and licences. Imported DSP provenance and adaptations are recorded in [vendor/SOURCES.json](vendor/SOURCES.json). Include both licence documents when redistributing source or firmware packages.

The original SWN hardware design has a separate CC BY-NC-SA licence as described in [THIRD_PARTY_LICENSES.md](THIRD_PARTY_LICENSES.md). That hardware licence is distinct from the firmware's software licence. Original derivative-work guidelines exclude 4ms branding and logos from derivative hardware.
