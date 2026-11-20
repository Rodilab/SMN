// SMN adapter: engine states and scratch buffers are private to each voice.
#include "smn_synth.h"

#include <algorithm>
#include <cmath>

#include "plaits/dsp/engine/fm_engine.h"
#include "plaits/dsp/engine/waveshaping_engine.h"
#include "plaits/dsp/engine/virtual_analog_engine.h"
#include "plaits/dsp/engine2/virtual_analog_vcf_engine.h"
#include "plaits/dsp/engine2/phase_distortion_engine.h"
#include "plaits/dsp/engine/additive_engine.h"
#include "plaits/dsp/engine/noise_engine.h"
#include "plaits/dsp/engine/particle_engine.h"
#include "plaits/dsp/engine/modal_engine.h"
#include "plaits/dsp/engine/string_engine.h"
#include "plaits/dsp/engine/grain_engine.h"
#include "plaits/dsp/engine/speech_engine.h"
#include "plaits/dsp/engine/swarm_engine.h"
#include "plaits/dsp/engine/wavetable_engine.h"
#include "plaits/dsp/engine2/wave_terrain_engine.h"
#include "stmlib/dsp/limiter.h"

namespace {

static_assert(SMN_SYNTH_BLOCK_SIZE <= plaits::kMaxBlockSize,
              "SMN audio block exceeds the Plaits maximum");
static_assert(SMN_SYNTH_BLOCK_SIZE == plaits::GrainEnvelope::kTransitionBlockSize,
              "Swarm transition table must match the SMN audio block");
static_assert(SMN_SYNTH_BLOCK_SIZE == plaits::WavetableEngine::kNativeBlockSize,
              "Wavetable smoothing must match the SMN audio block");
static_assert(SMN_SYNTH_BLOCK_SIZE == plaits::kParticleBlockSize,
              "Particle continuous path requires 16 samples");
static_assert(SMN_SYNTH_BLOCK_SIZE == plaits::kStringBlockSize,
              "String continuous path requires 16 samples");
static_assert(plaits::kSampleRate == SMN_SYNTH_SAMPLE_RATE_HZ &&
              plaits::kCorrectedSampleRate == SMN_SYNTH_SAMPLE_RATE_HZ,
              "Use the SMN-adapted Plaits copy at 44.1 kHz");

struct Voice {
    plaits::WaveshapingEngine engine;
    plaits::FMEngine fm;
    plaits::WavetableEngine wavetable;
    plaits::WaveTerrainEngine terrain;
    float terrain_scratch[4 * plaits::kMaxBlockSize];
    plaits::GrainEngine formant;
    plaits::SwarmEngine swarm;
    alignas(plaits::SwarmVoice) uint8_t swarm_scratch[
        sizeof(plaits::SwarmVoice) * plaits::kNumSwarmVoices];
    stmlib::Limiter swarm_limiter;
    plaits::SpeechEngine speech;
    float speech_scratch[2 * plaits::kMaxBlockSize];
    stmlib::Limiter speech_limiter;
    plaits::VirtualAnalogVCFEngine vcf;
    plaits::VirtualAnalogEngine va;
    float va_scratch[plaits::kMaxBlockSize];
    plaits::PhaseDistortionEngine pd;
    float pd_scratch[4 * plaits::kMaxBlockSize];
    plaits::AdditiveEngine additive;
    float additive_scratch[plaits::kNumHarmonics];
    plaits::ParticleEngine particle;
    stmlib::Limiter particle_limiter;
    plaits::NoiseEngine noise;
    float noise_scratch[plaits::kMaxBlockSize];
    stmlib::Limiter noise_limiter;
    plaits::StringEngine string;
    stmlib::Limiter string_limiter;
    plaits::ModalEngine modal;
    float modal_scratch[plaits::kMaxBlockSize];
    stmlib::Limiter modal_limiter;
    bool modal_aux_selected;
    float input_gain;
    float aux[SMN_SYNTH_BLOCK_SIZE];
};

Voice voices[SMN_SYNTH_MAX_VOICES];
// Private string delays in DTCM, cleared by startup and Init before audio.
// Only the CPU accesses them; DMA and cache maintenance are not involved.
alignas(32) float string_scratch[SMN_SYNTH_MAX_VOICES][plaits::kStringScratchFloats];
// Private diffuser histories in SRAM1. NOLOAD memory is not cleared by
// startup, so Init clears it before the codec callback is installed.
// These buffers are CPU-only and do not hold persistent data.
alignas(32) uint16_t particle_scratch[SMN_SYNTH_MAX_VOICES][8192]
#if !defined(TEST)
    __attribute__((section(".sram1data")))
#endif
    ;
uint8_t active_voices;

bool unit_parameter(float value)
{
    return std::isfinite(value) && value >= 0.0f && value <= 1.0f;
}

}  // namespace

extern "C" int smn_synth_init(uint8_t voice_count)
{
    active_voices = 0;
    if (voice_count == 0 || voice_count > SMN_SYNTH_MAX_VOICES ||
        SMN_SYNTH_MAX_FREQUENCY_HZ !=
            plaits::kMaxFrequency * plaits::kSampleRate)
        return 0;

    for (uint8_t i = 0; i < voice_count; ++i) {
        Voice& voice = voices[i];
        // Give each engine its private buffers before audio starts.
        stmlib::BufferAllocator allocator(voice.va_scratch, sizeof(voice.va_scratch));
        voice.engine.Init(&allocator);
        voice.fm.Init(&allocator); // No scratch allocation.
        voice.wavetable.Init(&allocator); // Immutable shared tables; no allocation.
        stmlib::BufferAllocator terrain_allocator(voice.terrain_scratch, sizeof(voice.terrain_scratch));
        voice.terrain.Init(&terrain_allocator);
        voice.formant.Init(&allocator);
        stmlib::BufferAllocator speech_allocator(voice.speech_scratch, sizeof(voice.speech_scratch));
        voice.speech.Init(&speech_allocator);
        voice.speech_limiter.Init();
        stmlib::BufferAllocator swarm_allocator(voice.swarm_scratch, sizeof(voice.swarm_scratch));
        voice.swarm.Init(&swarm_allocator);
        voice.swarm.Seed(0x53776172u + i * 0x9e3779b9u);
        voice.swarm_limiter.Init();
        voice.vcf.Init(&allocator);
        voice.va.Init(&allocator);
        stmlib::BufferAllocator pd_allocator(voice.pd_scratch, sizeof(voice.pd_scratch));
        voice.pd.Init(&pd_allocator);
        stmlib::BufferAllocator additive_allocator(voice.additive_scratch, sizeof(voice.additive_scratch));
        voice.additive.Init(&additive_allocator);
        voice.additive.Reset(); // Includes the silent gaps of the organ spectrum.
        stmlib::BufferAllocator noise_allocator(voice.noise_scratch, sizeof(voice.noise_scratch));
        voice.noise.Init(&noise_allocator);
        voice.noise.Seed(0x21u + i * 0x9e3779b9u);
        voice.noise_limiter.Init();
        stmlib::BufferAllocator particle_allocator(particle_scratch[i], sizeof(particle_scratch[i]));
        voice.particle.Init(&particle_allocator);
        if (particle_allocator.free() != 0) return 0;
        voice.particle.Seed(0x50617274u + i * 0x9e3779b9u);
        voice.particle_limiter.Init();
        stmlib::BufferAllocator string_allocator(string_scratch[i], sizeof(string_scratch[i]));
        voice.string.Init(&string_allocator);
        if (string_allocator.free() != 0) return 0;
        voice.string.Seed(0x53747267u + i * 0x9e3779b9u);
        voice.string_limiter.Init();
        stmlib::BufferAllocator modal_allocator(voice.modal_scratch, sizeof(voice.modal_scratch));
        voice.modal.Init(&modal_allocator);
        voice.modal.Seed(0x6d2b79f5u + i * 0x9e3779b9u);
        voice.modal_limiter.Init();
        voice.modal_aux_selected = false;
        voice.input_gain = 1.0f;
        std::fill_n(voice.aux, SMN_SYNTH_BLOCK_SIZE, 0.0f);
    }
    active_voices = voice_count;
    return 1;
}

extern "C" int smn_synth_render_with_input(uint8_t voice_index,
                                const SmnSynthParameters *parameters,
                                const float *audio_input, float *main_output,
                                size_t frames)
{
    if (!main_output || frames != SMN_SYNTH_BLOCK_SIZE)
        return 0;

    if (!parameters || voice_index >= active_voices) {
        std::fill_n(main_output, frames, 0.0f);
        return 0;
    }

    // The caller supplies a coherent snapshot, separate from shared UI state.
    const SmnSynthParameters p = *parameters;
    if (!std::isfinite(p.frequency_hz) ||
        p.frequency_hz < SMN_SYNTH_MIN_FREQUENCY_HZ ||
        !unit_parameter(p.harmonics) || !unit_parameter(p.timbre) ||
        !unit_parameter(p.morph) || !smn_model_valid(p.model_id)) {
        std::fill_n(main_output, frames, 0.0f);
        return 0;
    }

    const bool external = p.model_id == SMN_MODEL_STRING_AUDIO_IN ||
                          p.model_id == SMN_MODEL_MODAL_AUDIO_IN;
    if (external && audio_input) {
        for (size_t i = 0; i < frames; ++i) {
            if (!std::isfinite(audio_input[i]) || std::abs(audio_input[i]) > 1.0f) {
                std::fill_n(main_output, frames, 0.0f);
                return 0;
            }
        }
    }
    Voice& voice = voices[voice_index];
    const float frequency_hz = std::min(p.frequency_hz, SMN_SYNTH_MAX_FREQUENCY_HZ);
    // Cycles per sample, computed even under continuous pitch modulation.
    const float frequency = frequency_hz * (1.0f / plaits::kSampleRate);

    plaits::EngineParameters engine_parameters = {};
    engine_parameters.trigger = plaits::TRIGGER_UNPATCHED;
    engine_parameters.harmonics = p.harmonics;
    engine_parameters.timbre = p.timbre;
    engine_parameters.morph = p.morph;
    const plaits::EngineOutput output =
        (p.model_id == SMN_MODEL_WAVESHAPING_AUX ||
         p.model_id == SMN_MODEL_VIRTUAL_ANALOG_VCF_AUX ||
         p.model_id == SMN_MODEL_VIRTUAL_ANALOG_AUX ||
         p.model_id == SMN_MODEL_PHASE_DISTORTION_AUX ||
         p.model_id == SMN_MODEL_ADDITIVE_AUX ||
         p.model_id == SMN_MODEL_NOISE_AUX ||
         p.model_id == SMN_MODEL_PARTICLE_AUX ||
         p.model_id == SMN_MODEL_MODAL_AUX ||
         p.model_id == SMN_MODEL_FORMANT_AUX ||
         p.model_id == SMN_MODEL_SPEECH_AUX ||
         p.model_id == SMN_MODEL_SWARM_AUX ||
         p.model_id == SMN_MODEL_WAVETABLE_AUX ||
         p.model_id == SMN_MODEL_TERRAIN_AUX ||
         p.model_id == SMN_MODEL_FM_AUX)
        ? plaits::ENGINE_OUTPUT_AUX : plaits::ENGINE_OUTPUT_MAIN;
    // Engines keep their state while another model is selected. MAIN/AUX share
    // an engine; VA, PD and Formant pause their unselected oscillators.
    if (external) {
        // One gain ramp per voice; no dust generation, excitation LPF or AUX.
        float excitation[SMN_SYNTH_BLOCK_SIZE];
        stmlib::ParameterInterpolator gain(&voice.input_gain,
            4.0f * p.timbre * p.timbre, 1.0f / SMN_SYNTH_BLOCK_SIZE);
        for (size_t i = 0; i < frames; ++i) {
            const float g = gain.Next();
            excitation[i] = audio_input ? stmlib::SoftClip(audio_input[i] * g) : 0.0f;
        }
        if (p.model_id == SMN_MODEL_STRING_AUDIO_IN) {
            voice.string.RenderExternal(frequency, p.harmonics, p.morph,
                                         excitation, main_output);
            voice.string_limiter.Process(1.0f, main_output, frames);
        } else {
            if (voice.modal_aux_selected) {
                voice.modal.ResetResonator();
                voice.modal_limiter.Init();
            }
            voice.modal_aux_selected = false;
            voice.modal.RenderExternal(frequency, p.harmonics, p.morph,
                                        excitation, main_output, frames);
            voice.modal_limiter.Process(1.0f, main_output, frames);
        }
        for (size_t i = 0; i < frames; ++i)
            main_output[i] = std::clamp(main_output[i], -1.0f, 1.0f);
    } else if (p.model_id == SMN_MODEL_VIRTUAL_ANALOG_VCF_MAIN ||
        p.model_id == SMN_MODEL_VIRTUAL_ANALOG_VCF_AUX) {
        voice.vcf.RenderAtFrequency(engine_parameters, frequency,
                                    main_output, voice.aux, frames, output);
    } else if (p.model_id == SMN_MODEL_VIRTUAL_ANALOG_MAIN ||
               p.model_id == SMN_MODEL_VIRTUAL_ANALOG_AUX) {
        voice.va.RenderAtFrequency(engine_parameters, frequency,
                                  main_output, voice.aux, frames, output);
    } else if (p.model_id == SMN_MODEL_PHASE_DISTORTION_MAIN ||
               p.model_id == SMN_MODEL_PHASE_DISTORTION_AUX) {
        voice.pd.RenderAtFrequency(engine_parameters, frequency,
                                  main_output, voice.aux, frames, output);
    } else if (p.model_id == SMN_MODEL_ADDITIVE_MAIN ||
               p.model_id == SMN_MODEL_ADDITIVE_AUX) {
        voice.additive.RenderAtFrequency(engine_parameters, frequency,
                                        main_output, voice.aux, frames, output);
    } else if (p.model_id == SMN_MODEL_NOISE_MAIN ||
               p.model_id == SMN_MODEL_NOISE_AUX) {
        voice.noise.RenderAtFrequency(engine_parameters, frequency,
                                     main_output, voice.aux, frames, output);
        // Plaits requests a limiter (gain -1) for Noise, independently of LPG.
        // One mono limiter follows the selected signal through MAIN/AUX changes.
        voice.noise_limiter.Process(
            1.0f, main_output, frames);
        // Clip after limiting, as in Plaits, before mixing the six voices.
        // Samples remain floating-point rather than being converted to 16 bits.
        for (size_t i = 0; i < frames; ++i)
            main_output[i] = std::clamp(main_output[i], -1.0f, 1.0f);
    } else if (p.model_id == SMN_MODEL_PARTICLE_MAIN ||
               p.model_id == SMN_MODEL_PARTICLE_AUX) {
        // Fixed block, no native Trigger; preserve MAIN filtering/FX/limiting
        // and clipping, while AUX suspends every MAIN history and limiter.
        voice.particle.RenderContinuous(engine_parameters, frequency,
            main_output, output, &voice.particle_limiter);
    } else if (p.model_id == SMN_MODEL_STRING_MAIN) {
        voice.string.RenderContinuousMain(frequency, p.harmonics, p.timbre,
                                          p.morph, main_output);
        voice.string_limiter.Process(1.0f, main_output, frames);
        for (size_t i = 0; i < frames; ++i)
            main_output[i] = std::clamp(main_output[i], -1.0f, 1.0f);
    } else if (p.model_id == SMN_MODEL_MODAL_MAIN ||
               p.model_id == SMN_MODEL_MODAL_AUX) {
        // Plaits' default accent with no level input; no global envelope/LPG.
        engine_parameters.accent = 0.8f;
        if (output == plaits::ENGINE_OUTPUT_MAIN && voice.modal_aux_selected) {
            // Clear old vibrations when returning to MAIN, including from another
            // model family. Keep the noise seed and excitation/control history.
            voice.modal.ResetResonator();
            voice.modal_limiter.Init();
        }
        voice.modal_aux_selected = output == plaits::ENGINE_OUTPUT_AUX;
        voice.modal.RenderAtFrequency(engine_parameters, frequency,
                                     main_output, voice.aux, frames, output);
        if (output == plaits::ENGINE_OUTPUT_MAIN) {
            voice.modal_limiter.Process(1.0f, main_output, frames);
            for (size_t i = 0; i < frames; ++i)
                main_output[i] = std::clamp(main_output[i], -1.0f, 1.0f);
        } else {
            // Clip at the Plaits AUX threshold, accounting for the 0.8 gain
            // that the mixer applies later.
            for (size_t i = 0; i < frames; ++i)
                main_output[i] = std::clamp(main_output[i], -1.25f, 1.25f);
        }
    } else if (p.model_id == SMN_MODEL_FORMANT_MAIN ||
               p.model_id == SMN_MODEL_FORMANT_AUX) {
        voice.formant.RenderAtFrequency(engine_parameters, frequency,
                                       main_output, voice.aux, frames, output);
        // Account for the mixer's 0.7/0.6 gain when clipping at Plaits'
        // full-scale threshold. Keep floating-point samples; the Plaits
        // global LPG and envelope are not used.
        const float bound = output == plaits::ENGINE_OUTPUT_MAIN
            ? 1.0f / 0.7f : 1.0f / 0.6f;
        for (size_t i = 0; i < frames; ++i)
            main_output[i] = std::clamp(main_output[i], -bound, bound);
    } else if (p.model_id == SMN_MODEL_SPEECH_MAIN ||
               p.model_id == SMN_MODEL_SPEECH_AUX) {
        voice.speech.RenderAtFrequency(engine_parameters, frequency,
                                       main_output, voice.aux, frames, output);
        // Speech MAIN requests the Plaits limiter; AUX has fixed gain 0.8.
        if (output == plaits::ENGINE_OUTPUT_MAIN) {
            voice.speech_limiter.Process(0.7f, main_output, frames);
            for (size_t i = 0; i < frames; ++i)
                main_output[i] = std::clamp(main_output[i], -1.0f, 1.0f);
        } else {
            for (size_t i = 0; i < frames; ++i)
                main_output[i] = std::clamp(main_output[i], -1.25f, 1.25f);
        }
    } else if (p.model_id == SMN_MODEL_SWARM_MAIN ||
               p.model_id == SMN_MODEL_SWARM_AUX) {
        voice.swarm.RenderAtFrequency(engine_parameters, frequency,
                                      main_output, voice.aux, frames, output);
        if (output == plaits::ENGINE_OUTPUT_MAIN)
            voice.swarm_limiter.Process(3.0f, main_output, frames);
        for (size_t i = 0; i < frames; ++i)
            main_output[i] = std::clamp(main_output[i], -1.0f, 1.0f);
    } else if (p.model_id == SMN_MODEL_WAVETABLE_MAIN ||
               p.model_id == SMN_MODEL_WAVETABLE_AUX) {
        voice.wavetable.RenderAtFrequency(engine_parameters, frequency,
                                         main_output, voice.aux, frames, output);
        // Preserve Plaits' clipping after gain 0.6, applied once by the mixer.
        for (size_t i = 0; i < frames; ++i)
            main_output[i] = std::clamp(main_output[i], -1.0f / 0.6f, 1.0f / 0.6f);
    } else if (p.model_id == SMN_MODEL_TERRAIN_MAIN ||
               p.model_id == SMN_MODEL_TERRAIN_AUX) {
        voice.terrain.RenderAtFrequency(engine_parameters, frequency,
                                       main_output, voice.aux, frames, output);
        // Plaits gain 0.7 is applied once by the existing mixer.
        for (size_t i = 0; i < frames; ++i)
            main_output[i] = std::clamp(main_output[i], -1.0f / 0.7f, 1.0f / 0.7f);
    } else if (p.model_id == SMN_MODEL_FM_MAIN ||
               p.model_id == SMN_MODEL_FM_AUX) {
        voice.fm.RenderAtFrequency(engine_parameters, frequency,
                                  main_output, voice.aux, frames, output);
        // Sines are bounded by +/-1 and the positive FIR weights sum to ~1.
        // The Plaits clipping threshold +/-1/0.6 is therefore unreachable.
        // Fixed gain 0.6 is still applied once by the mixer.
    } else {
        voice.engine.RenderAtFrequency(engine_parameters, frequency,
                                       main_output, voice.aux, frames, output);
    }
    return 1;
}

extern "C" int smn_synth_render(uint8_t voice,
    const SmnSynthParameters* parameters, float* output, size_t frames) {
    return smn_synth_render_with_input(voice, parameters, nullptr, output, frames);
}
