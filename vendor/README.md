# Adapted synthesis sources

This directory contains the Plaits and stmlib subset used by SMN beta 0.1.
The selected engines are Waveshaping, Virtual Analog VCF, Virtual Analog,
Phase Distortion, Additive, Noise, Modal, Formant, Speech, Swarm, Wavetable,
Terrain, FM, String and Particle. Original file notices are preserved.

[SOURCES.json](SOURCES.json) records each imported file, its origin, original
and adapted SHA-256 hashes, and local changes. Plaits origins refer to the
local read-only reference tree used during development; that reference is not
required to build SMN. The upstream projects are
[Plaits](https://github.com/pichenettes/eurorack/tree/master/plaits) and
[stmlib](https://github.com/pichenettes/stmlib), with stmlib imported at
`d18def816c51d1da0c108236928b2bbd25c17481`.

The application Makefile lists its DSP sources explicitly and searches this
directory before the repository root. It does not compile the reference tree.
The C++17 adapter uses 44.1 kHz, direct normalized-frequency inputs and the
selected MAIN or AUX output. Constant tables are shared; mutable engine state
is private to each of six voices. No heap allocation or storage access occurs
in the audio render path.

Some adaptations intentionally change the original DSP: reduced Modal modes,
Particle diffusion at half rate, and the Terrain AUX midpoint policy. Others
avoid unused calculations or preserve the selected output through exact
precomputation. Suspension of an unselected oscillator is engine-specific.
See the [public README](../README.md#5-synthesis-models) for the audible
policies and limits, and the manifest for file-level details. Host comparisons
do not establish a CPU margin or electrical behaviour on the STM32.

The copies retain their MIT notices. See [LICENSE](../LICENSE) and
[THIRD_PARTY_LICENSES.md](../THIRD_PARTY_LICENSES.md) for SMN and component
licences. The separate support library in `bootloader/stmlib/` belongs to the
audio receiver and is not replaced by this directory.
