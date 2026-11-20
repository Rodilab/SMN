# Third-party licences and notices

SMN-specific contributions by Rodrigo Diaz are licensed under the MIT License
in [LICENSE](LICENSE). This document records the separate notices for code and
resources included in the public firmware source distribution: `src/`, `inc/`,
`vendor/`, `stm32/`, `stm_audio_bootloader/` and `bootloader/`.
Third-party copyrights remain with their respective owners. The SMN licence
does not replace file-specific notices or relicense third-party material.
Preserve those notices when redistributing the corresponding sources.

## 1. Original SWN firmware — MIT

SMN is derived from the SWN firmware by Dan Green / 4ms Company. The original
software attribution is retained below; original author and permission notices
also remain in the source files.

Copyright (c) 2017 Dan Green (danngreen1@gmail.com)

The following MIT permission and disclaimer apply to the SWN software and to
the MIT-licensed components listed in section 2, under their respective
copyright notices:

```text
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

## 2. Plaits, stmlib and audio bootloader — MIT

SMN incorporates DSP code and tables from Plaits and stmlib by Émilie Gillet /
Mutable Instruments. The imported files use the spelling **Emilie Gillet** in
their copyright notices. Older stmlib and STM audio bootloader sources retain
the original name **Olivier Gillet** as printed in those sources.

The following notices apply to the respective files, under the full MIT
permission and warranty text in section 1:

- Copyright 2012 Emilie Gillet.
- Copyright 2012 Olivier Gillet.
- Copyright 2013 Olivier Gillet.
- Copyright 2014 Emilie Gillet.
- Copyright 2015 Emilie Gillet.
- Copyright 2016 Emilie Gillet.
- Copyright 2017 Emilie Gillet.
- Copyright 2021 Emilie Gillet.

SMN's adapted DSP copies are in `vendor/plaits/` and `vendor/stmlib/`. [vendor/SOURCES.json](vendor/SOURCES.json)
records the imported files, origins, hashes and local adaptations. The software
licence covers the imported code and resources according to their notices; it
is not a licence to unrelated Mutable Instruments hardware designs or branding.

The audio-update implementation and tools are in `stm_audio_bootloader/` and
`bootloader/stm_audio_bootloader/`, with older stmlib support in
`bootloader/stmlib/`. Their MIT notices are retained. Full upstream notices are
also available in [stm_audio_bootloader/LICENSE](stm_audio_bootloader/LICENSE)
and [bootloader/stm_audio_bootloader/LICENSE](bootloader/stm_audio_bootloader/LICENSE).

### CRC32 notice

`bootloader/stmlib/utils/crc32.h` carries this separate permission:

```text
COPYRIGHT (C) 1986 Gary S. Brown.  You may use this program, or
code or tables extracted from it, as desired without restriction.
```

## 3. ARM CMSIS and STMicroelectronics support code

The `stm32/` support sources are not relicensed as MIT. They contain BSD-style
three-clause notices, including source and binary redistribution conditions
and a prohibition on using the authors' names to imply endorsement. Preserve
the exact copyright years and notices in each file. Representative full notices
from the included versions follow.

### ARM CMSIS core

```text
Copyright (c) 2009 - 2015 ARM LIMITED

   All rights reserved.
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
   - Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   - Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   - Neither the name of ARM nor the names of its contributors may be used
     to endorse or promote products derived from this software without
     specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
```

### ARM CMSIS DSP

```text
Copyright (C) 2010-2015 ARM Limited. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:
  - Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  - Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in
    the documentation and/or other materials provided with the
    distribution.
  - Neither the name of ARM LIMITED nor the names of its contributors
    may be used to endorse or promote products derived from this
    software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
```

### STMicroelectronics

Copyright notices in the included device, HAL and configuration sources include
2016, 2017 and 2018. The exact notice in each source file remains applicable.

```text
Copyright (c) 2017 STMicroelectronics

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
  1. Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.
  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation
     and/or other materials provided with the distribution.
  3. Neither the name of STMicroelectronics nor the names of its contributors
     may be used to endorse or promote products derived from this software
     without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```

## 4. Original SWN hardware and branding

The original SWN hardware notice is:

```text
Copyright (c) 2017 Dan Green (danngreen1@gmail.com)
```

The original SWN hardware design is licensed separately under **Creative Commons
Attribution-NonCommercial-ShareAlike 4.0 International (CC BY-NC-SA 4.0)**.
This hardware licence does not apply to SMN's MIT-licensed software.

Hardware reuse under that licence requires attribution, an indication of
changes, non-commercial use and sharing adaptations under the same licence,
as detailed in the [licence deed](https://creativecommons.org/licenses/by-nc-sa/4.0/)
and [full legal code](https://creativecommons.org/licenses/by-nc-sa/4.0/legalcode.en).
This section records the upstream hardware terms; SMN does not grant new rights
to hardware designs owned by others.

The original SWN derivative-work guidelines ask that derivative hardware
(faceplates, enclosures, PCBs and front panels) not carry the text “4ms”,
“4ms Company” or the graphic 4ms logo. Required software attribution remains
appropriate. MIT permission does not itself grant trademark rights or imply
endorsement by 4ms, Mutable Instruments, ARM or STMicroelectronics.

## 5. Redistribution

Include [LICENSE](LICENSE) and this document with source and compiled firmware
packages, including BIN, HEX and audio-update WAV distributions. Preserve the
licences and copyright notices in the component directories and source files.
The ARM and ST terms above require their notices, conditions and disclaimers
to accompany binary redistributions as well.

The software MIT licence permits commercial use subject to its notice
condition. It does not grant rights to third-party trademarks or hardware.

These notices cover the components listed above. If adding other tools or
sources to a distribution, include their applicable licences as well.
