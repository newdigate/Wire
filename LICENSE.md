# License

This repository contains code under two licenses, per file:

## MIMXRT1176 port — MIT

`WireIMXRT1176.h` and `WireIMXRT1176.cpp` are Copyright (c) 2026 Nicholas
Newdigate and licensed under the **MIT License** (full text in each file
header). They implement the documented Arduino Wire API over the RT1176
LPI2C hardware, developed from the NXP MCUXpresso SDK (BSD-3-Clause).

A build for `__IMXRT1176__` compiles **only** the MIT files: `Wire.h` merely
dispatches to `WireIMXRT1176.h` for this platform, and `Wire.cpp` /
`utility/twi.*` are entirely `__AVR__`-guarded.

## Upstream Arduino/Teensy platforms — LGPL-2.1 (and per-file headers)

`Wire.h`, `Wire.cpp`, and `utility/twi.*` (the original Arduino TwoWire /
AVR twi code, Copyright (c) 2006 Nicholas Zambetti et al.) are LGPL-2.1-or-
later per their headers. `WireKinetis.*` / `WireIMXRT.*` carry their own
PJRC headers. Builds for those platforms are governed by those licenses.
