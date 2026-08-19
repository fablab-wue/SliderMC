# SliderMC

C++ / FreeRTOS motor controller for the Raspberry Pi Pico (STEP/DIR axis).

This repository is independent of the MicroPython UI project (`SliderCtrl`).

> IMPORTANT: Docs and manuals moved to repository **SliderDOC** - https://github.com/fablab-wue/SliderDoc

## Features

- Retarget during movement
- Sine ramp acceleration / deceleration
- **API units:** mm, mm/s, mm/s²  
- **Links:** USB CDC (debug + CLI) and UART @ 1 Mbaud (UI controller)
- 10 optional output pins (port extentder)

### Step timing limits (theoretical)

- Maximum step frequency: ≈259kHz (≈800mm/s @320P/mm)
- Minimum step frequency: ≈0.75Hz (≈2.3µm/s ≈0.14mm/min ≈8mm/h @320P/mm)
- Step timing precision 20ns by PIO

## Quick start (VS Code)

1. Install [VS Code](https://code.visualstudio.com/) and the **PlatformIO IDE** extension.
2. **File → Open Folder** → this repository (`SliderMC`), not SliderCtrl.
3. PlatformIO: **Build** / **Upload** (env `pico`).
4. Open the serial monitor for the USB CLI.

Details: [docs/BUILD.md](docs/BUILD.md). On Windows, if Upload fails with `Please specify upload_port`, install the WinUSB driver for **RP2 Boot (Interface 1)** via Zadig — see [BUILD.md](docs/BUILD.md#windows-upload-picotool--upload_port-error).

## Repository layout

```text
include/     Public headers (pins, protocol, planner, PIO)
src/board/   GPIO, UART, USB CDC, LittleFS port
src/motion/  PIO STEP, planner, FreeRTOS tasks (+ host stub)
src/protocol ASCII parser, commands, verbose/status
src/config/  RAM config store
data/mc.ini  Factory defaults
test/host/   Host-side protocol tests
scripts/     Host tests + GitHub repo helper
```

## Host protocol tests

Requires `g++` or `clang++` on PATH:

```powershell
powershell -File scripts/run_host_tests.ps1
```

## Status

| Area | State |
|------|--------|
| Protocol (S/G/I/M/C/W/V groups, `?`/`!`, verbose) | Implemented |
| FreeRTOS tasks (feed / plan / proto) + heartbeat/WDT | Implemented |
| PIO STEP + sine planner (live retarget, soft/hard stop) | Implemented |
| Soft/hard limits, homing, EMO/`DRV_ERROR` | Implemented |
| LittleFS persistence | Load `/mc.ini` at boot; save on `CS` |

---

## License

Copyright (c) 2026 Jochen Krapf \<jk@nerd2nerd.org\>

Licensed under the [MIT License](LICENSE).

Company names and product names mentioned in this project are trademarks or registered trademarks of their respective owners. Use here is for identification only.
