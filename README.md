# SliderMC

**Deterministic motion firmware for DIY motorized camera sliders** — C++ / FreeRTOS on Raspberry Pi Pico, STEP/DIR axis with PIO timing.

## About

**SliderMC** is the **motion controller** for the open **motorized camera slider** system. It owns everything timing-critical: STEP/DIR pulses, sine ramps, homing, soft and hard limits, and EMO / driver-error handling — so the UI panel ([SliderCtrl](https://github.com/fablab-wue/SliderCtrl)) can focus on knobs, buttons, and OLED without stealing motion cycles.

This is a **DIY project with pro-grade motion firmware**. Live retarget, mm-based API, and open protocol put you on par with many closed commercial controllers for **features and transparency** — paired with SliderCtrl for operator UX on set. Rail quality, motor choice, and mechanical build remain yours.

The same motion firmware is the **electronics half of the construction kit**: one STEP/DIR axis with homing and limits — linear slider rail, mini-dolly, rotating head, or turntable depending on your mechanics and `mc.ini` setup.

Docs and manuals: **[SliderDoc](https://github.com/fablab-wue/SliderDoc)**.

> Documentation: [SliderDoc](https://github.com/fablab-wue/SliderDoc)

---

## Features

**Open motion stack, set-ready behaviour.** Dedicated RP2040 runs the planner and PIO step engine; the UIC talks millimetres over UART at 1 Mbaud.

### Motion quality

- **Live retarget** during movement — change target or speed without stopping the shoot  
- **Sine ramp** acceleration and deceleration — smooth starts and stops  
- **Homing**, soft travel limits, hard-limit alarms, **`DRV_ERROR` / EMO** interlock  
- **Path playback** — host-authored motion paths for advanced moves  
- **Joystick hold (`MJ`)** — signed % of session speed, 1- or 2-axis  
- **API units:** mm, mm/s, mm/s² (steps handled internally)

### Maker / pro transparency

- **Open ASCII protocol** on UART + USB CDC CLI — bench-test and script without a panel  
- **Persistent config** (`mc.ini` on LittleFS) · host-side protocol tests in `test/host/`  
- **10 extender outputs** for accessories · UI load fully isolated on the UIC Pico  
- **Replaceable UIC** — same motion board, different panel firmware or fork  

### Step timing limits (theoretical)

Proof points for motion firmware depth:

- Maximum step frequency: ≈259 kHz (≈800 mm/s @ 320 P/mm)  
- Minimum step frequency: ≈0.75 Hz (≈2.3 µm/s @ 320 P/mm)  
- Step timing precision: ~20 ns via PIO  

Competitive context: [architecture/compare.md](https://github.com/fablab-wue/SliderDoc/blob/main/architecture/compare.md).

---

## Quick start (VS Code)

1. Install [VS Code](https://code.visualstudio.com/) and the **PlatformIO IDE** extension.  
2. **File → Open Folder** → this repository (`SliderMC`), not SliderCtrl.  
3. PlatformIO: **Build** / **Upload** (env `pico`).  
4. Open the serial monitor for the USB CLI (press Enter to unlock, then type commands).

Details: [mc/build.md](https://github.com/fablab-wue/SliderDoc/blob/main/mc/build.md). On Windows, if Upload fails with `Please specify upload_port`, install the WinUSB driver for **RP2 Boot (Interface 1)** via Zadig — see [Windows upload section](https://github.com/fablab-wue/SliderDoc/blob/main/mc/build.md#windows-upload-picotool--upload_port-error).

---

## Documentation (SliderDoc)

| Topic | Document |
|-------|----------|
| Build / flash | [mc/build.md](https://github.com/fablab-wue/SliderDoc/blob/main/mc/build.md) |
| Protocol | [contract/protocol.md](https://github.com/fablab-wue/SliderDoc/blob/main/contract/protocol.md) |
| Config keys | [mc/config.md](https://github.com/fablab-wue/SliderDoc/blob/main/mc/config.md) |
| GPIO map | [mc/pins.md](https://github.com/fablab-wue/SliderDoc/blob/main/mc/pins.md) |
| Motion / planner | [mc/motion.md](https://github.com/fablab-wue/SliderDoc/blob/main/mc/motion.md) |
| Joystick (`MJ`) | [mc/motion-joy.md](https://github.com/fablab-wue/SliderDoc/blob/main/mc/motion-joy.md) |
| Working window (`SL`/`SR`) | [mc/working-window.md](https://github.com/fablab-wue/SliderDoc/blob/main/mc/working-window.md) |
| UIC (panel) | [SliderCtrl](https://github.com/fablab-wue/SliderCtrl) · [uic/](https://github.com/fablab-wue/SliderDoc/blob/main/uic/README.md) |

---

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

---

## Host protocol tests

Requires `g++` or `clang++` on PATH:

```powershell
powershell -File scripts/run_host_tests.ps1
```

---

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
