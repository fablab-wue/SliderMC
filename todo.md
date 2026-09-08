# Next steps after `axis3`

Recommendations deferred from the experimental 3rd-axis plan. Stay on Pico / Zero at 125 MHz until a new `ID` after the fill fix still shows remaining underruns.

## Measure first

- [x] **Per-axis `ID` diagnostics.** `ID` is now `underrun=u0,u1,u2 … fifo_min=f0,f1,f2`. Live `D:underrun` is rate-limited (`n=` burst count).
- [x] **Bench 3-axis ramps** (`ss100 sa200 mt 234 234 321`). Matches the 2500 Hz n=1 hotspot: a=2 dries first on accel (faster follower, always filled last), cruise is mostly clean, a=0/1 storm again on the brake crawl. Firmware fill fix is in (prefill before SM start, round-robin, 2 words/pass on 3-axis, pack from 1000 Hz on 3-axis, skip n=1 refinement). **Re-flash and repeat the same MT; read `ID`.**

## Only if underruns remain

## Only if underruns remain

- [x] **133 MHz in-spec clock.** `board_build.f_cpu` + `set_sys_clock_khz(133000)`. PIO clkdiv = `clk_sys / 50e6` (~2.66). `pio_step_sysclk_hz()` is SM clock from `clock_get_hz(clk_sys) / clkdiv`.
- [ ] **Dual-core feed / DMA word queues / full DDA interpolator** (plan suggestion 8). Do not start these unless `ID` still underruns after fill/path and optional 133 MHz. Pairwise `|d_n|/|d0|` cruise scaling is enough for experimental MT; a follower `max_speed` clamp can still desync arrival. Path mode stays the true shared clock.
- [x] **Pico 2 / Pico 2 W / RP2350 Mini PlatformIO envs** (`pico2`, `pico2w`, `rp2350mini`). 150 MHz CPU; PIO clkdiv from `clock_get_hz(clk_sys)`. Pico 2 reuses Pico pins; Mini reuses Zero pins. Bench 3-axis underruns on hardware still TBD.

## Firmware cleanup

- [ ] **`McAxis cfg[3]`** (plan suggestion 9). Replace `_2` / `_3` `McConfig` mirrors with an axis array. Experimental branch kept the copy-paste pattern on purpose.
- [x] **Stale rate comments.** `pio_step.h` already states max ≈ 259 kHz. Host `max hz >= 300k` still uses a 125 MHz `planner_max_step_hz` argument (not the 50 MHz SM clock).
- [ ] **Do not move axis 3 onto `pio1`** unless polarity-share ever fails (extra IRQ path). Sharing by STEP polarity is enough: worst case axis1=0 and axis2=1 already occupies both 14-word slots; axis 3 reuses axis 2’s program (`DRV_STEP_3_active` stays absent).

## Protocol / clients / docs (explicitly out of scope on `axis3`)

- [ ] **`CAMERA_CTRL` command.** Pin is reserved and inited inactive (`IX`/`VG` may list it). No protocol yet — add with docs and clients.
- [ ] **SliderDoc** pinouts, protocol tables, cheatsheets, architecture notes for axis 3 and the EXT / `CAMERA_CTRL` remap.
- [ ] **SliderCtrl / SliderWeb / SliderHost / UIC** — `MC_Client` third argument on `MT`/`M`/`MJ`/`SP`/`SL`/`SR`/`PD`, `IA:3`, `CG axis`, and verbose `#I p1 | p2 | p3`. Clients will not send a 3rd token until this pass.
- [ ] Config key rename already shipped in firmware (`SW_LIMIT_R_3_use`, `DRV_STEP_1_active`, `home_mode_1`, …; no `axis2_use` / `SW_HOME_*`). Update clients and docs to the new names; no firmware aliases.
