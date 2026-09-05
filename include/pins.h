#pragma once

// Hardware pin map — compile-time only (not changeable by commands).
// Pico 2 / Pico 2 W reuse the Pico / Pico W header map.
// RP2350 Mini (Waveshare RP2350-Zero) reuses the RP2040-Zero map.

#if defined(BOARD_PICO2) && !defined(BOARD_PICO)
#define BOARD_PICO 1
#endif
#if defined(BOARD_PICO2_W) && !defined(BOARD_PICO_W)
#define BOARD_PICO_W 1
#endif
#if defined(BOARD_RP2350_MINI) && !defined(BOARD_RP2040_ZERO)
#define BOARD_RP2040_ZERO 1
#endif

#if !defined(BOARD_PICO) && !defined(BOARD_PICO_W) && !defined(BOARD_RP2040_ZERO)
#define BOARD_PICO 1
#endif
#if (defined(BOARD_PICO) + defined(BOARD_PICO_W) + defined(BOARD_RP2040_ZERO)) > 1
#error "pins.h: define exactly one board (Pico / Pico W / Pico2 / Pico2 W / RP2040-Zero / RP2350 Mini)"
#endif

/* Oscilloscope HW debug pins (compile-time). Define DEBUG_HW to enable.
 * Off by default. On Pico, DEBUG_HW GPIOs overlap axis2 (GP10–13) and EXT
 * (GP14–15); runtime gates dbg_hw_* when those functions are claimed.
 * On Zero, DBG overlaps EXT / LIMIT3 (GP18, 21–23). */
/* #define DEBUG_HW 1 */

#define PIN_EXT_COUNT 4 /* only PIN_EXT_0…3 */
#define PIN_UART_SERIAL Serial1
#define UART_BAUD 115200

#if defined(BOARD_RP2040_ZERO)

/* Waveshare RP2040-Zero — see SliderDoc mc/pins.md / assets/img/rp2040zero_pinout_mc.png
 * axis 2 and 3 are supported; DBG (if DEBUG_HW) overlaps new EXT / LIMIT3. */
#define PIN_EXT_0 24
#define PIN_EXT_1 23
#define PIN_EXT_2 22
#define PIN_EXT_3 21

#define PIN_DRV_STEP 0
#define PIN_DRV_DIR 1
#define PIN_DRV_EN 2
#define PIN_DRV_ERROR 3
#define PIN_SW_LIMIT_L 4
#define PIN_SW_LIMIT_R 5

/* Optional 2nd STEP/DIR axis (axis>=2). */
#define PIN_DRV_STEP2 6
#define PIN_DRV_DIR2 7
#define PIN_DRV_EN2 11
#define PIN_DRV_ERROR2 8
#define PIN_SW_LIMIT_L2 9
#define PIN_SW_LIMIT_R2 10

/* Optional 3rd STEP/DIR axis (axis>=3). STEP polarity follows axis 2. */
#define PIN_DRV_STEP3 27
#define PIN_DRV_DIR3 26
#define PIN_DRV_EN3 15
#define PIN_DRV_ERROR3 14
#define PIN_SW_LIMIT_L3 18
#define PIN_SW_LIMIT_R3 17

#define PIN_CAMERA_CTRL 25

#define PIN_AXIS2_SUPPORTED 1
#define PIN_AXIS3_SUPPORTED 1
#define PIN_DBG_OVERLAPS_AXIS2 0
#define PIN_DBG_OVERLAPS_EXT 1
#define PIN_DBG_OVERLAPS_AXIS3 1

/* UART0 TX/RX on GP12/13 → Serial1 */
#define PIN_UART_TX 12
#define PIN_UART_RX 13

#define PIN_LED 29 /* external status LED; onboard WS2812 on GP16 unused */
#define PIN_BUZZER 28

#ifdef DEBUG_HW
#define PIN_DBG_FIFO 23
#define PIN_DBG_MOV 22
#define PIN_DBG_MOV_CONST 21
#define PIN_DBG_CMD 20
#define PIN_DBG_IRQ 19
#define PIN_DBG_UNDERRUN 18
#endif

#else /* BOARD_PICO / BOARD_PICO_W — classic Pico header map */

/* General-purpose extender outputs (logical on/off via Xn / Extn). */
#define PIN_EXT_0 8
#define PIN_EXT_1 9
#define PIN_EXT_2 14
#define PIN_EXT_3 15

#define PIN_DRV_STEP 18
#define PIN_DRV_DIR 19
#define PIN_DRV_EN 20
#define PIN_DRV_ERROR 21
#define PIN_SW_LIMIT_L 26
#define PIN_SW_LIMIT_R 27

/* Optional 2nd STEP/DIR axis (axis>=2). Overlaps DBG GP10–13. */
#define PIN_DRV_STEP2 13
#define PIN_DRV_DIR2 12
#define PIN_DRV_EN2 11
#define PIN_DRV_ERROR2 10
#define PIN_SW_LIMIT_L2 7
#define PIN_SW_LIMIT_R2 6

/* Optional 3rd STEP/DIR axis (axis>=3). STEP polarity follows axis 2. */
#define PIN_DRV_STEP3 5
#define PIN_DRV_DIR3 4
#define PIN_DRV_EN3 3
#define PIN_DRV_ERROR3 2
#define PIN_SW_LIMIT_L3 1
#define PIN_SW_LIMIT_R3 0

#define PIN_CAMERA_CTRL 22

#define PIN_AXIS2_SUPPORTED 1
#define PIN_AXIS3_SUPPORTED 1
#define PIN_DBG_OVERLAPS_AXIS2 1
#define PIN_DBG_OVERLAPS_EXT 1
#define PIN_DBG_OVERLAPS_AXIS3 0

// UART to UI controller (115200 baud).
// Available HW UART pins (RP2040 / earlephilhower Arduino core):
//   UART0 = Serial1  TX/RX:  0/1, 12/13, 16/17
//   UART1 = Serial2  TX/RX:  4/5,  8/9, 20/21, 24/25
// Current: GP16/GP17 → UART0 → Serial1 (keep PIN_UART_SERIAL in sync with the pair).
#define PIN_UART_TX 16
#define PIN_UART_RX 17

#define PIN_BUZZER 28

#if defined(BOARD_PICO_W)
/* External LED on GP28. Pico W onboard LED is CYW43 WL_GPIO0 (LED_BUILTIN=32);
 * earlephilhower does not support CYW43 access from FreeRTOS tasks (heartbeat
 * runs on proto). Use PlatformIO env picow or pico2w. */
#define PIN_LED 28
#else
/* Classic Pico onboard LED (GP25 via board package LED_BUILTIN). */
#if defined(HOST_TEST)
#define PIN_LED 25
#else
#define PIN_LED LED_BUILTIN
#endif
#endif

#ifdef DEBUG_HW
#define PIN_DBG_FIFO 10
#define PIN_DBG_MOV 11
#define PIN_DBG_MOV_CONST 12
#define PIN_DBG_CMD 13
#define PIN_DBG_IRQ 14
#define PIN_DBG_UNDERRUN 15
#endif

#endif /* BOARD_RP2040_ZERO */

#ifndef PIN_AXIS2_SUPPORTED
#define PIN_AXIS2_SUPPORTED 0
#endif
#ifndef PIN_AXIS3_SUPPORTED
#define PIN_AXIS3_SUPPORTED 0
#endif
#ifndef PIN_DBG_OVERLAPS_AXIS2
#define PIN_DBG_OVERLAPS_AXIS2 0
#endif
#ifndef PIN_DBG_OVERLAPS_AXIS3
#define PIN_DBG_OVERLAPS_AXIS3 0
#endif
#ifndef PIN_DBG_OVERLAPS_EXT
#define PIN_DBG_OVERLAPS_EXT 0
#endif
