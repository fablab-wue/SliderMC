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
 * Off by default. On Pico, DEBUG_HW GPIOs overlap LIMIT3 (GP10–11),
 * DRV_ERROR_1..3 (GP12–14), and DRV_ENABLE (GP15); runtime gates dbg_hw_*
 * because those pins are always claimed.
 * On Zero, DBG overlaps EXT_2..4 (GP18–20). */
/* #define DEBUG_HW 1 */

#define PIN_EXT_COUNT 4 /* PIN_EXT_1…4 */
#define PIN_UART_SERIAL Serial1
#define UART_BAUD 115200

#if defined(BOARD_RP2040_ZERO)

/* Waveshare RP2040-Zero — see SliderDoc mc/pins.md / assets/img/rp2040zero_pinout_mc.png
 * axis 2 and 3 are supported; DBG (if DEBUG_HW) overlaps EXT_2..4. */
#define PIN_EXT_1 17
#define PIN_EXT_2 18
#define PIN_EXT_3 19
#define PIN_EXT_4 20

#define PIN_DRV_STEP_1 1
#define PIN_DRV_DIR_1 2
#define PIN_SW_LIMIT_L_1 3
#define PIN_SW_LIMIT_R_1 4

/* Optional 2nd STEP/DIR axis (axis>=2). */
#define PIN_DRV_STEP_2 5
#define PIN_DRV_DIR_2 6
#define PIN_SW_LIMIT_L_2 7
#define PIN_SW_LIMIT_R_2 8

/* Optional 3rd STEP/DIR axis (axis>=3). STEP polarity follows axis 2. */
#define PIN_DRV_STEP_3 27
#define PIN_DRV_DIR_3 26
#define PIN_SW_LIMIT_L_3 15
#define PIN_SW_LIMIT_R_3 14

#define PIN_DRV_ERROR_1 9
#define PIN_DRV_ERROR_2 10
#define PIN_DRV_ERROR_3 11
#define PIN_DRV_ENABLE 0

#define PIN_CAMERA_CTRL 25

#define PIN_SERVO_1 21
#define PIN_SERVO_2 22
#define PIN_SERVO_3 23
#define PIN_SERVO3_STEALS_EXT4 0

#define PIN_AXIS2_SUPPORTED 1
#define PIN_AXIS3_SUPPORTED 1
#define PIN_DBG_OVERLAPS_AXIS2 0
#define PIN_DBG_OVERLAPS_EXT 1
#define PIN_DBG_OVERLAPS_AXIS3 0
#define PIN_DBG_OVERLAPS_DRV 0

/* UART0 TX/RX on GP12/13 → Serial1 */
#define PIN_UART_TX 12
#define PIN_UART_RX 13

#ifndef PIN_NEOPIXEL
#define PIN_NEOPIXEL 16 /* onboard WS2812 status */
#endif
#define PIN_LED 29 /* external status LED (blinks in parallel with NeoPixel) */
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

/* General-purpose extender outputs (logical on/off via EOn). */
#define PIN_EXT_1 21
#define PIN_EXT_2 20
#define PIN_EXT_3 19
#define PIN_EXT_4 18

#define PIN_DRV_STEP_1 0
#define PIN_DRV_DIR_1 1
#define PIN_SW_LIMIT_L_1 2
#define PIN_SW_LIMIT_R_1 3

/* Optional 2nd STEP/DIR axis (axis>=2). */
#define PIN_DRV_STEP_2 4
#define PIN_DRV_DIR_2 5
#define PIN_SW_LIMIT_L_2 6
#define PIN_SW_LIMIT_R_2 7

/* Optional 3rd STEP/DIR axis (axis>=3). STEP polarity follows axis 2. */
#define PIN_DRV_STEP_3 8
#define PIN_DRV_DIR_3 9
#define PIN_SW_LIMIT_L_3 10
#define PIN_SW_LIMIT_R_3 11

#define PIN_DRV_ERROR_1 12
#define PIN_DRV_ERROR_2 13
#define PIN_DRV_ERROR_3 14
#define PIN_DRV_ENABLE 15

#define PIN_CAMERA_CTRL 22

#define PIN_SERVO_1 26
#define PIN_SERVO_2 27
#define PIN_SERVO_3 18 /* steals EXT_4 when servos>=3 */
#define PIN_SERVO3_STEALS_EXT4 1

#define PIN_AXIS2_SUPPORTED 1
#define PIN_AXIS3_SUPPORTED 1
#define PIN_DBG_OVERLAPS_AXIS2 0
#define PIN_DBG_OVERLAPS_EXT 0
#define PIN_DBG_OVERLAPS_AXIS3 1
#define PIN_DBG_OVERLAPS_DRV 1

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
/* PIN_NEOPIXEL defaults to PIN_LED (GPIO LED only) unless overridden below. */

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
#ifndef PIN_DBG_OVERLAPS_DRV
#define PIN_DBG_OVERLAPS_DRV 0
#endif
#ifndef PIN_NEOPIXEL
#define PIN_NEOPIXEL PIN_LED /* Pico: GPIO LED only unless overridden to a free GPIO */
#endif
