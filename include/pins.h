#pragma once

// Hardware pin map — compile-time only (not changeable by commands).
// Select exactly one board: BOARD_PICO (default), BOARD_PICO_W, or BOARD_RP2040_ZERO.

#if !defined(BOARD_PICO) && !defined(BOARD_PICO_W) && !defined(BOARD_RP2040_ZERO)
#define BOARD_PICO 1
#endif
#if (defined(BOARD_PICO) + defined(BOARD_PICO_W) + defined(BOARD_RP2040_ZERO)) > 1
#error "pins.h: define exactly one of BOARD_PICO, BOARD_PICO_W, BOARD_RP2040_ZERO"
#endif

/* Oscilloscope HW debug pins (compile-time). Comment out DEBUG_HW to disable.
 * On Pico, DEBUG_HW GPIOs overlap axis2 (GP10–13); when axis2_use=1, runtime
 * gates dbg_hw_* so those pins are used for the second STEP/DIR axis instead.
 * On Zero, DBG and axis2 use disjoint GPIOs and may be active together. */
#define DEBUG_HW 1

#define PIN_EXT_COUNT 4 /* only PIN_EXT_0…3 */
#define PIN_UART_SERIAL Serial1
#define UART_BAUD 115200

#if defined(BOARD_RP2040_ZERO)

/* Waveshare RP2040-Zero — see SliderDoc mc/pins.md / assets/img/rp2040zero_pinout_mc.png
 * axis2_use is supported; DBG (GP18–23) does not overlap axis2. */
#define PIN_EXT_0 27
#define PIN_EXT_1 26
#define PIN_EXT_2 15
#define PIN_EXT_3 14

#define PIN_DRV_STEP 0
#define PIN_DRV_DIR 1
#define PIN_DRV_EN 2
#define PIN_DRV_ERROR 3
#define PIN_SW_HOME 4
#define PIN_SW_LIMIT_L 10
#define PIN_SW_LIMIT_R 9

/* Optional 2nd STEP/DIR axis (axis2_use=1). Disjoint from DBG. */
#define PIN_DRV_STEP2 5
#define PIN_DRV_DIR2 6
#define PIN_DRV_EN2 17
#define PIN_DRV_ERROR2 7
#define PIN_SW_HOME2 8
#define PIN_SW_LIMIT_L2 25
#define PIN_SW_LIMIT_R2 24

#define PIN_AXIS2_SUPPORTED 1
#define PIN_DBG_OVERLAPS_AXIS2 0

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
#define PIN_EXT_0 2
#define PIN_EXT_1 3
#define PIN_EXT_2 4
#define PIN_EXT_3 5

#define PIN_DRV_STEP 18
#define PIN_DRV_DIR 19
#define PIN_DRV_EN 20
#define PIN_DRV_ERROR 21
#define PIN_SW_HOME 22
#define PIN_SW_LIMIT_L 26
#define PIN_SW_LIMIT_R 27

/* Optional 2nd STEP/DIR axis (axis2_use=1). Overlaps DBG GP10–13. */
#define PIN_DRV_STEP2 13
#define PIN_DRV_DIR2 12
#define PIN_DRV_EN2 11
#define PIN_DRV_ERROR2 10
#define PIN_SW_HOME2 9
#define PIN_SW_LIMIT_L2 7
#define PIN_SW_LIMIT_R2 6
#define PIN_AXIS2_SUPPORTED 1
#define PIN_DBG_OVERLAPS_AXIS2 1

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
 * runs on proto). Use PlatformIO env picow (board=rpipicow, -DBOARD_PICO_W). */
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
#ifndef PIN_DBG_OVERLAPS_AXIS2
#define PIN_DBG_OVERLAPS_AXIS2 0
#endif
