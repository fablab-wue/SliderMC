#pragma once

// Hardware pin map — compile-time only (not changeable by commands).
// Select exactly one board: BOARD_PICO (default), BOARD_PICO_W, or BOARD_RP2040_ZERO.

#if !defined(BOARD_PICO) && !defined(BOARD_PICO_W) && !defined(BOARD_RP2040_ZERO)
#define BOARD_PICO 1
#endif
#if (defined(BOARD_PICO) + defined(BOARD_PICO_W) + defined(BOARD_RP2040_ZERO)) > 1
#error "pins.h: define exactly one of BOARD_PICO, BOARD_PICO_W, BOARD_RP2040_ZERO"
#endif

/* Oscilloscope HW debug pins (compile-time). Comment out DEBUG_HW to disable. */
#define DEBUG_HW 1

#define PIN_EXT_COUNT 10
#define PIN_UART_SERIAL Serial1
#define UART_BAUD 1000000

#if defined(BOARD_RP2040_ZERO)

/* Waveshare RP2040-Zero — see docs/PINS.md / docs/img/rp2040zero_pinout_mc.png */
#define PIN_EXT_0 0
#define PIN_EXT_1 1
#define PIN_EXT_2 2
#define PIN_EXT_3 3
#define PIN_EXT_4 4
#define PIN_EXT_5 5
#define PIN_EXT_6 6
#define PIN_EXT_7 7
#define PIN_EXT_8 8
#define PIN_EXT_9 17

#define PIN_SW_HOME 9
#define PIN_SW_LIMIT_L 10
#define PIN_SW_LIMIT_R 11

/* UART0 TX/RX on GP12/13 → Serial1 */
#define PIN_UART_TX 12
#define PIN_UART_RX 13

#define PIN_LED 14 /* external status LED; onboard WS2812 on GP16 unused */

#define PIN_DRV_STEP 26
#define PIN_DRV_DIR 27
#define PIN_DRV_EN 28
#define PIN_DRV_ERROR 29

#ifdef DEBUG_HW
#define PIN_DBG_FIFO 20
#define PIN_DBG_MOV 21
#define PIN_DBG_MOV_CONST 22
#define PIN_DBG_CMD 23
#define PIN_DBG_IRQ 24
#define PIN_DBG_UNDERRUN 25
#endif

#else /* BOARD_PICO / BOARD_PICO_W — classic Pico header map */

/* General-purpose extender outputs (logical on/off via Xn / Extn). */
#define PIN_EXT_0 0
#define PIN_EXT_1 1
#define PIN_EXT_2 2
#define PIN_EXT_3 3
#define PIN_EXT_4 4
#define PIN_EXT_5 5
#define PIN_EXT_6 6
#define PIN_EXT_7 7
#define PIN_EXT_8 8
#define PIN_EXT_9 9

#define PIN_DRV_STEP 18
#define PIN_DRV_DIR 19
#define PIN_DRV_EN 20
#define PIN_DRV_ERROR 21
#define PIN_SW_HOME 22
#define PIN_SW_LIMIT_L 26
#define PIN_SW_LIMIT_R 27

// UART to UI controller (1 Mbaud).
// Available HW UART pins (RP2040 / earlephilhower Arduino core):
//   UART0 = Serial1  TX/RX:  0/1, 12/13, 16/17
//   UART1 = Serial2  TX/RX:  4/5,  8/9, 20/21, 24/25
// Current: GP16/GP17 → UART0 → Serial1 (keep PIN_UART_SERIAL in sync with the pair).
#define PIN_UART_TX 16
#define PIN_UART_RX 17

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
