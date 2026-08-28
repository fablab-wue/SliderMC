#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void board_gpio_init(void);

/** Set extender output 0..5: on=true → active level from EXT_n_active. */
bool board_ext_set(int index, bool on);

/** Current logical on/off for extender output 0..5. */
bool board_ext_get(int index);

/** Pulse PIN_BUZZER high ~100 ms (no-op if BUZZER_use=0 or pin is PIN_LED). */
void board_buzzer_pulse(void);
/** Countdown active buzzer pulse; call every proto/heartbeat tick. */
void board_buzzer_tick(unsigned dt_ms);
/** Apply BUZZER_use (claim or release pin). */
void board_buzzer_reconfigure(void);

void board_uart_init(void);

/**
 * Block until UIC UART or USB CDC receives '\\n'. Discard all other bytes
 * (including '\\r') on both ports. Does not feed the protocol parser —
 * pre-handshake noise must not become commands. Calls board_heartbeat_tick
 * each ~5 ms so wait LED / WDT keep running.
 */
void board_wait_unlock_newline(void);

/** Non-blocking: pull RX bytes from UIC UART into protocol_feed_uart_byte. */
void board_uart_poll_rx(void);

/** Write to UIC UART only (never used for debug text). */
void board_uart_write(const char *data, size_t n);

/** Write to USB CDC (CLI replies + optional debug). */
void board_usb_write(const char *data, size_t n);

/** Write debug to USB CDC only. */
void board_usb_debug(const char *data, size_t n);

/** True if USB CDC host is connected (best-effort). */
bool board_usb_connected(void);

#ifdef __cplusplus
}
#endif
