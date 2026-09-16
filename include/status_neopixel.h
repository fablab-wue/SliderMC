#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Claim pio1 for a single WS2812 (GRB, 800 kHz). No-op when PIN_NEOPIXEL
 * aliases PIN_LED so the classic GPIO LED is left intact.
 */
void status_neopixel_begin(void);

/** Set pixel RGB (0–255). Brightness scaled like Adafruit setBrightness(32). Skips PIO if unchanged. */
void status_neopixel_set(uint8_t r, uint8_t g, uint8_t b);

#ifdef __cplusplus
}
#endif
