#include "status_neopixel.h"
#include "pins.h"

#ifndef HOST_TEST

#include <Arduino.h>
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"

/*
 * 1-pixel WS2812 on pio1 (STEP stays on pio0). Bit timing is WS2812 800 kHz
 * (T1=2, T2=5, T3=3). Brightness 32/255 like SliderDMC.
 *
 * Wire order: Waveshare RP2040-Zero / RP2350-Zero onboard LED is RGB
 * (wiki “RGB LED” / WS2812B demo). Standard WS2812B and Adafruit NEO_GRB
 * send G,R,B — that swaps red and green on this pixel. External Pico pixels
 * stay GRB.
 */
#define NEOPIXEL_BRIGHTNESS 32u
#define NEOPIXEL_BIT_HZ 800000u
#define NEOPIXEL_CYCLES_PER_BIT 10u
#if defined(BOARD_RP2040_ZERO)
#define NEOPIXEL_WIRE_RGB 1
#else
#define NEOPIXEL_WIRE_RGB 0
#endif

static const uint16_t k_ws2812_instr[] = {
    0x6221, /* out x, 1     side 0 [2] */
    0x1123, /* jmp !x, 3    side 1 [1] */
    0x1400, /* jmp 0        side 1 [4] */
    0xa442, /* nop          side 0 [4] */
};

static const pio_program_t k_ws2812_prog = {
    .instructions = k_ws2812_instr,
    .length = 4,
    .origin = -1,
};

static PIO g_pio;
static int g_sm = -1;
static uint g_off;
static bool g_ok;
static uint8_t g_last_r = 255;
static uint8_t g_last_g = 255;
static uint8_t g_last_b = 255;

static uint8_t scale8(uint8_t c) {
  return (uint8_t)(((uint16_t)c * NEOPIXEL_BRIGHTNESS) >> 8);
}

void status_neopixel_begin(void) {
  g_ok = false;
  g_sm = -1;
  if (PIN_NEOPIXEL == PIN_LED) {
    return;
  }

  g_pio = pio1;
  g_sm = (int)pio_claim_unused_sm(g_pio, false);
  if (g_sm < 0) {
    return;
  }
  if (!pio_can_add_program(g_pio, &k_ws2812_prog)) {
    pio_sm_unclaim(g_pio, (uint)g_sm);
    g_sm = -1;
    return;
  }
  g_off = pio_add_program(g_pio, &k_ws2812_prog);

  const uint pin = (uint)PIN_NEOPIXEL;
  pio_gpio_init(g_pio, pin);
  pio_sm_set_consecutive_pindirs(g_pio, (uint)g_sm, pin, 1, true);

  pio_sm_config c = pio_get_default_sm_config();
  sm_config_set_wrap(&c, g_off, g_off + 3u);
  sm_config_set_sideset(&c, 1, false, false);
  sm_config_set_sideset_pins(&c, pin);
  sm_config_set_out_shift(&c, false, true, 24);
  sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
  float div = (float)clock_get_hz(clk_sys) /
              ((float)NEOPIXEL_BIT_HZ * (float)NEOPIXEL_CYCLES_PER_BIT);
  sm_config_set_clkdiv(&c, div);

  pio_sm_init(g_pio, (uint)g_sm, g_off, &c);
  pio_sm_set_enabled(g_pio, (uint)g_sm, true);
  g_ok = true;
  g_last_r = 255;
  g_last_g = 255;
  g_last_b = 255;
  status_neopixel_set(0, 0, 0);
}

void status_neopixel_set(uint8_t r, uint8_t g, uint8_t b) {
  if (!g_ok || g_sm < 0) {
    return;
  }
  if (r == g_last_r && g == g_last_g && b == g_last_b) {
    return;
  }
  g_last_r = r;
  g_last_g = g;
  g_last_b = b;
  uint8_t rs = scale8(r);
  uint8_t gs = scale8(g);
  uint8_t bs = scale8(b);
#if NEOPIXEL_WIRE_RGB
  uint32_t packed = ((uint32_t)rs << 16) | ((uint32_t)gs << 8) | (uint32_t)bs;
#else
  uint32_t packed = ((uint32_t)gs << 16) | ((uint32_t)rs << 8) | (uint32_t)bs;
#endif
  pio_sm_put_blocking(g_pio, (uint)g_sm, packed << 8);
}

#else /* HOST_TEST */

void status_neopixel_begin(void) {}
void status_neopixel_set(uint8_t r, uint8_t g, uint8_t b) {
  (void)r;
  (void)g;
  (void)b;
}

#endif
