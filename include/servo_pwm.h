#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Init / re-init hardware PWM slices for configured servos (100 Hz, wrap 65535). */
void servo_pwm_init(void);
void servo_pwm_reconfigure(void);

/** SE 0 stops PWM (limp). SE 1 restores the last pulse. */
void servo_pwm_set_enabled(bool on);
bool servo_pwm_enabled(void);

/** Write compare from angle (clamped to SERVO_N envelope). No-op if disabled or index unused. */
void servo_pwm_write_deg(int servo0, float deg);

/** Re-apply last pose to PWM after CS pulse/swap/envelope/active. */
void servo_pwm_refresh(void);

/**
 * Linear envelope → pulse µs (swap + clamp). Host-testable; no Pico PWM.
 * Invalid servo0 → 0.
 */
float servo_pwm_deg_to_us(int servo0, float deg);

#ifdef __cplusplus
}
#endif
