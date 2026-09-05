#pragma once

#include "pins.h"
#include "config_store.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Number of active STEP/DIR axes (1..3). */
static inline int axis_hw_count(void) { return config_axis_count(); }

static inline int axis_hw_step_pin(int axis) {
#if PIN_AXIS3_SUPPORTED
  if (axis == 2) {
    return PIN_DRV_STEP3;
  }
#endif
#if PIN_AXIS2_SUPPORTED
  if (axis == 1) {
    return PIN_DRV_STEP2;
  }
#endif
  (void)axis;
  return PIN_DRV_STEP;
}

static inline int axis_hw_dir_pin(int axis) {
#if PIN_AXIS3_SUPPORTED
  if (axis == 2) {
    return PIN_DRV_DIR3;
  }
#endif
#if PIN_AXIS2_SUPPORTED
  if (axis == 1) {
    return PIN_DRV_DIR2;
  }
#endif
  (void)axis;
  return PIN_DRV_DIR;
}

static inline int axis_hw_en_pin(int axis) {
#if PIN_AXIS3_SUPPORTED
  if (axis == 2) {
    return PIN_DRV_EN3;
  }
#endif
#if PIN_AXIS2_SUPPORTED
  if (axis == 1) {
    return PIN_DRV_EN2;
  }
#endif
  (void)axis;
  return PIN_DRV_EN;
}

static inline int axis_hw_error_pin(int axis) {
#if PIN_AXIS3_SUPPORTED
  if (axis == 2) {
    return PIN_DRV_ERROR3;
  }
#endif
#if PIN_AXIS2_SUPPORTED
  if (axis == 1) {
    return PIN_DRV_ERROR2;
  }
#endif
  (void)axis;
  return PIN_DRV_ERROR;
}

static inline int axis_hw_limit_l_pin(int axis) {
#if PIN_AXIS3_SUPPORTED
  if (axis == 2) {
    return PIN_SW_LIMIT_L3;
  }
#endif
#if PIN_AXIS2_SUPPORTED
  if (axis == 1) {
    return PIN_SW_LIMIT_L2;
  }
#endif
  (void)axis;
  return PIN_SW_LIMIT_L;
}

static inline int axis_hw_limit_r_pin(int axis) {
#if PIN_AXIS3_SUPPORTED
  if (axis == 2) {
    return PIN_SW_LIMIT_R3;
  }
#endif
#if PIN_AXIS2_SUPPORTED
  if (axis == 1) {
    return PIN_SW_LIMIT_R2;
  }
#endif
  (void)axis;
  return PIN_SW_LIMIT_R;
}

static inline int axis_hw_step_active(int axis) {
  const McConfig *c = config_get();
  /* Axis 3 STEP polarity follows axis 2. */
  return (axis >= 1) ? c->drv_step_active_2 : c->drv_step_active;
}

static inline int axis_hw_dir_active(int axis) {
  const McConfig *c = config_get();
  if (axis == 2) {
    return c->drv_dir_active_3;
  }
  return (axis == 1) ? c->drv_dir_active_2 : c->drv_dir_active;
}

static inline int axis_hw_en_active(int axis) {
  const McConfig *c = config_get();
  if (axis == 2) {
    return c->drv_en_active_3;
  }
  return (axis == 1) ? c->drv_en_active_2 : c->drv_en_active;
}

static inline int axis_hw_error_active(int axis) {
  const McConfig *c = config_get();
  if (axis == 2) {
    return c->drv_error_active_3;
  }
  return (axis == 1) ? c->drv_error_active_2 : c->drv_error_active;
}

static inline int axis_hw_limit_l_active(int axis) {
  const McConfig *c = config_get();
  if (axis == 2) {
    return c->sw_limit_l_active_3;
  }
  return (axis == 1) ? c->sw_limit_l_active_2 : c->sw_limit_l_active;
}

static inline int axis_hw_limit_r_active(int axis) {
  const McConfig *c = config_get();
  if (axis == 2) {
    return c->sw_limit_r_active_3;
  }
  return (axis == 1) ? c->sw_limit_r_active_2 : c->sw_limit_r_active;
}

static inline int axis_hw_limit_l_use(int axis) {
  const McConfig *c = config_get();
  if (axis == 2) {
    return c->sw_limit_l_use_3;
  }
  return (axis == 1) ? c->sw_limit_l_use_2 : c->sw_limit_l_use;
}

static inline int axis_hw_limit_r_use(int axis) {
  const McConfig *c = config_get();
  if (axis == 2) {
    return c->sw_limit_r_use_3;
  }
  return (axis == 1) ? c->sw_limit_r_use_2 : c->sw_limit_r_use;
}

static inline float axis_hw_steps_per_unit(int axis) {
  const McConfig *c = config_get();
  if (axis == 2) {
    return c->steps_per_unit_3;
  }
  return (axis == 1) ? c->steps_per_unit_2 : c->steps_per_unit;
}

static inline float axis_hw_slider_min(int axis) {
  const McConfig *c = config_get();
  if (axis == 2) {
    return c->slider_min_mm_3;
  }
  return (axis == 1) ? c->slider_min_mm_2 : c->slider_min_mm;
}

static inline float axis_hw_slider_max(int axis) {
  const McConfig *c = config_get();
  if (axis == 2) {
    return c->slider_max_mm_3;
  }
  return (axis == 1) ? c->slider_max_mm_2 : c->slider_max_mm;
}

/** Effective working-window min (session, else envelope). Homing uses axis_hw_slider_min. */
static inline float axis_hw_window_min(int axis) {
  return session_effective_left(axis);
}

/** Effective working-window max (session, else envelope). */
static inline float axis_hw_window_max(int axis) {
  return session_effective_right(axis);
}

static inline int axis_hw_home_mode(int axis) {
  const McConfig *c = config_get();
  if (axis == 2) {
    return c->home_mode_3;
  }
  return (axis == 1) ? c->home_mode_2 : c->home_mode;
}

static inline float axis_hw_home_move_out(int axis) {
  const McConfig *c = config_get();
  if (axis == 2) {
    return c->home_move_out_mm_3;
  }
  return (axis == 1) ? c->home_move_out_mm_2 : c->home_move_out_mm;
}

static inline float axis_hw_home_speed(int axis) {
  const McConfig *c = config_get();
  if (axis == 2) {
    return c->home_speed_mm_s_3;
  }
  return (axis == 1) ? c->home_speed_mm_s_2 : c->home_speed_mm_s;
}

static inline float axis_hw_home_accel(int axis) {
  const McConfig *c = config_get();
  if (axis == 2) {
    return c->home_accel_mm_s2_3;
  }
  return (axis == 1) ? c->home_accel_mm_s2_2 : c->home_accel_mm_s2;
}

static inline float axis_hw_max_speed(int axis) {
  const McConfig *c = config_get();
  if (axis == 2) {
    return c->max_speed_mm_s_3;
  }
  return (axis == 1) ? c->max_speed_mm_s_2 : c->max_speed_mm_s;
}

static inline float axis_hw_max_accel(int axis) {
  const McConfig *c = config_get();
  if (axis == 2) {
    return c->max_accel_mm_s2_3;
  }
  return (axis == 1) ? c->max_accel_mm_s2_2 : c->max_accel_mm_s2;
}

#ifdef __cplusplus
}
#endif
