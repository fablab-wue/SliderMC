#pragma once

#include "pins.h"
#include "config_store.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Number of active STEP/DIR axes: 1, or 2 when axis2 is enabled. */
static inline int axis_hw_count(void) { return config_axis2_enabled() ? 2 : 1; }

static inline int axis_hw_step_pin(int axis) {
#if PIN_AXIS2_SUPPORTED
  return (axis == 1) ? PIN_DRV_STEP2 : PIN_DRV_STEP;
#else
  (void)axis;
  return PIN_DRV_STEP;
#endif
}

static inline int axis_hw_dir_pin(int axis) {
#if PIN_AXIS2_SUPPORTED
  return (axis == 1) ? PIN_DRV_DIR2 : PIN_DRV_DIR;
#else
  (void)axis;
  return PIN_DRV_DIR;
#endif
}

static inline int axis_hw_en_pin(int axis) {
#if PIN_AXIS2_SUPPORTED
  return (axis == 1) ? PIN_DRV_EN2 : PIN_DRV_EN;
#else
  (void)axis;
  return PIN_DRV_EN;
#endif
}

static inline int axis_hw_error_pin(int axis) {
#if PIN_AXIS2_SUPPORTED
  return (axis == 1) ? PIN_DRV_ERROR2 : PIN_DRV_ERROR;
#else
  (void)axis;
  return PIN_DRV_ERROR;
#endif
}

static inline int axis_hw_home_pin(int axis) {
#if PIN_AXIS2_SUPPORTED
  return (axis == 1) ? PIN_SW_HOME2 : PIN_SW_HOME;
#else
  (void)axis;
  return PIN_SW_HOME;
#endif
}

static inline int axis_hw_limit_l_pin(int axis) {
#if PIN_AXIS2_SUPPORTED
  return (axis == 1) ? PIN_SW_LIMIT_L2 : PIN_SW_LIMIT_L;
#else
  (void)axis;
  return PIN_SW_LIMIT_L;
#endif
}

static inline int axis_hw_limit_r_pin(int axis) {
#if PIN_AXIS2_SUPPORTED
  return (axis == 1) ? PIN_SW_LIMIT_R2 : PIN_SW_LIMIT_R;
#else
  (void)axis;
  return PIN_SW_LIMIT_R;
#endif
}

static inline int axis_hw_step_active(int axis) {
  const McConfig *c = config_get();
  return (axis == 1) ? c->drv_step_active_2 : c->drv_step_active;
}

static inline int axis_hw_dir_active(int axis) {
  const McConfig *c = config_get();
  return (axis == 1) ? c->drv_dir_active_2 : c->drv_dir_active;
}

static inline int axis_hw_en_active(int axis) {
  const McConfig *c = config_get();
  return (axis == 1) ? c->drv_en_active_2 : c->drv_en_active;
}

static inline int axis_hw_error_active(int axis) {
  const McConfig *c = config_get();
  return (axis == 1) ? c->drv_error_active_2 : c->drv_error_active;
}

static inline int axis_hw_home_active(int axis) {
  const McConfig *c = config_get();
  return (axis == 1) ? c->sw_home_active_2 : c->sw_home_active;
}

static inline int axis_hw_home_use(int axis) {
  const McConfig *c = config_get();
  return (axis == 1) ? c->sw_home_use_2 : c->sw_home_use;
}

static inline int axis_hw_limit_l_active(int axis) {
  const McConfig *c = config_get();
  return (axis == 1) ? c->sw_limit_l_active_2 : c->sw_limit_l_active;
}

static inline int axis_hw_limit_r_active(int axis) {
  const McConfig *c = config_get();
  return (axis == 1) ? c->sw_limit_r_active_2 : c->sw_limit_r_active;
}

static inline int axis_hw_limit_l_use(int axis) {
  const McConfig *c = config_get();
  return (axis == 1) ? c->sw_limit_l_use_2 : c->sw_limit_l_use;
}

static inline int axis_hw_limit_r_use(int axis) {
  const McConfig *c = config_get();
  return (axis == 1) ? c->sw_limit_r_use_2 : c->sw_limit_r_use;
}

static inline float axis_hw_steps_per_unit(int axis) {
  const McConfig *c = config_get();
  return (axis == 1) ? c->steps_per_unit_2 : c->steps_per_unit;
}

static inline float axis_hw_slider_min(int axis) {
  const McConfig *c = config_get();
  return (axis == 1) ? c->slider_min_mm_2 : c->slider_min_mm;
}

static inline float axis_hw_slider_max(int axis) {
  const McConfig *c = config_get();
  return (axis == 1) ? c->slider_max_mm_2 : c->slider_max_mm;
}

static inline int axis_hw_home_mode(int axis) {
  const McConfig *c = config_get();
  return (axis == 1) ? c->home_mode_2 : c->home_mode;
}

static inline float axis_hw_home_move_out(int axis) {
  const McConfig *c = config_get();
  return (axis == 1) ? c->home_move_out_mm_2 : c->home_move_out_mm;
}

static inline float axis_hw_home_speed(int axis) {
  const McConfig *c = config_get();
  return (axis == 1) ? c->home_speed_mm_s_2 : c->home_speed_mm_s;
}

static inline float axis_hw_home_accel(int axis) {
  const McConfig *c = config_get();
  return (axis == 1) ? c->home_accel_mm_s2_2 : c->home_accel_mm_s2;
}

#ifdef __cplusplus
}
#endif
