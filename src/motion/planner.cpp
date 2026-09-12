#include "planner.h"
#include "planner_math.h"
#include "pio_step.h"
#include "config_store.h"
#include "motion_diag.h"
#include "pins.h"
#include "debug_hw.h"
#include "protocol.h"
#include "protocol_internal.h"
#include "config_defaults.h"
#include "motion_path.h"
#include "servo_pwm.h"

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#ifndef HOST_TEST
#include <Arduino.h>
#endif

#include "axis_hw.h"

#define HARD_LIMIT_DEBOUNCE_S 0.020f
#define HOME_EN_PULSE_S 0.20f
#define HOME_STALL_CLEAR_S 2.0f
#define HOME_STALL_IGNORE_S 0.05f
#define AXIS_MAX 3

typedef enum {
  HOME_IDLE = 0,
  HOME_CLEAR,
  HOME_SEEK,
  HOME_STALL_EN,
  HOME_STALL_WAIT,
  HOME_BACKOFF_REL,
  HOME_BACKOFF_OUT
} HomePhase;

typedef struct {
  int id;
  McStatus st;
  float cruise_mm_s;
  float accel_mm_s2;
  int64_t pos_steps;
  int64_t target_steps;
  float vel_mm_s;
  float ramp_v0;
  float ramp_v1;
  float ramp_phi;
  bool ramp_active;
  bool stopping;
  bool dir_pause;
  float dir_pause_s;
  int last_sign;
  bool fill_wants_more;
  float acc_meas;
  bool braking;
  int brake_d;
  int64_t brake_pos0;
  float brake_v0;
  float spmm;
  bool hl_l_latched;
  bool hl_r_latched;
  bool hl_l_stable;
  bool hl_r_stable;
  float hl_l_timer_s;
  float hl_r_timer_s;
  bool hl_l_raw_prev;
  bool hl_r_raw_prev;
  bool home_stable;
  float home_timer_s;
  bool home_raw_prev;
  bool drv_err_stable;
  float drv_err_timer_s;
  bool drv_err_raw_prev;
  bool drv_err_seeded;
  HomePhase home_phase;
  int home_sign;
  int64_t home_seek_start_pos;
  int64_t home_max_travel_steps;
  float home_saved_cruise;
  float home_saved_accel;
  bool home_speeds_saved;
  float home_stall_timer_s;
  float home_stall_ignore_s;
} PlannerAxis;

static PlannerAxis g_ax[AXIS_MAX];
static bool g_enabled;
static bool g_drv_error;
/* Coordinated MT: follower ratios vs the time master, for mid-move SS/SA. */
static bool g_coord_active;
static int g_coord_master;
static int g_move_master;
static float g_coord_ratio[MC_CH_MAX];
static bool g_joy_active;
static float g_joy_pct[MC_CH_MAX];
static unsigned g_servo_rr;

typedef struct {
  float pos;
  float vel;
  float target;
  float cruise;
  float accel;
  bool moving;
  bool has_target;
  bool stopping;
} PlannerServo;

static PlannerServo g_sv[SERVO_MAX];

#define AX (g_ax[axis])

static void reset_ramp(int axis);
static void begin_ramp(int axis, float v_cmd);
static void home_restore_speeds(int axis);
static void home_begin_seek(int axis);
static void home_begin_backoff(int axis);
static void home_begin_clear(int axis, int clear_sign);
static void home_finish(int axis);
static void home_abort(int axis, const char *msg);
static void home_start_move_sign(int axis, int sign, float dist_mm);
static bool home_ref_asserted(int axis);
static int home_seek_sign(int axis);
static bool home_cfg_ok(int axis);
static void home_poll_fsm(int axis, float dt_s);
static void home_begin_stall_reset(int axis);
static void planner_halt_axis(int axis);
static void planner_halt_all(void);
static void apply_en_axis(int axis, bool enabled);
static bool stall_home_mode(int axis);
static bool stall_home_ignore_emo(int axis);
static void coord_clear(void);
static void coord_clear_if_idle(void);
static void apply_session_cruise_accel(void);
static void joy_clear(void);
static float cruise_cap(int axis);
static void apply_joy_cruise_accel(void);
static void planner_request_stop_axis(int axis);
static void planner_request_joy(int axis, float signed_v);
static int axis_count(void) { return config_motor_count(); }
static int ch_count(void) { return config_axis_count(); }

static bool ch_is_motor(int ch) { return ch >= 0 && ch < axis_count(); }

static float ch_max_speed(int ch) {
  if (ch_is_motor(ch)) {
    return axis_hw_max_speed(ch);
  }
  int s = ch - axis_count();
  if (s < 0 || s >= SERVO_MAX) {
    return 1.0f;
  }
  return config_get()->servo_max_speed[s];
}

static float ch_max_accel(int ch) {
  if (ch_is_motor(ch)) {
    return axis_hw_max_accel(ch);
  }
  int s = ch - axis_count();
  if (s < 0 || s >= SERVO_MAX) {
    return 1.0f;
  }
  return config_get()->servo_max_accel[s];
}

static float ch_pos(int ch) {
  if (ch_is_motor(ch)) {
    float sp = g_ax[ch].spmm > 1e-3f ? g_ax[ch].spmm : 1.0f;
    return (float)g_ax[ch].pos_steps / sp;
  }
  int s = ch - axis_count();
  return g_sv[s].pos;
}

static void ch_set_cruise_accel(int ch, float v, float a) {
  if (ch_is_motor(ch)) {
    g_ax[ch].cruise_mm_s = v;
    g_ax[ch].accel_mm_s2 = a;
    return;
  }
  int s = ch - axis_count();
  g_sv[s].cruise = v;
  g_sv[s].accel = a;
}

static float servo_boot_deg(int s) {
  const McConfig *c = config_get();
  float p = 0.0f;
  if (!isnan(c->servo_min[s]) && p < c->servo_min[s]) {
    p = c->servo_min[s];
  }
  if (!isnan(c->servo_max[s]) && p > c->servo_max[s]) {
    p = c->servo_max[s];
  }
  return p;
}

static void servo_boot_all(void) {
  const McConfig *c = config_get();
  for (int s = 0; s < SERVO_MAX; ++s) {
    float p = servo_boot_deg(s);
    g_sv[s].pos = p;
    g_sv[s].target = p;
    g_sv[s].vel = 0.0f;
    g_sv[s].cruise = c->servo_max_speed[s];
    g_sv[s].accel = c->servo_max_accel[s];
    g_sv[s].moving = false;
    g_sv[s].has_target = false;
    g_sv[s].stopping = false;
  }
  g_servo_rr = 0;
}

static void servo_request_move(int s, float dest) {
  PlannerServo *sv = &g_sv[s];
  sv->target = dest;
  sv->stopping = false;
  sv->has_target = true;
  if (fabsf(dest - sv->pos) < 1e-6f) {
    sv->moving = false;
    sv->vel = 0.0f;
    sv->has_target = false;
    return;
  }
  sv->moving = true;
}

static void servo_request_stop(int s) {
  PlannerServo *sv = &g_sv[s];
  sv->has_target = false;
  sv->stopping = true;
  if (!sv->moving || fabsf(sv->vel) < 0.05f) {
    sv->vel = 0.0f;
    sv->moving = false;
    sv->stopping = false;
    sv->target = sv->pos;
  }
}

static void servo_halt(int s) {
  g_sv[s].vel = 0.0f;
  g_sv[s].moving = false;
  g_sv[s].has_target = false;
  g_sv[s].stopping = false;
  g_sv[s].target = g_sv[s].pos;
}

static void servo_integrate(int s, float dt) {
  PlannerServo *sv = &g_sv[s];
  if (dt < 0.0f) {
    dt = 0.0f;
  }
  float a = sv->accel;
  if (a < 0.001f) {
    a = 0.001f;
  }
  float vmax = sv->cruise;
  if (vmax < 0.001f) {
    vmax = 0.001f;
  }
  if (sv->stopping) {
    float v = sv->vel;
    if (fabsf(v) < 0.05f) {
      sv->vel = 0.0f;
      sv->moving = false;
      sv->stopping = false;
      sv->target = sv->pos;
      return;
    }
    float sign = (v >= 0.0f) ? 1.0f : -1.0f;
    v -= sign * a * dt;
    if (v * sign <= 0.0f) {
      v = 0.0f;
      sv->moving = false;
      sv->stopping = false;
      sv->target = sv->pos;
    }
    sv->pos += sv->vel * dt;
    sv->vel = v;
    return;
  }
  if (!sv->moving) {
    sv->vel = 0.0f;
    return;
  }
  float dest = sv->target;
  float pos = sv->pos;
  float err = dest - pos;
  float v = sv->vel;
  int sign = (err > 0.0f) ? 1 : (err < 0.0f) ? -1 : 0;
  if (sign == 0 || (fabsf(err) <= 1e-4f && fabsf(v) < 0.05f)) {
    sv->pos = dest;
    sv->vel = 0.0f;
    sv->moving = false;
    sv->has_target = false;
    return;
  }
  float stop_d = (v * v) / (2.0f * a);
  bool toward = (fabsf(v) < 1e-4f) || ((v >= 0.0f) == (sign > 0));
  if (toward && fabsf(err) <= stop_d + 1e-6f) {
    v -= (float)sign * a * dt;
    if (v * (float)sign <= 0.0f) {
      v = 0.0f;
    }
  } else {
    v += (float)sign * a * dt;
    if (fabsf(v) > vmax) {
      v = (float)sign * vmax;
    }
  }
  pos += v * dt;
  if ((sign > 0 && pos >= dest) || (sign < 0 && pos <= dest)) {
    pos = dest;
    v = 0.0f;
    sv->moving = false;
    sv->has_target = false;
  }
  sv->pos = pos;
  sv->vel = v;
}

static void servo_request_joy(int s, float signed_v) {
  if (s < 0 || s >= config_servo_count() || !g_enabled) {
    return;
  }
  PlannerServo *sv = &g_sv[s];
  sv->accel = ch_max_accel(axis_count() + s);
  float a0 = session_get()->accel_mm_s2;
  if (a0 < sv->accel) {
    sv->accel = a0 < 0.001f ? 0.001f : a0;
  }
  if (fabsf(signed_v) < 1e-4f) {
    servo_request_stop(s);
    return;
  }
  int ch = axis_count() + s;
  int sign = signed_v > 0.0f ? 1 : -1;
  float dest = (sign > 0) ? session_effective_right(ch) : session_effective_left(ch);
  if (isnan(dest)) {
    dest = sv->pos + (float)sign * 1e6f;
  }
  sv->cruise = fabsf(signed_v);
  servo_request_move(s, dest);
}

static void coord_clear(void) {
  g_coord_active = false;
  g_coord_master = 0;
  g_move_master = -1;
  for (int i = 0; i < MC_CH_MAX; ++i) {
    g_coord_ratio[i] = 1.0f;
  }
}

static void joy_clear(void) {
  g_joy_active = false;
  for (int i = 0; i < MC_CH_MAX; ++i) {
    g_joy_pct[i] = 0.0f;
  }
}

static void coord_clear_if_idle(void) {
  if (!g_coord_active) {
    return;
  }
  for (int axis = 0; axis < axis_count(); ++axis) {
    if (AX.st.moving || AX.st.homing) {
      return;
    }
  }
  int ns = config_servo_count();
  for (int s = 0; s < ns; ++s) {
    if (g_sv[s].moving) {
      return;
    }
  }
  coord_clear();
}

static void scale_cruise_accel(int ch, float v0, float a0, float ratio, float *v_out,
                               float *a_out) {
  float v1 = v0 * ratio;
  float a1 = a0 * ratio;
  float vmax = ch_max_speed(ch);
  float amax = ch_max_accel(ch);
  if (v1 > vmax) {
    v1 = vmax;
  }
  if (a1 > amax) {
    a1 = amax;
  }
  if (v1 < 0.001f) {
    v1 = 0.001f;
  }
  if (a1 < 0.001f) {
    a1 = 0.001f;
  }
  *v_out = v1;
  *a_out = a1;
}

static float clamp_speed_axis(int axis, float v) {
  float mx = axis_hw_max_speed(axis);
  if (v > mx) {
    v = mx;
  }
  if (v < 0.0f) {
    v = 0.0f;
  }
  return v;
}

static float clamp_accel_axis(int axis, float a) {
  float mx = axis_hw_max_accel(axis);
  if (a > mx) {
    a = mx;
  }
  if (a < 0.001f) {
    a = 0.001f;
  }
  return a;
}

static float clamp_speed_ch(int ch, float v) {
  float mx = ch_max_speed(ch);
  if (v > mx) {
    v = mx;
  }
  if (v < 0.0f) {
    v = 0.0f;
  }
  return v;
}

static float clamp_accel_ch(int ch, float a) {
  float mx = ch_max_accel(ch);
  if (a > mx) {
    a = mx;
  }
  if (a < 0.001f) {
    a = 0.001f;
  }
  return a;
}

/** Apply session cruise/accel; coordinated followers scale vs the master. */
static void apply_cruise_accel(float v0, float a0) {
  int n = ch_count();
  int master = g_coord_active ? g_coord_master : 0;
  if (master < 0 || master >= n) {
    master = 0;
  }
  if (g_coord_active) {
    ch_set_cruise_accel(master, clamp_speed_ch(master, v0), clamp_accel_ch(master, a0));
    for (int ch = 0; ch < n; ++ch) {
      if (ch == master) {
        continue;
      }
      float v, a;
      scale_cruise_accel(ch, v0, a0, g_coord_ratio[ch], &v, &a);
      ch_set_cruise_accel(ch, v, a);
    }
    return;
  }
  for (int ch = 0; ch < n; ++ch) {
    ch_set_cruise_accel(ch, clamp_speed_ch(ch, v0), clamp_accel_ch(ch, a0));
  }
}

static void apply_session_cruise_accel(void) {
  apply_cruise_accel(session_get()->speed_mm_s, session_get()->accel_mm_s2);
}

static bool read_limit_raw(int axis, bool left) {
#ifndef HOST_TEST
  if (left) {
    if (!axis_hw_limit_l_use(axis)) {
      return false;
    }
    int level = digitalRead(axis_hw_limit_l_pin(axis)) ? 1 : 0;
    return config_pin_asserted(level, axis_hw_limit_l_active(axis));
  }
  if (!axis_hw_limit_r_use(axis)) {
    return false;
  }
  int level = digitalRead(axis_hw_limit_r_pin(axis)) ? 1 : 0;
  return config_pin_asserted(level, axis_hw_limit_r_active(axis));
#else
  (void)axis;
  (void)left;
  return false;
#endif
}

static bool read_drv_error_raw(int axis) {
#ifndef HOST_TEST
  int level = digitalRead(axis_hw_error_pin(axis)) ? 1 : 0;
  return config_pin_asserted(level, axis_hw_error_active(axis));
#else
  (void)axis;
  return false;
#endif
}

static void apply_en_axis(int axis, bool enabled) {
#ifndef HOST_TEST
  int pin = axis_hw_en_pin(axis);
  int active = axis_hw_en_active(axis);
  if (enabled) {
    digitalWrite(pin, active ? HIGH : LOW);
  } else {
    digitalWrite(pin, active ? LOW : HIGH);
  }
#else
  (void)axis;
  (void)enabled;
#endif
}

static void apply_en_output(bool enabled) {
  apply_en_axis(0, enabled);
}

static bool stall_home_mode(int axis) {
  int hm = axis_hw_home_mode(axis);
  return hm == 3 || hm == 4;
}

static bool stall_home_ignore_emo(int axis) {
  if (!AX.st.homing || !stall_home_mode(axis)) {
    return false;
  }
  if (AX.home_stall_ignore_s > 0.0f) {
    return true;
  }
  return AX.home_phase == HOME_SEEK || AX.home_phase == HOME_STALL_EN ||
         AX.home_phase == HOME_STALL_WAIT;
}

static void refresh_state(int axis) {
  AX.st.hard_limit = AX.hl_l_latched || AX.hl_r_latched || AX.hl_l_stable || AX.hl_r_stable;
  AX.st.drv_error = g_drv_error;
  AX.st.enabled = g_enabled;
  if (g_drv_error) {
    AX.st.state = MC_STATE_ERROR;
  } else if (AX.st.hard_limit && !AX.st.moving) {
    AX.st.state = MC_STATE_HARD_LIMIT;
  } else if (!g_enabled) {
    AX.st.state = MC_STATE_DISABLED;
  } else if (AX.st.homing) {
    AX.st.state = MC_STATE_HOMING;
  } else if (AX.st.moving) {
    if (AX.braking || AX.stopping) {
      AX.st.state = MC_STATE_DECELERATING;
    } else if (AX.ramp_active) {
      /* Mid-move SS change and start/stop ramps share begin_ramp(axis, v0→v1). */
      if (fabsf(AX.ramp_v1) + 1e-4f < fabsf(AX.ramp_v0)) {
        AX.st.state = MC_STATE_DECELERATING;
      } else if (fabsf(AX.ramp_v1) > fabsf(AX.ramp_v0) + 1e-4f) {
        AX.st.state = MC_STATE_ACCELERATING;
      } else {
        AX.st.state = MC_STATE_MOVING;
      }
    } else {
      AX.st.state = MC_STATE_MOVING;
    }
  } else {
    AX.st.state = MC_STATE_IDLE;
  }
  AX.st.pos[0] = (float)AX.pos_steps / AX.spmm;
  AX.st.target[0] = (float)AX.target_steps / AX.spmm;
  AX.st.vel[0] = AX.vel_mm_s;
  /* Measured |a| of the issued words; 0 in cruise/idle. */
  AX.st.acc[0] = AX.st.moving ? fabsf(AX.acc_meas) : 0.0f;
  AX.st.has_target = AX.st.moving && !AX.stopping;
  if (axis == 0) {
    dbg_hw_set(PIN_DBG_MOV, AX.st.moving || AX.st.homing);
  }
}

/**
 * Emergency halt: immediate STEP abort, EN off, cancel waits/chain.
 * Used by ME, ESC, hard-limit trips, and PIN_DRV_ERROR_*.
 */
static void planner_halt_axis(int axis) {
  AX.vel_mm_s = 0.0f;
  AX.st.moving = false;
  AX.st.homing = false;
  AX.home_phase = HOME_IDLE;
  home_restore_speeds(axis);
  AX.stopping = false;
  AX.dir_pause = false;
  AX.st.has_target = false;
  AX.target_steps = AX.pos_steps;
  reset_ramp(axis);
  pio_step_stop_hard(axis);
  refresh_state(axis);
}

static void planner_halt_all(void) {
  protocol_debug(3, "D:halt\n");
  joy_clear();
  coord_clear();
  for (int a = 0; a < axis_count(); ++a) {
    planner_halt_axis(a);
  }
  int ns = config_servo_count();
  for (int s = 0; s < ns; ++s) {
    servo_halt(s);
  }
  g_enabled = false;
  apply_en_output(false);
  protocol_cancel_waits_and_chain();
}

static void planner_trip_hard_limit(int axis, bool left) {
  if (AX.st.homing) {
    int hm = axis_hw_home_mode(axis);
    bool is_ref = (hm == 1 && left) || (hm == 2 && !left);
    if (is_ref && AX.home_phase == HOME_SEEK) {
      /* Reference limit hit — enter backoff, do not fault. */
      home_begin_backoff(axis);
      return;
    }
    if (is_ref && (AX.home_phase == HOME_BACKOFF_REL || AX.home_phase == HOME_BACKOFF_OUT)) {
      return;
    }
    if (hm >= 1 && hm <= 4) {
      protocol_error("home", "hard");
      /* fall through to hard-stop fault */
    }
  }

  if (left) {
    if (AX.hl_l_latched) {
      return;
    }
    AX.hl_l_latched = true;
  } else {
    if (AX.hl_r_latched) {
      return;
    }
    AX.hl_r_latched = true;
  }
  planner_halt_all();
}

bool planner_hard_limit_blocks_sign(int axis, int sign) {
  if (sign < 0 && (AX.hl_l_latched || AX.hl_l_stable)) {
    return true;
  }
  if (sign > 0 && (AX.hl_r_latched || AX.hl_r_stable)) {
    return true;
  }
  return false;
}

static void debounce_side(int axis, bool left, float dt_s) {
  bool use = left ? axis_hw_limit_l_use(axis) : axis_hw_limit_r_use(axis);
  bool *stable = left ? &AX.hl_l_stable : &AX.hl_r_stable;
  bool *latched = left ? &AX.hl_l_latched : &AX.hl_r_latched;
  float *timer = left ? &AX.hl_l_timer_s : &AX.hl_r_timer_s;
  bool *raw_prev = left ? &AX.hl_l_raw_prev : &AX.hl_r_raw_prev;

  if (!use) {
    *stable = false;
    *latched = false;
    *timer = 0.0f;
    *raw_prev = false;
    return;
  }

#ifndef HOST_TEST
  /* Lazy init if CS enables use after boot. */
  pinMode(left ? axis_hw_limit_l_pin(axis) : axis_hw_limit_r_pin(axis), INPUT_PULLUP);
#endif

  bool raw = read_limit_raw(axis, left);
  if (raw != *raw_prev) {
    *timer = 0.0f;
    *raw_prev = raw;
  } else {
    *timer += dt_s;
  }

  if (*timer >= HARD_LIMIT_DEBOUNCE_S) {
    if (raw && !*stable) {
      *stable = true;
      planner_trip_hard_limit(axis, left);
    } else if (!raw && *stable) {
      *stable = false;
      *latched = false;
    }
  }
}

static void debounce_drv_error(int axis, float dt_s) {
#ifndef HOST_TEST
  pinMode(axis_hw_error_pin(axis), INPUT_PULLUP);
#endif

  bool raw = read_drv_error_raw(axis);
  if (!AX.drv_err_seeded) {
    /* Power-up: seed from first sample so an already-asserted pin can become stable. */
    AX.drv_err_raw_prev = raw;
    AX.drv_err_timer_s = 0.0f;
    AX.drv_err_seeded = true;
  } else if (raw != AX.drv_err_raw_prev) {
    AX.drv_err_timer_s = 0.0f;
    AX.drv_err_raw_prev = raw;
  } else {
    AX.drv_err_timer_s += dt_s;
  }

  if (AX.home_stall_ignore_s > 0.0f) {
    AX.home_stall_ignore_s -= dt_s;
    if (AX.home_stall_ignore_s < 0.0f) {
      AX.home_stall_ignore_s = 0.0f;
    }
  }

  if (AX.drv_err_timer_s >= HARD_LIMIT_DEBOUNCE_S) {
    if (raw && !AX.drv_err_stable) {
      AX.drv_err_stable = true;
      if (stall_home_ignore_emo(axis)) {
        if (AX.home_phase == HOME_SEEK) {
          home_begin_stall_reset(axis);
        }
      } else {
        g_drv_error = true;
        AX.st.drv_error = true;
        planner_halt_all();
      }
    } else if (!raw && AX.drv_err_stable) {
      AX.drv_err_stable = false;
      /* clear only if no axis still asserts; rechecked below */
      bool any = false;
      for (int a = 0; a < axis_count(); ++a) {
        if (g_ax[a].drv_err_stable) { any = true; break; }
      }
      g_drv_error = any;
      AX.st.drv_error = g_drv_error;
      refresh_state(axis);
    }
  }
}

static void planner_poll_switches(int axis, float dt_s) {
  debounce_side(axis, true, dt_s);
  debounce_side(axis, false, dt_s);
  debounce_drv_error(axis, dt_s);
}

static int64_t mm_to_steps(int axis, float mm) {
  return (int64_t)lroundf(mm * AX.spmm);
}

static void home_restore_speeds(int axis) {
  if (!AX.home_speeds_saved) {
    return;
  }
  AX.cruise_mm_s = AX.home_saved_cruise;
  AX.accel_mm_s2 = AX.home_saved_accel;
  AX.home_speeds_saved = false;
}

static int home_seek_sign(int axis) {
  int hm = axis_hw_home_mode(axis);
  return (hm == 1 || hm == 3) ? -1 : 1;
}

static bool home_ref_asserted(int axis) {
  int hm = axis_hw_home_mode(axis);
  if (hm == 1) {
    return AX.hl_l_stable;
  }
  if (hm == 2) {
    return AX.hl_r_stable;
  }
  if (hm == 3 || hm == 4) {
    return AX.drv_err_stable;
  }
  return false;
}

static bool home_cfg_ok(int axis) {
  switch (axis_hw_home_mode(axis)) {
  case 1:
    return axis_hw_limit_l_use(axis) != 0;
  case 2:
    return axis_hw_limit_r_use(axis) != 0;
  case 3:
  case 4:
    return true; /* DRV_ERROR is always polled */
  default:
    return false;
  }
}

static void home_start_move_sign(int axis, int sign, float dist_mm) {
  if (sign == 0 || dist_mm < 0.0f) {
    return;
  }
  AX.spmm = axis_hw_steps_per_unit(axis);
  if (AX.spmm < 1e-3f) {
    AX.spmm = 1.0f;
  }
  int64_t delta = mm_to_steps(axis, dist_mm);
  if (delta < 1) {
    delta = 1;
  }
  AX.target_steps = AX.pos_steps + (int64_t)sign * delta;
  AX.stopping = false;
  AX.st.moving = true;
  AX.st.has_target = true;
  begin_ramp(axis, AX.vel_mm_s);
  AX.fill_wants_more = true;
  AX.acc_meas = 0.0f;
  AX.braking = false;
  AX.brake_d = 0;
  pio_step_clear_stall(axis); /* stale flag from the previous idle gap */
  pio_step_kick_feed();
}

static void home_begin_seek(int axis) {
  AX.home_phase = HOME_SEEK;
  AX.home_sign = home_seek_sign(axis);
  AX.home_seek_start_pos = AX.pos_steps;
  float span_mm;
  float smin = axis_hw_slider_min(axis);
  float smax = axis_hw_slider_max(axis);
  if (!isnan(smin) && !isnan(smax) && smax > smin) {
    span_mm = smax - smin;
  } else {
    span_mm = CFG_DEFAULT_MOTOR_MAX - CFG_DEFAULT_MOTOR_MIN;
  }
  float max_mm = span_mm * 1.1f;
  if (max_mm < 1.0f) {
    max_mm = 1.0f;
  }
  AX.home_max_travel_steps = mm_to_steps(axis, max_mm);
  if (AX.home_max_travel_steps < 1) {
    AX.home_max_travel_steps = 1;
  }
  home_start_move_sign(axis, AX.home_sign, max_mm);
}

static void home_begin_backoff(int axis) {
  AX.home_phase = HOME_BACKOFF_REL;
  /* Leave reference switch; clear latches on reference limit so drive-out works. */
  int hm = axis_hw_home_mode(axis);
  if (hm == 1) {
    AX.hl_l_latched = false;
  } else if (hm == 2) {
    AX.hl_r_latched = false;
  }
  if (stall_home_mode(axis)) {
    AX.home_stall_ignore_s = HOME_STALL_IGNORE_S;
  }
  float span = 50.0f;
  float smin = axis_hw_slider_min(axis);
  float smax = axis_hw_slider_max(axis);
  if (!isnan(smin) && !isnan(smax) && smax > smin) {
    span = (smax - smin) * 0.25f;
  }
  if (span < 10.0f) {
    span = 10.0f;
  }
  home_start_move_sign(axis, -AX.home_sign, span);
}

static void home_begin_clear(int axis, int clear_sign) {
  AX.home_phase = HOME_CLEAR;
  if (clear_sign < 0) {
    AX.hl_l_latched = false;
  } else if (clear_sign > 0) {
    AX.hl_r_latched = false;
  }
  float span = 50.0f;
  float smin = axis_hw_slider_min(axis);
  float smax = axis_hw_slider_max(axis);
  if (!isnan(smin) && !isnan(smax) && smax > smin) {
    span = (smax - smin) * 0.25f;
  }
  if (span < 10.0f) {
    span = 10.0f;
  }
  home_start_move_sign(axis, clear_sign, span);
}

static void home_finish(int axis) {
  float pos_mm;
  float smin = axis_hw_slider_min(axis);
  float smax = axis_hw_slider_max(axis);
  if (axis_hw_home_mode(axis) == 1 || axis_hw_home_mode(axis) == 3) {
    pos_mm = isnan(smin) ? 0.0f : smin;
  } else {
    pos_mm = isnan(smax) ? 0.0f : smax;
  }
  AX.spmm = axis_hw_steps_per_unit(axis);
  if (AX.spmm < 1e-3f) {
    AX.spmm = 1.0f;
  }
  AX.pos_steps = mm_to_steps(axis, pos_mm);
  AX.target_steps = AX.pos_steps;
  AX.vel_mm_s = 0.0f;
  AX.st.moving = false;
  AX.st.homing = false;
  AX.st.has_target = false;
  AX.stopping = false;
  AX.home_phase = HOME_IDLE;
  reset_ramp(axis);
  home_restore_speeds(axis);
  if (g_enabled) {
    apply_en_axis(axis, true);
  }
  pio_step_stop_soft(axis);
  refresh_state(axis);
}

static void home_begin_stall_reset(int axis) {
  AX.vel_mm_s = 0.0f;
  AX.st.moving = false;
  AX.st.has_target = false;
  AX.stopping = false;
  AX.target_steps = AX.pos_steps;
  reset_ramp(axis);
  pio_step_stop_hard(axis);
  AX.home_phase = HOME_STALL_EN;
  AX.home_stall_timer_s = 0.0f;
  apply_en_axis(axis, false);
  refresh_state(axis);
}

static void home_abort(int axis, const char *msg) {
  AX.home_phase = HOME_IDLE;
  AX.st.homing = false;
  AX.st.moving = false;
  AX.st.has_target = false;
  AX.stopping = false;
  AX.dir_pause = false;
  AX.vel_mm_s = 0.0f;
  AX.target_steps = AX.pos_steps;
  reset_ramp(axis);
  home_restore_speeds(axis);
  if (g_enabled) {
    apply_en_axis(axis, true);
  }
  pio_step_stop_hard(axis);
  protocol_error("home", msg ? msg : "abort");
  protocol_cancel_waits_and_chain();
  refresh_state(axis);
}

static void home_poll_fsm(int axis, float dt_s) {
  if (!AX.st.homing || AX.home_phase == HOME_IDLE || AX.stopping) {
    return;
  }

  if (AX.home_phase == HOME_STALL_EN) {
    AX.home_stall_timer_s += dt_s;
    if (AX.home_stall_timer_s >= HOME_EN_PULSE_S) {
      apply_en_axis(axis, true);
      AX.drv_err_seeded = false;
      AX.home_phase = HOME_STALL_WAIT;
      AX.home_stall_timer_s = 0.0f;
    }
    return;
  }

  if (AX.home_phase == HOME_STALL_WAIT) {
    AX.home_stall_timer_s += dt_s;
    if (!AX.drv_err_stable) {
      home_begin_backoff(axis);
      return;
    }
    if (AX.home_stall_timer_s >= HOME_STALL_CLEAR_S) {
      home_abort(axis, "stall");
    }
    return;
  }

  if (AX.home_phase == HOME_CLEAR) {
    if (!AX.hl_l_stable && !AX.hl_r_stable) {
      AX.hl_l_latched = false;
      AX.hl_r_latched = false;
      AX.vel_mm_s = 0.0f;
      AX.target_steps = AX.pos_steps;
      reset_ramp(axis);
      pio_step_stop_hard(axis);
      if (home_ref_asserted(axis)) {
        home_begin_backoff(axis);
      } else {
        home_begin_seek(axis);
      }
    }
    return;
  }

  if (AX.home_phase == HOME_SEEK) {
    int64_t traveled = AX.pos_steps - AX.home_seek_start_pos;
    if (traveled < 0) {
      traveled = -traveled;
    }
    if (traveled >= AX.home_max_travel_steps) {
      home_abort(axis, "travel");
      return;
    }
    if (home_ref_asserted(axis)) {
      home_begin_backoff(axis);
    }
    return;
  }

  if (AX.home_phase == HOME_BACKOFF_REL) {
    if (!home_ref_asserted(axis)) {
      float out = axis_hw_home_move_out(axis);
      if (out < 0.0f) {
        out = 0.0f;
      }
      AX.home_phase = HOME_BACKOFF_OUT;
      if (out < 1e-4f) {
        home_finish(axis);
      } else {
        home_start_move_sign(axis, -AX.home_sign, out);
      }
    }
    return;
  }

  if (AX.home_phase == HOME_BACKOFF_OUT) {
    if (AX.pos_steps == AX.target_steps && fabsf(AX.vel_mm_s) < 0.01f && pio_step_tx_empty(axis)) {
      home_finish(axis);
    }
  }
}

static void reset_ramp(int axis) {
  AX.ramp_active = false;
  AX.ramp_v0 = AX.vel_mm_s;
  AX.ramp_v1 = AX.vel_mm_s;
  AX.ramp_phi = 0.0f;
}

static void begin_ramp(int axis, float v_cmd) {
  if (!AX.ramp_active || fabsf(v_cmd - AX.ramp_v1) > 1e-4f) {
    AX.ramp_v0 = AX.vel_mm_s;
    AX.ramp_v1 = v_cmd;
    AX.ramp_phi = 0.0f;
    AX.ramp_active = true;
  }
}

static int64_t soft_min_steps(int axis) {
  float mn = axis_hw_window_min(axis);
  if (isnan(mn)) {
    return INT64_MIN / 4;
  }
  return mm_to_steps(axis, mn);
}

static int64_t soft_max_steps(int axis) {
  float mx = axis_hw_window_max(axis);
  if (isnan(mx)) {
    return INT64_MAX / 4;
  }
  return mm_to_steps(axis, mx);
}

static int remaining_steps_for_sign(int axis, int sign) {
  int64_t rem_tgt = AX.target_steps - AX.pos_steps;
  if (sign > 0 && rem_tgt < 0) {
    rem_tgt = 0;
  }
  if (sign < 0 && rem_tgt > 0) {
    rem_tgt = 0;
  }
  int rem = (int)(rem_tgt >= 0 ? rem_tgt : -rem_tgt);

  /* Soft limits do not constrain homing moves. */
  if (!AX.st.homing) {
    if (sign > 0) {
      int64_t to_lim = soft_max_steps(axis) - AX.pos_steps;
      if (to_lim < 0) {
        to_lim = 0;
      }
      if (to_lim < rem) {
        rem = (int)to_lim;
      }
    } else if (sign < 0) {
      int64_t to_lim = AX.pos_steps - soft_min_steps(axis);
      if (to_lim < 0) {
        to_lim = 0;
      }
      if (to_lim < rem) {
        rem = (int)to_lim;
      }
    }
  }
  return rem;
}

static float cruise_cap(int axis) {
  float v = AX.cruise_mm_s;
  float mx = axis_hw_max_speed(axis);
  if (v > mx) {
    v = mx;
  }
  float hz_max = planner_max_step_hz(pio_step_sysclk_hz(), PIO_STEP_PERIOD_FIXED);
  float v_hz = hz_max / AX.spmm;
  if (v > v_hz) {
    v = v_hz;
  }
  return v;
}

static float signed_cruise_from_pct(int ch, float pct) {
  if (fabsf(pct) < 1e-3f) {
    return 0.0f;
  }
  float v = fabsf(pct) * 0.01f * session_get()->speed_mm_s;
  v = clamp_speed_ch(ch, v);
  return (pct < 0.0f) ? -v : v;
}

static void apply_joy_cruise_accel(void) {
  int nm = axis_count();
  int n = ch_count();
  for (int ch = 0; ch < n; ++ch) {
    float pct = g_joy_pct[ch];
    if (fabsf(pct) < 1e-3f) {
      continue;
    }
    float signed_v = signed_cruise_from_pct(ch, pct);
    if (ch < nm) {
      int axis = ch;
      AX.accel_mm_s2 = clamp_accel_axis(axis, session_get()->accel_mm_s2);
      AX.cruise_mm_s = fabsf(signed_v);
      if (AX.st.moving && !AX.stopping) {
        int sign = (signed_v >= 0.0f) ? 1 : -1;
        begin_ramp(axis, (float)sign * cruise_cap(axis));
      }
    } else {
      int s = ch - nm;
      g_sv[s].accel = clamp_accel_ch(ch, session_get()->accel_mm_s2);
      g_sv[s].cruise = fabsf(signed_v);
    }
  }
}

static bool pio_axis_drained(int axis) {
  return pio_step_tx_empty(axis) && pio_step_pending_steps(axis) <= 0;
}

/* At the commanded pose with nothing left in PIO: leftover vel_mm_s is stale
 * (no pulses are going out). Zero it and leave M, otherwise a 0-step MT can
 * sit in M forever with a cruise residue and no position change. */
static void settle_idle_at_pose(int axis) {
  AX.vel_mm_s = 0.0f;
  AX.st.moving = false;
  AX.st.homing = false;
  AX.stopping = false;
  AX.fill_wants_more = false;
  AX.braking = false;
  AX.brake_d = 0;
  AX.target_steps = AX.pos_steps;
  reset_ramp(axis);
  pio_step_stop_soft(axis);
  refresh_state(axis);
  coord_clear_if_idle();
}

static void settle_if_done(int axis) {
  int64_t err = AX.target_steps - AX.pos_steps;
  if (AX.stopping) {
    /* Keep the vel gate here so a momentary empty FIFO during a real stop
     * ramp does not abort the remaining brake words. */
    if (fabsf(AX.vel_mm_s) < 0.01f && pio_axis_drained(axis)) {
      home_restore_speeds(axis);
      if (g_enabled) {
        apply_en_axis(axis, true);
      }
      AX.home_phase = HOME_IDLE;
      settle_idle_at_pose(axis);
    }
    return;
  }
  if (AX.st.homing && AX.home_phase != HOME_IDLE) {
    /* Homing FSM owns completion (home_poll_fsm / home_finish). */
    return;
  }
  if (err == 0 && pio_step_tx_empty(axis)) {
    /* Do not wait for pending_steps==0: shadow can stay stale after TXSTALL
     * while the FIFO is already empty, which stuck M at v=0. */
    settle_idle_at_pose(axis);
    return;
  }
  /* Fill gave up (fill_wants_more=false) while still short of the target and
   * nothing is pulsing — re-arm rather than sit in M. */
  if (!AX.dir_pause && fabsf(AX.vel_mm_s) < 0.01f && pio_step_tx_empty(axis) &&
      !AX.fill_wants_more) {
    AX.fill_wants_more = true;
    pio_step_kick_feed();
  }
}

void planner_init(void) {
  g_enabled = false;
  g_drv_error = false;
  coord_clear();
  joy_clear();
  for (int axis = 0; axis < AXIS_MAX; ++axis) {
    memset(&AX, 0, sizeof(AX));
    AX.id = axis;
    AX.st.state = MC_STATE_DISABLED;
    AX.spmm = axis_hw_steps_per_unit(axis);
    if (AX.spmm < 1e-3f) {
      AX.spmm = 1.0f;
    }
    AX.cruise_mm_s = session_get()->speed_mm_s;
    AX.accel_mm_s2 = session_get()->accel_mm_s2;
    reset_ramp(axis);
    refresh_state(axis);
  }
  motion_diag_init_from_noinit();
  pio_step_init();
  servo_boot_all();
  servo_pwm_init();
}

void planner_set_cruise_accel(int axis, float cruise_mm_s, float accel_mm_s2) {
  if (axis < 0 || axis >= AXIS_MAX) {
    return;
  }
  AX.cruise_mm_s = cruise_mm_s;
  AX.accel_mm_s2 = accel_mm_s2;
}

int planner_fill_fifo(int axis) {
  int emitted = 0;
  if (!AX.st.moving || g_drv_error || !g_enabled) {
    AX.fill_wants_more = false;
    return 0;
  }
  if (AX.dir_pause) {
    AX.fill_wants_more = false;
    return 0;
  }

  int nax = axis_count();
  /* 3-axis moves are the tightest case: keep a slightly larger per-call burst so
   * we do not spend a full interrupt/service cycle on every 1-2 word refill.
   * The extra headroom is small, but it reduces the chance of a low-water stall
   * between the time the FIFO due to drain and the next feed wake. */
  unsigned room_budget = pio_step_tx_empty(axis)
                             ? 8u
                             : ((nax >= 3) ? 3u : (nax >= 2) ? 4u : 6u);
  while (room_budget-- && pio_step_tx_room(axis) > 0) {
    int64_t err = AX.target_steps - AX.pos_steps;
    int sign;
    if (AX.stopping) {
      if (fabsf(AX.vel_mm_s) < 0.01f) {
        AX.fill_wants_more = false;
        break;
      }
      sign = (AX.vel_mm_s >= 0.0f) ? 1 : -1;
    } else if (err > 0) {
      sign = 1;
    } else if (err < 0) {
      sign = -1;
    } else {
      /* At target: finish deceleration, but stop rather than creep past it for
         a residue that is only worth a few steps. The approach floor leaves the
         axis at stop_approach_hz, so that speed also counts as arrived —
         otherwise the trailing decel would issue steps beyond the target. */
      int stop_app_hz = config_get()->stop_approach_hz;
      float v_stop = (stop_app_hz > 0) ? (float)stop_app_hz / AX.spmm : 0.0f;
      if (fabsf(AX.vel_mm_s) <= v_stop + 1e-3f ||
          planner_stop_rem_steps(AX.vel_mm_s, AX.accel_mm_s2, AX.spmm) <= 4) {
        AX.vel_mm_s = 0.0f;
        AX.fill_wants_more = false;
        break;
      }
      sign = (AX.vel_mm_s >= 0.0f) ? 1 : -1;
    }

    /* Reverse: decelerate in current travel direction until ~0, then pause. */
    bool reverse_decel = false;
    if (!AX.stopping && planner_needs_reverse_decel(AX.last_sign, sign, AX.vel_mm_s)) {
      begin_ramp(axis, 0.0f);
      sign = (AX.vel_mm_s >= 0.0f) ? 1 : -1;
      reverse_decel = true;
      protocol_debug(4, "D:reverse_decel v=%.2f\n", (double)AX.vel_mm_s);
    }

    int rem;
    if (AX.stopping || err == 0 || reverse_decel) {
      rem = planner_stop_rem_steps(AX.vel_mm_s, AX.accel_mm_s2, AX.spmm);
    } else {
      rem = remaining_steps_for_sign(axis, sign);
    }

    if (rem <= 0) {
      if (err != 0 && !AX.stopping && !reverse_decel) {
        /* Nothing left to issue although the target is not reached (soft limit).
           Drop the target so settle_if_done(axis) can leave the moving state. */
        motion_diag_note_overshoot(1);
        protocol_debug(2, "D:move_blocked pos=%ld tgt=%ld\n", (long)AX.pos_steps,
                       (long)AX.target_steps);
        AX.target_steps = AX.pos_steps;
      }
      AX.vel_mm_s = 0.0f;
      reset_ramp(axis);
      AX.braking = false;
      AX.brake_d = 0;
      AX.fill_wants_more = false;
      break;
    }

    float rem_mm = (float)rem / AX.spmm;
    float vmax = planner_vmax_for_distance(rem_mm, AX.accel_mm_s2);
    /* vel≈0 is a launch/crawl, not a brake. Stopping-distance + 4 with v=0 is
     * rem<=4, which aborted 1–4 step MT (FRAME_NEXT) with fill_wants_more=false
     * and left the axis in M at v=0 with the target still ahead. */
    bool need_brake = !AX.stopping && !reverse_decel && fabsf(AX.vel_mm_s) > 0.01f &&
                      rem <= planner_stop_rem_steps(fabsf(AX.vel_mm_s), AX.accel_mm_s2, AX.spmm) + 4;
    /*
     * Ramp target is either cruise or 0. Feeding it min(cruise, vmax) would
     * move the target every word as rem shrinks, restart the sine phase, and
     * leave the axis stuck near ramp_start_hz (seen as "higher ss → slower").
     * When remaining distance cannot support the current speed, brake to 0;
     * the vmax clamp below is the hard safety net for the issued word.
     *
     * Use the true stop-distance threshold here instead of a stale
     * speed-vs-vmax trigger so the brake arc starts at the same geometric
     * point that the accel arc ended, avoiding the visible accel/decel skew.
     */
    float v_cmd;
    if (AX.stopping || reverse_decel) {
      v_cmd = 0.0f;
    } else if (AX.braking || need_brake) {
      /* Braking point reached: commit to one ramp down to 0. Re-deriving it per
         word would restart the sine and flatten it into constant 2a/pi. */
      AX.braking = true;
      v_cmd = 0.0f;
    } else {
      v_cmd = (float)sign * cruise_cap(axis);
    }

    bool decel = AX.stopping || reverse_decel || AX.braking;
    if (decel) {
      /* Commit the brake arc once and then track progress along it by distance.
         A soft-stop rem is re-derived from v every word and would drift; keyed
         on distance the profile lands on the target by construction. */
      if (AX.brake_d <= 0) {
        AX.brake_d = rem;
        AX.brake_pos0 = AX.pos_steps;
        AX.brake_v0 = fabsf(AX.vel_mm_s);
      }
      rem = AX.brake_d - (int)llabs(AX.pos_steps - AX.brake_pos0);
      if (rem <= 0) {
        AX.vel_mm_s = 0.0f;
        reset_ramp(axis);
        AX.braking = false;
        AX.brake_d = 0;
        AX.fill_wants_more = false;
        break;
      }
      if (AX.brake_v0 <= 0.01f) {
        /* Committed brake with no speed: remaining distance is a crawl. */
        AX.braking = false;
        AX.brake_d = 0;
        decel = false;
        v_cmd = (float)sign * cruise_cap(axis);
      }
    } else {
      AX.brake_d = 0;
    }

    /* Direction change at zero crossing */
    if (!AX.stopping && AX.last_sign != 0 && sign != AX.last_sign && fabsf(AX.vel_mm_s) < 0.05f) {
      float pause = config_get()->dir_change_pause_s;
      if (pause > 0.0f) {
        AX.dir_pause = true;
        AX.dir_pause_s = pause;
        AX.vel_mm_s = 0.0f;
        reset_ramp(axis);
        AX.braking = false;
        AX.brake_d = 0;
        AX.fill_wants_more = false;
        protocol_debug(4, "D:dir_pause %.3f\n", (double)pause);
        break;
      }
    }

    int ramp_start = config_get()->ramp_start_hz;
    int stop_app = config_get()->stop_approach_hz;
    /*
     * Leave standstill at ramp_start_hz. A sine from v=0 has near-zero initial
     * accel, and the old floor kept STEP stuck at ramp_start while phi crawled
     * up — felt as a long constant low-speed creep (worse with small SA).
     * Seed vmin so the S-curve accelerates immediately toward cruise.
     */
    if (ramp_start > 0 && !AX.stopping && !reverse_decel && !AX.braking) {
      float vmin = (float)ramp_start / AX.spmm;
      if (vmin > 0.0f && fabsf(AX.vel_mm_s) < vmin && fabsf(v_cmd) > vmin) {
        AX.vel_mm_s = (float)sign * vmin;
        reset_ramp(axis);
      }
    }

    begin_ramp(axis, v_cmd);

    /*
     * ramp_start_hz is the *launch* floor only. Applying it while braking pinned
     * STEP at ramp_start for the last d_stop(ramp_start) — the axis then halted
     * from that rate instead of tapering (end-of-move snap). Approaching the
     * target is bounded by stop_approach_hz instead.
     */
    float min_hz = (decel && stop_app > 0) ? (float)stop_app : (float)ramp_start;

    float step_hz_est = fabsf(AX.vel_mm_s) * AX.spmm;
    if (step_hz_est < 1.0f) {
      step_hz_est = min_hz;
    }
    if (!decel && rem > 1 && step_hz_est < (float)ramp_start) {
      step_hz_est = (float)ramp_start;
    }
    /* Approach floor: avoid crawling below stop_approach_hz while braking in. */
    if ((decel || rem <= 4) && stop_app > 0 && step_hz_est > 0.0f &&
        step_hz_est < (float)stop_app) {
      step_hz_est = (float)stop_app;
    }

    /* SM stopped: ignore time-budget pending so we can prefill 4+ words. */
    int pending = pio_step_is_running(axis) ? pio_step_pending_steps(axis) : 0;
    int pack_min = (nax >= 3) ? PLANNER_PACK_MIN_HZ_3AXIS : PLANNER_PACK_MIN_HZ;
    int n = planner_pack_n_min(step_hz_est, rem, pending, pack_min);
    if (n <= 0) {
      /* Enough steps already queued for the time budget. */
      AX.fill_wants_more = false;
      break;
    }
    if (n > rem) {
      n = rem;
    }
    /*
     * Curvature cap while braking: the time-budget pack size only shrinks
     * once close to the target (rem <= remaining/5), so a word entered mid
     * brake-arc (steepest part of the sine, near D/2) could still hold ~64
     * pulses at one fixed rate — a visible speed "step" each time the next
     * word's rate is applied (reported as pumping / un-smooth deceleration).
     * |dv/dR| peaks at pi*dv_total/(2*D) (at R=D/2); bounding n against that
     * worst case keeps the per-word speed change under ~brake_step_frac of
     * the total brake span everywhere on the arc, not just near the target.
     */
    if (decel && AX.brake_d > 0) {
      const float brake_step_frac = 0.06f;
      int n_curv = (int)(brake_step_frac * (2.0f / (float)M_PI) * (float)AX.brake_d);
      if (n_curv < 1) {
        n_curv = 1;
      }
      if (n > n_curv) {
        n = n_curv;
      }
    }

    /* Compute the actual PIO delay for this word from the start-of-word
       rate estimate and derive the real issued duration (dt) for the packed
       pulses. Reuse that delay for the emitted word so phi advancement
       matches the issued timing. */
    uint32_t delay_cycles_for_word =
        planner_hz_to_delay(step_hz_est, pio_step_sysclk_hz(), PIO_STEP_PERIOD_FIXED);
    /* dt (seconds) = n * period; period = (fixed_overhead + delay_cycles)/sysclk */
    double dt = (double)n * ((double)delay_cycles_for_word + (double)PIO_STEP_PERIOD_FIXED) /
                (double)pio_step_sysclk_hz();
    /* One refinement pass: estimate end-of-word velocity and recompute a
       delay from the average rate, improving alignment between phi advance
       and the issued PIO word without heavy iteration. Cover both the accel
       ramp and the brake arc — omitting decel here left every braking word's
       delay keyed on the start-of-word (pre-decel) rate, i.e. the corner
       from steady speed into deceleration ran a whole word too fast before
       snapping to the next word's slower rate. */
    if (n > 1 && decel && AX.brake_d > 0) {
      float v_app_est = (stop_app > 0) ? (float)stop_app / AX.spmm : 0.0f;
      float dv_est = AX.brake_v0 - v_app_est;
      if (dv_est < 0.0f) {
        dv_est = 0.0f;
      }
      int r_after_est = rem - n;
      float v_end_est;
      if (r_after_est <= 0) {
        v_end_est = v_app_est;
      } else {
        float frac_est = (float)M_PI * (float)r_after_est / (2.0f * (float)AX.brake_d);
        v_end_est = v_app_est + dv_est * planner_sinf(frac_est);
      }
      double step_hz_est2 = fabs((double)v_end_est) * (double)AX.spmm;
      if (step_hz_est2 < 1.0) {
        step_hz_est2 = (double)min_hz;
      }
      uint32_t delay2 = planner_hz_to_delay((float)step_hz_est2, pio_step_sysclk_hz(), PIO_STEP_PERIOD_FIXED);
      double dt2 = (double)n * ((double)delay2 + (double)PIO_STEP_PERIOD_FIXED) / (double)pio_step_sysclk_hz();
      if (fabs(dt2 - dt) / (dt + 1e-12) > 0.02) {
        dt = dt2;
        delay_cycles_for_word = delay2;
      }
    } else if (n > 1 && !decel && AX.ramp_active) {
      double phi_est = planner_sine_advance_phi((double)AX.ramp_phi, AX.ramp_v0, AX.ramp_v1, AX.accel_mm_s2, dt);
      double v_est = (double)planner_sine_vel(AX.ramp_v0, AX.ramp_v1, (float)phi_est);
      double step_hz_est2 = fabs(v_est) * (double)AX.spmm;
      if (step_hz_est2 < 1.0) {
        step_hz_est2 = (double)min_hz;
      }
      uint32_t delay2 = planner_hz_to_delay((float)step_hz_est2, pio_step_sysclk_hz(), PIO_STEP_PERIOD_FIXED);
      double dt2 = (double)n * ((double)delay2 + (double)PIO_STEP_PERIOD_FIXED) / (double)pio_step_sysclk_hz();
      /* Accept the refined dt if it differs noticeably. */
      if (fabs(dt2 - dt) / (dt + 1e-12) > 0.02) {
        dt = dt2;
        delay_cycles_for_word = delay2;
      }
    }
    float v_prev = AX.vel_mm_s;
    if (decel) {
      /*
       * Brake speed as a closed form of the distance still to run:
       *   v(R) = v_app + (v0 - v_app) * sin(pi*R / (2*D))
       * Integrating the ramp in time instead let it lag (dphi/ds = 1/(v*T) is
       * stiff as v -> 0); the v_cap clamp then took over and stopped the axis at
       * full accel — the end-of-move snap. Keyed on distance it cannot drift.
       * Landing on v_app (stop_approach) instead of 0 keeps the arc finite: a
       * tail decaying to zero takes exponentially long and crawls on the floor.
       */
      float v_app = (stop_app > 0) ? (float)stop_app / AX.spmm : 0.0f;
      float dv = AX.brake_v0 - v_app;
      if (dv < 0.0f) {
        dv = 0.0f;
      }
      int r_after = rem - n;
      if (r_after <= 0) {
        AX.vel_mm_s = (float)sign * v_app;
      } else {
        float frac = (float)M_PI * (float)r_after / (2.0f * (float)AX.brake_d);
        AX.vel_mm_s = (float)sign * (v_app + dv * planner_sinf(frac));
      }
    } else {
      AX.ramp_phi = (float)planner_sine_advance_phi((double)AX.ramp_phi, AX.ramp_v0, AX.ramp_v1, AX.accel_mm_s2, dt);
      AX.vel_mm_s = planner_sine_vel(AX.ramp_v0, AX.ramp_v1, AX.ramp_phi);
      if (AX.ramp_phi >= 1.0f) {
        AX.vel_mm_s = AX.ramp_v1;
        AX.ramp_active = false;
      }
    }

    /* Clamp to stop distance. While braking this is only an outer safety cap,
       so the committed ramp is not torn down by it. */
    float v_cap = AX.braking ? vmax * 1.25f : vmax;
    if (fabsf(AX.vel_mm_s) > v_cap) {
      AX.vel_mm_s = (AX.vel_mm_s >= 0.0f) ? v_cap : -v_cap;
      if (!AX.braking) {
        reset_ramp(axis);
      }
    }

    float step_hz = fabsf(AX.vel_mm_s) * AX.spmm;
    if (!decel && rem > 1 && step_hz < (float)ramp_start) {
      step_hz = (float)ramp_start;
    }
    if ((decel || rem <= 4) && stop_app > 0 && step_hz > 0.0f &&
        step_hz < (float)stop_app) {
      step_hz = (float)stop_app;
    }
    if (step_hz < 1.0f) {
      if (AX.stopping || reverse_decel || err == 0) {
        AX.vel_mm_s = 0.0f;
        AX.fill_wants_more = false;
        break;
      }
      step_hz = 1.0f;
    }

    /* Use the previously computed delay (based on start-of-word estimate)
       so the PIO word duration aligns with the dt used above. */
    uint32_t delay = delay_cycles_for_word;
    pio_step_set_dir(axis, sign);
    if (!pio_step_put_word(axis, delay, (uint8_t)n)) {
      /* TX full: work remains, so a dry FIFO later would be a real underrun. */
      AX.fill_wants_more = true;
      break;
    }
    pio_step_start_if_ready(axis);
    AX.fill_wants_more = true;
    motion_diag_note_hz(step_hz);
    if (dt > 1e-6f) {
      float a_now = (AX.vel_mm_s - v_prev) / dt;
      AX.acc_meas += 0.25f * (a_now - AX.acc_meas);
    }
    int64_t pos_before = AX.pos_steps;
    AX.pos_steps += (int64_t)n * (int64_t)sign;
    AX.last_sign = sign;
    ++emitted;

    /*
     * Overshoot = this word carried the axis across the target. Being *beyond*
     * the target is normal while the trailing deceleration is issued (err == 0)
     * or while a reverse move bleeds off speed, so only a crossing counts.
     */
    if (!AX.stopping && !reverse_decel && err != 0) {
      bool crossed = (sign > 0 && pos_before <= AX.target_steps && AX.pos_steps > AX.target_steps) ||
                     (sign < 0 && pos_before >= AX.target_steps && AX.pos_steps < AX.target_steps);
      if (crossed) {
        motion_diag_note_overshoot((int)llabs(AX.pos_steps - AX.target_steps));
        protocol_debug(2, "D:overshoot %ld\n", (long)llabs(AX.pos_steps - AX.target_steps));
        AX.pos_steps = AX.target_steps;
        AX.vel_mm_s = 0.0f;
        reset_ramp(axis);
        AX.braking = false;
        AX.brake_d = 0;
        AX.fill_wants_more = false;
        break;
      }
    }
  }
  if (!pio_step_is_running(axis) && !pio_step_tx_empty(axis) &&
      (pio_step_tx_level(axis) >= PIO_STEP_START_MIN_LEVEL || !AX.fill_wants_more)) {
    pio_step_start(axis);
  }
  refresh_state(axis);
  return emitted;
}


void planner_tick_axis(int axis, float dt_s) {
  planner_poll_switches(axis, dt_s);
  home_poll_fsm(axis, dt_s);
  if (AX.dir_pause) {
    AX.dir_pause_s -= dt_s;
    if (AX.dir_pause_s <= 0.0f) {
      AX.dir_pause = false;
      AX.dir_pause_s = 0.0f;
      AX.last_sign = 0;
      pio_step_clear_stall(axis);
      AX.fill_wants_more = true;
      pio_step_kick_feed();
    }
  }

  if (AX.st.moving && !AX.dir_pause &&
      (AX.fill_wants_more || pio_step_pending_steps(axis) > 0) &&
      (pio_step_tx_empty(axis) || pio_step_tx_level(axis) <= PIO_STEP_START_MIN_LEVEL + 2u)) {
    pio_step_kick_feed();
  }

  /*
   * Underrun = mid-burst dry-out, not "SM waited for a word."
   * After sync_shadow_to_fifo, an empty FIFO yields pending==0 — that is the
   * normal gap between fill bursts (see pio_step_is_stalled). Counting that
   * (TXSTALL && empty && fill_wants_more only) prints D:underrun on every
   * planner tick and the USB load causes the stall it claims to report.
   * Require pending>0 *and* empty: issued work gone from the FIFO while still
   * on the books. Do not loosen this gate.
   */
  int pending = pio_step_pending_steps(axis);
  if (AX.st.moving && !AX.dir_pause && pio_step_is_running(axis) && pending > 0 &&
      AX.fill_wants_more && pio_step_tx_empty(axis) && pio_step_is_stalled(axis)) {
    motion_diag_note_underrun(axis);
    static uint32_t ur_burst[AXIS_MAX];
    ++ur_burst[axis];
    if (ur_burst[axis] == 1u || (ur_burst[axis] & 15u) == 0u) {
      protocol_debug(2,
                    "D:underrun a=%d n=%lu tx=%u pend=%d fill=%d run=%d\n",
                    axis, (unsigned long)ur_burst[axis],
                    (unsigned)pio_step_tx_level(axis), pending,
                    AX.fill_wants_more ? 1 : 0, pio_step_is_running(axis) ? 1 : 0);
    }
#ifndef HOST_TEST
#ifdef DEBUG_HW
    dbg_hw_set(PIN_DBG_UNDERRUN, 1);
    dbg_hw_set(PIN_DBG_UNDERRUN, 0);
#endif
#endif
  }

  if (AX.st.moving) {
    pio_step_start_if_ready(axis);
    settle_if_done(axis);
  }
  home_poll_fsm(axis, dt_s);
  refresh_state(axis);
}

void planner_tick(float dt_s) {
  if (dt_s < 0.0f) {
    dt_s = 0.0f;
  }
  for (int axis = 0; axis < axis_count(); ++axis) {
    planner_tick_axis(axis, dt_s);
  }
  int ns = config_servo_count();
  if (g_enabled && ns > 0 && !motion_path_is_active()) {
    for (int s = 0; s < ns; ++s) {
      servo_integrate(s, dt_s);
    }
  }
  if (ns > 0) {
    int due = (int)(g_servo_rr % (unsigned)ns);
    servo_pwm_write_deg(due, g_sv[due].pos);
    ++g_servo_rr;
  }
  coord_clear_if_idle();
}

static bool clamp_target_mm(int axis, float *mm) {
  float mn = axis_hw_window_min(axis);
  float mx = axis_hw_window_max(axis);
  bool ok = true;
  if (!isnan(mn) && *mm < mn) {
    *mm = mn;
    AX.st.at_soft_limit = true;
    ok = false;
  } else if (!isnan(mx) && *mm > mx) {
    *mm = mx;
    AX.st.at_soft_limit = true;
    ok = false;
  } else {
    AX.st.at_soft_limit = false;
  }
  return ok;
}

void planner_request_move_to(int axis, float mm) {
  if (axis < 0 || axis >= axis_count() || !g_enabled || g_drv_error) {
    return;
  }
  clamp_target_mm(axis, &mm);
  AX.spmm = axis_hw_steps_per_unit(axis);
  float pos_mm = (float)AX.pos_steps / AX.spmm;
  int sign = 0;
  if (mm > pos_mm + 1e-6f) {
    sign = 1;
  } else if (mm < pos_mm - 1e-6f) {
    sign = -1;
  }
  if (planner_hard_limit_blocks_sign(axis, sign)) {
    return;
  }
  AX.target_steps = mm_to_steps(axis, mm);
  /* Sub-step MT (or overlapping MT to the live pose) can round to 0 steps.
   * Arming moving with an empty PIO and a leftover cruise sticks in M: fill
   * never issues words, settle used to require vel≈0, and the UI shows speed
   * with no position change until MS. */
  if (AX.target_steps == AX.pos_steps) {
    if (pio_axis_drained(axis)) {
      AX.st.has_target = true;
      AX.st.homing = false;
      settle_idle_at_pose(axis);
      return;
    }
    planner_request_stop_axis(axis);
    return;
  }
  AX.stopping = false;
  AX.st.moving = true;
  AX.st.homing = false;
  AX.st.has_target = true;
  begin_ramp(axis, AX.vel_mm_s);
  /* Warm-start the feed loop at motion launch so the first refill burst does
   * not wait for a later low-water wake-up. This is the critical edge for the
   * 3-axis startup FIFO handoff. */
  AX.fill_wants_more = true;
  AX.acc_meas = 0.0f;
  AX.braking = false;
  AX.brake_d = 0;
  pio_step_clear_stall(axis);
  pio_step_kick_feed();
  refresh_state(axis);
}

void planner_request_move_by(int axis, float mm) {
  float dest = (float)AX.pos_steps / AX.spmm + mm;
  planner_request_move_to(axis, dest);
}

static void planner_request_joy(int axis, float signed_v) {
  if (axis < 0 || axis >= axis_count() || !g_enabled || g_drv_error) {
    return;
  }
  AX.spmm = axis_hw_steps_per_unit(axis);
  if (AX.spmm < 1e-3f) {
    AX.spmm = 1.0f;
  }
  AX.accel_mm_s2 = clamp_accel_axis(axis, session_get()->accel_mm_s2);

  if (fabsf(signed_v) < 1e-4f) {
    if (AX.st.moving && !AX.stopping) {
      planner_request_stop_axis(axis);
    }
    return;
  }

  int sign = signed_v > 0.0f ? 1 : -1;
  if (planner_hard_limit_blocks_sign(axis, sign)) {
    if (AX.st.moving && !AX.stopping) {
      planner_request_stop_axis(axis);
    }
    return;
  }

  int64_t dest_steps = (sign > 0) ? soft_max_steps(axis) : soft_min_steps(axis);
  if ((sign > 0 && AX.pos_steps >= dest_steps) || (sign < 0 && AX.pos_steps <= dest_steps)) {
    if (AX.st.moving && !AX.stopping) {
      planner_request_stop_axis(axis);
    }
    return;
  }

  float cruise = fabsf(signed_v);
  bool same = AX.st.moving && !AX.stopping && AX.target_steps == dest_steps &&
              fabsf(AX.cruise_mm_s - cruise) < 1e-4f;
  AX.cruise_mm_s = cruise;
  if (same) {
    return;
  }

  AX.target_steps = dest_steps;
  AX.stopping = false;
  AX.st.moving = true;
  AX.st.homing = false;
  AX.st.has_target = true;
  AX.fill_wants_more = true;
  AX.acc_meas = 0.0f;
  AX.braking = false;
  AX.brake_d = 0;
  begin_ramp(axis, (float)sign * cruise_cap(axis));
  pio_step_clear_stall(axis);
  pio_step_kick_feed();
  refresh_state(axis);
}

static void planner_request_stop_axis(int axis) {
  if (AX.st.homing) {
    AX.home_phase = HOME_IDLE;
    AX.st.homing = false;
    home_restore_speeds(axis);
  }
  AX.stopping = true;
  AX.dir_pause = false;
  AX.dir_pause_s = 0.0f;
  AX.st.has_target = false;
  AX.target_steps = AX.pos_steps;
  begin_ramp(axis, 0.0f);
  if (!AX.st.moving) {
    AX.vel_mm_s = 0.0f;
    pio_step_stop_soft(axis);
  } else {
    AX.st.moving = true;
    pio_step_kick_feed();
  }
  refresh_state(axis);
}

void planner_request_stop(void) {
  for (int axis = 0; axis < axis_count(); ++axis) {
    planner_request_stop_axis(axis);
  }
  int ns = config_servo_count();
  for (int s = 0; s < ns; ++s) {
    servo_request_stop(s);
  }
}

void planner_request_halt(void) { planner_halt_all(); }

void planner_set_position(int axis, float mm) {
  if (axis < 0 || axis >= axis_count()) {
    return;
  }
  AX.spmm = axis_hw_steps_per_unit(axis);
  if (AX.spmm < 1e-3f) {
    AX.spmm = 1.0f;
  }
  AX.pos_steps = mm_to_steps(axis, mm);
  AX.target_steps = AX.pos_steps;
  AX.st.pos[0] = (float)AX.pos_steps / AX.spmm;
  AX.st.target[0] = AX.st.pos[0];
  refresh_state(axis);
}

void planner_takeover_from_path(int axis, int64_t pos_steps, float vel_mm_s) {
  if (axis < 0 || axis >= AXIS_MAX) {
    return;
  }
  joy_clear();
  AX.pos_steps = pos_steps;
  AX.target_steps = pos_steps;
  AX.vel_mm_s = vel_mm_s;
  AX.last_sign = (vel_mm_s >= 0.0f) ? 1 : -1;
  AX.st.moving = true;
  AX.st.homing = false;
  AX.st.has_target = false;
  reset_ramp(axis);
  refresh_state(axis);
}

void planner_path_set_servo(int s, float deg, float vel_deg_s) {
  if (s < 0 || s >= SERVO_MAX) {
    return;
  }
  g_sv[s].pos = deg;
  g_sv[s].vel = vel_deg_s;
  g_sv[s].target = deg;
  g_sv[s].stopping = false;
  g_sv[s].moving = fabsf(vel_deg_s) > 0.01f;
  g_sv[s].has_target = g_sv[s].moving;
}

void planner_request_home(int axis) {
  if (axis < 0 || axis >= axis_count() || !g_enabled || g_drv_error) {
    return;
  }
  if (axis_hw_home_mode(axis) == 0 || !home_cfg_ok(axis)) {
    return;
  }

  AX.spmm = axis_hw_steps_per_unit(axis);
  if (AX.spmm < 1e-3f) {
    AX.spmm = 1.0f;
  }

  if (!AX.home_speeds_saved) {
    AX.home_saved_cruise = AX.cruise_mm_s;
    AX.home_saved_accel = AX.accel_mm_s2;
    AX.home_speeds_saved = true;
  }
  AX.cruise_mm_s = axis_hw_home_speed(axis);
  AX.accel_mm_s2 = axis_hw_home_accel(axis);
  if (AX.cruise_mm_s < 0.001f) {
    AX.cruise_mm_s = 0.001f;
  }
  if (AX.accel_mm_s2 < 0.001f) {
    AX.accel_mm_s2 = 0.001f;
  }

  AX.stopping = false;
  AX.st.homing = true;
  AX.home_sign = home_seek_sign(axis);

  if (stall_home_mode(axis)) {
    if (AX.drv_err_stable) {
      home_begin_stall_reset(axis);
    } else {
      home_begin_seek(axis);
    }
  } else if (home_ref_asserted(axis)) {
    home_begin_backoff(axis);
  } else if (AX.hl_l_stable || AX.hl_l_latched) {
    home_begin_clear(axis, +1);
  } else if (AX.hl_r_stable || AX.hl_r_latched) {
    home_begin_clear(axis, -1);
  } else {
    home_begin_seek(axis);
  }
  refresh_state(axis);
}

void planner_soft_reset(void) {
  g_drv_error = false;
  joy_clear();
  coord_clear();
  for (int axis = 0; axis < axis_count(); ++axis) {
    AX.st.drv_error = false;
    AX.drv_err_stable = false;
    AX.drv_err_timer_s = 0.0f;
    AX.stopping = false;
    AX.st.moving = false;
    AX.st.homing = false;
    AX.home_phase = HOME_IDLE;
    home_restore_speeds(axis);
    AX.vel_mm_s = 0.0f;
    AX.target_steps = AX.pos_steps;
    AX.hl_l_latched = false;
    AX.hl_r_latched = false;
    reset_ramp(axis);
    pio_step_stop_hard(axis);
    refresh_state(axis);
  }
  int ns = config_servo_count();
  for (int s = 0; s < ns; ++s) {
    servo_halt(s);
  }
}

bool planner_is_busy(void) {
  for (int axis = 0; axis < axis_count(); ++axis) {
    if (AX.st.moving || AX.st.homing) {
      return true;
    }
  }
  int ns = config_servo_count();
  for (int s = 0; s < ns; ++s) {
    if (g_sv[s].moving) {
      return true;
    }
  }
  return false;
}

bool planner_is_moving(void) {
  for (int axis = 0; axis < axis_count(); ++axis) {
    if (AX.st.moving) {
      return true;
    }
  }
  int ns = config_servo_count();
  for (int s = 0; s < ns; ++s) {
    if (g_sv[s].moving) {
      return true;
    }
  }
  return false;
}

bool planner_feed_active_axis(int axis) {
  if (axis < 0 || axis >= axis_count()) {
    return false;
  }
  /* The tail of a move still has queued step work even after the planner has
   * stopped raising fill_wants_more. Keep the feed task alive until the last
   * pending step has actually drained out of the FIFO/shadow bookkeeping; this
   * avoids the last-word stall during the return-to-zero decel edge. */
  bool pending_steps = pio_step_pending_steps(axis) > 0;
  return (AX.st.moving || AX.stopping || AX.braking || AX.fill_wants_more || pending_steps) &&
         !AX.dir_pause && !g_drv_error && g_enabled;
}

bool planner_feed_active(void) {
  for (int axis = 0; axis < axis_count(); ++axis) {
    if (planner_feed_active_axis(axis)) {
      return true;
    }
  }
  return false;
}

static McState merge_state(void) {
  if (g_drv_error) {
    return MC_STATE_ERROR;
  }
  bool any_hl = false, any_homing = false, any_moving = false;
  bool any_accel = false, any_decel = false;
  for (int axis = 0; axis < axis_count(); ++axis) {
    refresh_state(axis);
    if (AX.st.hard_limit && !AX.st.moving) {
      any_hl = true;
    }
    if (AX.st.homing) {
      any_homing = true;
    }
    if (AX.st.moving) {
      any_moving = true;
      if (AX.st.state == MC_STATE_ACCELERATING) {
        any_accel = true;
      }
      if (AX.st.state == MC_STATE_DECELERATING) {
        any_decel = true;
      }
    }
  }
  if (any_hl && !any_moving && !any_homing) {
    return MC_STATE_HARD_LIMIT;
  }
  if (!g_enabled) {
    return MC_STATE_DISABLED;
  }
  if (any_homing) {
    return MC_STATE_HOMING;
  }
  if (any_decel) {
    return MC_STATE_DECELERATING;
  }
  if (any_accel) {
    return MC_STATE_ACCELERATING;
  }
  if (any_moving) {
    return MC_STATE_MOVING;
  }
  int ns = config_servo_count();
  for (int s = 0; s < ns; ++s) {
    if (g_sv[s].moving) {
      if (g_sv[s].stopping) {
        return MC_STATE_DECELERATING;
      }
      return MC_STATE_MOVING;
    }
  }
  return MC_STATE_IDLE;
}

void planner_get_status(McStatus *out) {
  if (!out) {
    return;
  }
  memset(out, 0, sizeof(*out));
  out->enabled = g_enabled;
  out->drv_error = g_drv_error;
  out->state = merge_state();
  int nm = axis_count();
  int ns = config_servo_count();
  for (int axis = 0; axis < nm; ++axis) {
    refresh_state(axis);
    float sp = AX.spmm > 1e-3f ? AX.spmm : 1.0f;
    out->pos[axis] = (float)AX.pos_steps / sp;
    out->target[axis] = AX.st.target[0];
    out->vel[axis] = AX.st.vel[0];
    out->acc[axis] = AX.st.acc[0];
    out->homing = out->homing || AX.st.homing;
    out->hard_limit = out->hard_limit || AX.st.hard_limit;
    out->has_target = out->has_target || AX.st.has_target;
  }
  for (int s = 0; s < ns; ++s) {
    int ch = nm + s;
    out->pos[ch] = g_sv[s].pos;
    out->target[ch] = g_sv[s].target;
    out->vel[ch] = g_sv[s].vel;
    out->acc[ch] = g_sv[s].moving ? fabsf(g_sv[s].accel) : 0.0f;
    out->has_target = out->has_target || g_sv[s].has_target;
  }
  out->moving = planner_is_moving();
  if (nm > 0) {
    out->at_soft_limit = g_ax[0].st.at_soft_limit;
  }
}

#ifndef HOST_TEST

#include "motion_api.h"

void motion_init(void) {
  static bool once;
  if (once) {
    return;
  }
  once = true;
  config_init_defaults();
  (void)config_axis_count();
  planner_init();
  motion_path_init();
}

bool motion_enable(bool on) {
  if (on && !pio_step_ok()) {
    return false;
  }
  g_enabled = on;
  for (int axis = 0; axis < axis_count(); ++axis) {
    AX.st.enabled = on;
  }
  servo_pwm_set_enabled(on);
  if (!on) {
    joy_clear();
    planner_request_stop();
    coord_clear();
    for (int axis = 0; axis < axis_count(); ++axis) {
      pio_step_stop_hard(axis);
      AX.st.moving = false;
      AX.vel_mm_s = 0.0f;
      refresh_state(axis);
    }
    int ns = config_servo_count();
    for (int s = 0; s < ns; ++s) {
      servo_halt(s);
    }
    apply_en_output(false);
  } else {
    apply_en_output(true);
    for (int axis = 0; axis < axis_count(); ++axis) {
      refresh_state(axis);
    }
  }
  return true;
}

void motion_on_counts_changed(void) {
  servo_boot_all();
}

bool motion_move_to_n(const float dest_in[MC_CH_MAX]) {
  if (!g_enabled) {
    return false;
  }
  float dest[MC_CH_MAX];
  bool want[MC_CH_MAX];
  float dist[MC_CH_MAX];
  int n = ch_count();
  int nm = axis_count();
  int movers = 0;
  int master = -1;

  for (int ch = 0; ch < MC_CH_MAX; ++ch) {
    dest[ch] = dest_in[ch];
    want[ch] = false;
    dist[ch] = 0.0f;
  }

  for (int ch = 0; ch < n; ++ch) {
    if (isnan(dest[ch])) {
      continue;
    }
    float mn = session_effective_left(ch);
    float mx = session_effective_right(ch);
    if ((!isnan(mn) && dest[ch] < mn) || (!isnan(mx) && dest[ch] > mx)) {
      if (ch < nm) {
        g_ax[ch].st.at_soft_limit = true;
      }
      return false;
    }
    dist[ch] = dest[ch] - ch_pos(ch);
    if (fabsf(dist[ch]) < 1e-6f) {
      continue;
    }
    if (ch < nm && planner_hard_limit_blocks_sign(ch, dist[ch] > 0 ? 1 : -1)) {
      return false;
    }
    want[ch] = true;
    ++movers;
  }
  /* First motor with a distance, else first servo with a distance. */
  for (int ch = 0; ch < n; ++ch) {
    if (want[ch]) {
      master = ch;
      break;
    }
  }
  if (movers == 0) {
    joy_clear();
    coord_clear();
    return true;
  }

  joy_clear();
  g_move_master = master;
  if (movers >= 2) {
    g_coord_active = true;
    g_coord_master = master;
    float dmaster = fabsf(dist[master]);
    if (dmaster < 1e-6f) {
      dmaster = 1e-6f;
    }
    for (int ch = 0; ch < n; ++ch) {
      g_coord_ratio[ch] = want[ch] ? (fabsf(dist[ch]) / dmaster) : 0.0f;
    }
    apply_session_cruise_accel();
  } else {
    g_coord_active = false;
    g_coord_master = master;
    ch_set_cruise_accel(master, clamp_speed_ch(master, session_get()->speed_mm_s),
                        clamp_accel_ch(master, session_get()->accel_mm_s2));
  }

  for (int ch = 0; ch < n; ++ch) {
    if (!want[ch]) {
      continue;
    }
    if (ch < nm) {
      planner_request_move_to(ch, dest[ch]);
    } else {
      servo_request_move(ch - nm, dest[ch]);
    }
  }
  return true;
}

bool motion_move_to(float mm) {
  float dest[MC_CH_MAX];
  for (int i = 0; i < MC_CH_MAX; ++i) {
    dest[i] = NAN;
  }
  dest[0] = mm;
  return motion_move_to_n(dest);
}

bool motion_move_to2(float mm1_or_nan, float mm2_or_nan) {
  float dest[MC_CH_MAX];
  for (int i = 0; i < MC_CH_MAX; ++i) {
    dest[i] = NAN;
  }
  dest[0] = mm1_or_nan;
  dest[1] = mm2_or_nan;
  return motion_move_to_n(dest);
}

bool motion_move_by(float mm) {
  float dest[MC_CH_MAX];
  for (int i = 0; i < MC_CH_MAX; ++i) {
    dest[i] = NAN;
  }
  dest[0] = ch_pos(0) + mm;
  return motion_move_to_n(dest);
}

bool motion_joy(const float pct_in[MC_CH_MAX]) {
  if (!g_enabled) {
    return false;
  }
  coord_clear();
  g_joy_active = true;
  int n = ch_count();
  int nm = axis_count();
  for (int ch = 0; ch < MC_CH_MAX; ++ch) {
    float p = pct_in[ch];
    if (ch >= n || isnan(p) || fabsf(p) < 1e-3f) {
      p = 0.0f;
    }
    g_joy_pct[ch] = p;
    if (ch < n) {
      float signed_v = signed_cruise_from_pct(ch, p);
      if (ch < nm) {
        planner_request_joy(ch, signed_v);
      } else {
        servo_request_joy(ch - nm, signed_v);
      }
    }
  }
  return true;
}

void motion_end_joy(void) { joy_clear(); }

bool motion_stop(void) {
  joy_clear();
  planner_request_stop();
  return true;
}

bool motion_halt(void) {
  planner_request_halt();
  return true;
}

bool motion_home(int axis_1based) {
  if (!g_enabled) {
    return false;
  }
  int axis = axis_1based - 1;
  if (axis < 0 || axis >= axis_count()) {
    return false;
  }
  if (axis_hw_home_mode(axis) == 0) {
    joy_clear();
    return true;
  }
  if (!home_cfg_ok(axis)) {
    return false;
  }
  joy_clear();
  g_move_master = axis;
  planner_request_home(axis);
  return true;
}

bool motion_soft_reset(void) {
  planner_soft_reset();
  return true;
}

bool motion_set_position(const float mm[MC_CH_MAX]) {
  if (motion_path_is_active() || planner_is_busy()) {
    return false;
  }
  int n = ch_count();
  int nm = axis_count();
  for (int ch = 0; ch < n; ++ch) {
    if (isnan(mm[ch])) {
      continue;
    }
    if (ch < nm) {
      planner_set_position(ch, mm[ch]);
    } else {
      int s = ch - nm;
      g_sv[s].pos = mm[ch];
      g_sv[s].target = mm[ch];
      g_sv[s].vel = 0.0f;
      g_sv[s].moving = false;
      g_sv[s].has_target = false;
      g_sv[s].stopping = false;
    }
  }
  return true;
}

bool motion_set_speed(float mm_s) {
  session_get()->speed_mm_s = mm_s;
  if (g_joy_active) {
    apply_joy_cruise_accel();
  } else {
    apply_session_cruise_accel();
  }
  return true;
}

bool motion_set_accel(float mm_s2) {
  session_get()->accel_mm_s2 = mm_s2;
  if (g_joy_active) {
    apply_joy_cruise_accel();
  } else {
    apply_session_cruise_accel();
  }
  return true;
}

bool motion_set_max_speed(float mm_s) {
  config_get()->max_speed_mm_s = mm_s;
  return true;
}

bool motion_set_window_left(const bool set[MC_CH_MAX], const float mm[MC_CH_MAX]) {
  return session_set_window_left(set, mm);
}

bool motion_set_window_right(const bool set[MC_CH_MAX], const float mm[MC_CH_MAX]) {
  return session_set_window_right(set, mm);
}

void motion_reset_window_left(void) { session_reset_left(); }

void motion_reset_window_right(void) { session_reset_right(); }

int motion_master_channel(void) {
  if (!planner_is_busy()) {
    return -1;
  }
  if (g_move_master >= 0) {
    return g_move_master;
  }
  return g_coord_active ? g_coord_master : 0;
}

void motion_get_status(McStatus *out) {
  if (motion_path_is_active()) {
    motion_path_get_status(out);
    return;
  }
  planner_get_status(out);
}

bool motion_is_busy(void) { return planner_is_busy(); }

void motion_stub_tick_ms(unsigned ms) { (void)ms; }

#endif /* !HOST_TEST */
