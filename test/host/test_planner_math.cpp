#include "planner_math.h"
#include "pio_step.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>

static int g_fail;

static void expect_true(const char *name, bool ok) {
  if (!ok) {
    std::fprintf(stderr, "FAIL %s\n", name);
    ++g_fail;
  } else {
    std::printf("OK   %s\n", name);
  }
}

static void expect_near(const char *name, float a, float b, float eps) {
  expect_true(name, std::fabs(a - b) <= eps);
}

int main(void) {
  /* stop distance invert: d = pi v^2 / (4a) => v = sqrt(4 a d / pi) */
  float a = 200.0f;
  float v = 50.0f;
  float d = (float)M_PI * v * v / (4.0f * a);
  float vmax = planner_vmax_for_distance(d, a);
  expect_near("vmax_for_distance roundtrip", vmax, v, 0.05f);
  expect_near("vmax zero dist", planner_vmax_for_distance(0.0f, a), 0.0f, 1e-6f);

  /* pack_n pendeln guard */
  expect_true("pack rem 0", planner_pack_n(100.0f, 0, 0) == 0);
  expect_true("pack rem neg", planner_pack_n(100.0f, -3, 0) == 0);
  expect_true("pack low hz single", planner_pack_n(100.0f, 10, 0) == 1);
  int n = planner_pack_n(50000.0f, 100, 0);
  expect_true("pack high hz bounded", n >= 1 && n <= 64 && n <= 100);
  expect_true("pack never past rem", planner_pack_n(50000.0f, 3, 0) <= 3);

  /* Simulated fill must not pass target (pendeln regression) */
  int pos = 0;
  int target = 1000;
  float hz = 5000.0f;
  for (int i = 0; i < 5000; ++i) {
    int rem = target - pos;
    int pn = planner_pack_n(hz, rem, 0);
    if (pn <= 0) {
      break;
    }
    pos += pn;
  }
  expect_true("no overshoot in pack loop", pos == target);

  /* Max rate with period = 192 + delay @ 125 MHz */
  float max_hz = planner_max_step_hz(125000000u, PIO_STEP_PERIOD_FIXED);
  expect_true("max hz >= 300k", max_hz >= 300000.0f);
  std::printf("     max_step_hz=%.0f\n", (double)max_hz);

  uint32_t delay = planner_hz_to_delay(300000.0f, 125000000u, PIO_STEP_PERIOD_FIXED);
  expect_true("delay at 300k >= 1", delay >= 1);

  /* Sine ramp endpoints */
  expect_near("sine phi0", planner_sine_vel(0.0f, 10.0f, 0.0f), 0.0f, 1e-5f);
  expect_near("sine phi1", planner_sine_vel(0.0f, 10.0f, 1.0f), 10.0f, 1e-5f);
  float mid = planner_sine_vel(0.0f, 10.0f, 0.5f);
  expect_near("sine mid", mid, 5.0f, 0.05f);

  /* pack-dt invariant: phi advance scales with packed pulse count n */
  {
    float v0 = 0.0f;
    float v1 = 50.0f;
    float a = 200.0f;
    float step_hz = 16000.0f; /* above PLANNER_PACK_MIN_HZ */
    double dt1 = 1.0 / (double)step_hz;
    double dtn = 64.0 / (double)step_hz;
    double phi1 = planner_sine_advance_phi(0.0, v0, v1, a, dt1);
    double phin = planner_sine_advance_phi(0.0, v0, v1, a, dtn);
    expect_true("pack-dt n advances more than 1", phin > phi1 * 10.0);
    expect_true("pack-dt n not instantly done", phin < 1.0);
  }

  /* reverse_decel helpers */
  expect_true("reverse needs decel", planner_needs_reverse_decel(1, -1, 10.0f));
  expect_true("same sign no reverse", !planner_needs_reverse_decel(1, 1, 10.0f));
  expect_true("slow no reverse", !planner_needs_reverse_decel(1, -1, 0.01f));
  expect_true("no last sign", !planner_needs_reverse_decel(0, -1, 10.0f));

  int rem_stop = planner_stop_rem_steps(50.0f, 200.0f, 320.0f);
  float d_stop = (float)M_PI * 50.0f * 50.0f / (4.0f * 200.0f);
  int rem_expect = (int)std::ceil(d_stop * 320.0f);
  expect_true("stop_rem matches d_stop", rem_stop == rem_expect);
  expect_true("stop_rem zero vel", planner_stop_rem_steps(0.0f, 200.0f, 320.0f) == 0);

  /* Live retarget idea: remaining shrinks when target moves closer */
  int pos2 = 100;
  int tgt_a = 500;
  int tgt_b = 120;
  int rem_a = tgt_a - pos2;
  int rem_b = tgt_b - pos2;
  expect_true("retarget reduces rem", rem_b < rem_a && rem_b > 0);

  if (g_fail) {
    std::fprintf(stderr, "\n%d planner math test(s) failed\n", g_fail);
    return 1;
  }
  std::puts("\nAll planner math tests passed.");
  return 0;
}
