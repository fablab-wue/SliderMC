#include "motion_path.h"
#include "motion_api.h"
#include "config_store.h"
#include "config_defaults.h"

#include <cmath>
#include <cstdio>

static int g_fail;

static void expect_true(const char *name, bool ok) {
  if (!ok) {
    std::fprintf(stderr, "FAIL %s\n", name);
    ++g_fail;
  } else {
    std::printf("OK   %s\n", name);
  }
}

int main(void) {
  config_init_defaults();
  motion_init();

  /* --- error-diffusion: steps track distance over many slices --- */
  {
    double step_err = 0.0;
    float steps_per_mm = 320.0f;
    int16_t distance_um = 333; /* 0.333 mm -> 106.56 steps/slice, non-integer */
    int32_t total_steps = 0;
    const int n = 10000;
    for (int i = 0; i < n; ++i) {
      total_steps += motion_path_diffuse_steps(distance_um, steps_per_mm, &step_err);
    }
    double expected = (double)distance_um / 1000.0 * steps_per_mm * n;
    expect_true("step diffusion converges", std::fabs((double)total_steps - expected) < 1.0);
  }

  /* --- zero distance always yields 0 steps and does not touch the carry --- */
  {
    double step_err = 0.5;
    int32_t s = motion_path_diffuse_steps(0, 320.0f, &step_err);
    expect_true("zero distance -> 0 steps", s == 0);
    expect_true("zero distance leaves carry untouched", step_err == 0.5);
  }

  /* --- error-diffusion: cycles track total slice time --- */
  {
    double time_err = 0.0;
    uint32_t slice_us = 10000; /* 10 ms */
    uint32_t sysclk = 125000000u;
    uint64_t total_cycles = 0;
    const int n = 1000;
    for (int i = 0; i < n; ++i) {
      total_cycles += motion_path_diffuse_cycles(slice_us, sysclk, &time_err);
    }
    double expected = (double)slice_us * ((double)sysclk / 1e6) * n;
    expect_true("time diffusion converges",
                std::fabs((double)total_cycles - expected) < 1.0);
  }

  /* --- buffer add/clear/count roundtrip + full rejection --- */
  {
    expect_true("clear ok when idle", motion_path_clear());
    expect_true("count 0 after clear", motion_path_count() == 0);
    config_set_key("path_buffer_size", "3");
    expect_true("add 1", motion_path_add(100));
    expect_true("add 2", motion_path_add(-200));
    expect_true("add 3", motion_path_add(0));
    expect_true("count is 3", motion_path_count() == 3);
    expect_true("add 4 rejected (full)", !motion_path_add(1));
    config_set_key("path_buffer_size", "64000");
  }

  /* --- slice: min bound + bare reset --- */
  {
    expect_true("slice reject below min", !motion_path_set_slice_us(500));
    expect_true("slice accept at min", motion_path_set_slice_us(1000));
    expect_true("slice get reflects set", motion_path_get_slice_us() == 1000);
    session_reset_path_slice();
    expect_true("slice reset to config default",
                motion_path_get_slice_us() == (uint32_t)config_get()->init_path_slice_us);
  }

  /* --- go(): gating on empty/disabled, and re-entrancy --- */
  {
    motion_path_clear();
    expect_true("go rejected when empty", !motion_path_go());
    motion_path_add(50);
    motion_enable(false);
    expect_true("go rejected when disabled", !motion_path_go());
    motion_enable(true);
    expect_true("go accepted", motion_path_go());
    expect_true("is_active true after go", motion_path_is_active());
    expect_true("go rejected while already active", !motion_path_go());
    expect_true("add allowed while active (live-move streaming)", motion_path_add(10));
    expect_true("count reflects live-streamed add", motion_path_count() == 2);
    motion_path_abort_to_planner();
    expect_true("is_active false after abort", !motion_path_is_active());
  }

  if (g_fail) {
    std::fprintf(stderr, "%d FAILURES\n", g_fail);
    return 1;
  }
  std::printf("All motion_path tests passed.\n");
  return 0;
}
