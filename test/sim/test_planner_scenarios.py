"""Host unit tests: planner fill/PIO twin across speed×accel scenario matrix."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

# Allow `python -m unittest test.sim.test_planner_scenarios` from repo root.
_ROOT = Path(__file__).resolve().parents[2]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from test.sim.planner_model import (  # noqa: E402
    ScenarioResult,
    expected_triangle_time,
    halfway_time,
    run_scenario,
    vmax_for_distance,
)

SPEEDS = (5.0, 20.0, 50.0, 100.0)
ACCELS = (5.0, 10.0, 50.0, 200.0)


def _has_overshoot(res: ScenarioResult) -> bool:
    return any("OVERSHOOT" in m for _, _, _, m in res.events)


def _has_dir_pause(res: ScenarioResult) -> bool:
    return any(m == "dir_pause" for _, _, _, m in res.events)


def _assert_common(tc: unittest.TestCase, res: ScenarioResult, cruise: float, accel: float,
                   dist_for_peak: float = 100.0) -> None:
    tc.assertFalse(res.skipped, res.name)
    tc.assertTrue(res.ok, "%s: did not settle cleanly (%s)" % (res.name, res.detail))
    tc.assertLess(res.pos_err, 0.05, "%s: pos err" % res.name)
    tc.assertFalse(_has_overshoot(res), "%s: OVERSHOOT event" % res.name)
    tc.assertLess(res.t, 120.0, "%s: timed out" % res.name)

    v_phys = vmax_for_distance(dist_for_peak * 0.5, accel)
    v_cap = max(cruise, v_phys) * 1.15 + 0.5
    tc.assertLessEqual(res.peak_v, v_cap + 1e-3,
                       "%s: peak_v=%.2f > cap=%.2f" % (res.name, res.peak_v, v_cap))

    if v_phys >= cruise * 0.99:
        tc.assertGreaterEqual(res.peak_v, cruise * 0.9 - 1e-3,
                              "%s: peak_v=%.2f < 0.9*cruise=%.2f" %
                              (res.name, res.peak_v, cruise * 0.9))


class PlannerScenarioMatrix(unittest.TestCase):
    """Normal moves + retarget / speed-change across SPEED×ACCEL."""

    def test_matrix(self):
        fails = []
        passed = 0
        skipped = 0

        for cruise in SPEEDS:
            for accel in ACCELS:
                tag = "ss%g/sa%g" % (cruise, accel)
                t_half = halfway_time(cruise, accel, 100.0)

                # --- Normal mt100 ---
                name = "normal %s" % tag
                res = run_scenario(
                    name, cruise, accel,
                    [(0.0, lambda s: s.move_to(100.0))])
                try:
                    _assert_common(self, res, cruise, accel)
                    print("PASS %s %s" % (name, res.detail))
                    passed += 1
                except AssertionError as e:
                    print("FAIL %s %s — %s" % (name, res.detail, e))
                    fails.append(str(e))

                # --- Normal return mt100 then mt0 ---
                name = "return %s" % tag
                # Two-phase: finish mt100, then mt0 from idle.
                res1 = run_scenario(
                    "normal-leg %s" % tag, cruise, accel,
                    [(0.0, lambda s: s.move_to(100.0))])
                try:
                    _assert_common(self, res1, cruise, accel)
                except AssertionError as e:
                    print("FAIL %s (leg1) — %s" % (name, e))
                    fails.append(str(e))
                    continue

                # Restart from 100: seed by running mt0 with a custom start.
                def _start_from_100_to_0(s, c=cruise, a=accel):
                    s.pos = int(round(100.0 * 320.0))
                    s.cruise = c
                    s.accel = a
                    s.move_to(0.0)

                res = run_scenario(
                    name, cruise, accel,
                    [(0.0, _start_from_100_to_0)])
                try:
                    _assert_common(self, res, cruise, accel)
                    self.assertAlmostEqual(res.target_mm, 0.0, places=2)
                    print("PASS %s %s" % (name, res.detail))
                    passed += 1
                except AssertionError as e:
                    print("FAIL %s %s — %s" % (name, res.detail, e))
                    fails.append(str(e))

                # --- Reverse retarget ---
                name = "reverse %s" % tag
                retarget_v_box = [0.0]

                def _rev_at(s, th=t_half):
                    retarget_v_box[0] = abs(s.vel)
                    s.move_to(0.0)

                res = run_scenario(
                    name, cruise, accel,
                    [(0.0, lambda s: s.move_to(100.0)),
                     (t_half, _rev_at)])
                try:
                    _assert_common(self, res, cruise, accel, dist_for_peak=100.0)
                    self.assertAlmostEqual(res.target_mm, 0.0, places=2)
                    if retarget_v_box[0] > 0.5:
                        self.assertTrue(_has_dir_pause(res),
                                        "%s: expected dir_pause (v=%.2f at retarget)" %
                                        (name, retarget_v_box[0]))
                    print("PASS %s %s" % (name, res.detail))
                    passed += 1
                except AssertionError as e:
                    print("FAIL %s %s — %s" % (name, res.detail, e))
                    fails.append(str(e))

                # --- Forward near mt50 ---
                name = "fwd_near %s" % tag
                pos_at = [0.0]

                def _near(s):
                    pos_at[0] = s.pos / 320.0
                    s.move_to(50.0)

                res = run_scenario(
                    name, cruise, accel,
                    [(0.0, lambda s: s.move_to(100.0)),
                     (t_half, _near)])
                if pos_at[0] >= 50.0 - 1e-3:
                    print("SKIP %s (already past 50 at t=%.2f pos=%.2f)" %
                          (name, t_half, pos_at[0]))
                    skipped += 1
                else:
                    try:
                        _assert_common(self, res, cruise, accel)
                        self.assertAlmostEqual(res.target_mm, 50.0, places=2)
                        print("PASS %s %s" % (name, res.detail))
                        passed += 1
                    except AssertionError as e:
                        print("FAIL %s %s — %s" % (name, res.detail, e))
                        fails.append(str(e))

                # --- Forward far mt150 ---
                name = "fwd_far %s" % tag
                res = run_scenario(
                    name, cruise, accel,
                    [(0.0, lambda s: s.move_to(100.0)),
                     (t_half, lambda s: s.move_to(150.0))])
                try:
                    # Peak may reflect the longer 150 mm move.
                    _assert_common(self, res, cruise, accel, dist_for_peak=150.0)
                    self.assertAlmostEqual(res.target_mm, 150.0, places=2)
                    print("PASS %s %s" % (name, res.detail))
                    passed += 1
                except AssertionError as e:
                    print("FAIL %s %s — %s" % (name, res.detail, e))
                    fails.append(str(e))

                # --- Speed up (to next higher or 100) ---
                higher = next((v for v in SPEEDS if v > cruise), None)
                if higher is None:
                    print("SKIP speed_up %s (already max cruise)" % tag)
                    skipped += 1
                else:
                    name = "speed_up %s->%g" % (tag, higher)
                    res = run_scenario(
                        name, cruise, accel,
                        [(0.0, lambda s: s.move_to(100.0)),
                         (t_half, lambda s, h=higher: s.set_speed(h))])
                    try:
                        # Cap uses the higher cruise after the change.
                        _assert_common(self, res, higher, accel)
                        self.assertAlmostEqual(res.target_mm, 100.0, places=2)
                        print("PASS %s %s" % (name, res.detail))
                        passed += 1
                    except AssertionError as e:
                        print("FAIL %s %s — %s" % (name, res.detail, e))
                        fails.append(str(e))

                # --- Speed down (from higher start) ---
                if cruise <= SPEEDS[0]:
                    print("SKIP speed_down %s (lowest cruise)" % tag)
                    skipped += 1
                else:
                    # Start at this cruise, drop to a lower one early.
                    lower = SPEEDS[0] if cruise > SPEEDS[0] else cruise * 0.5
                    # Prefer previous matrix speed when available.
                    lowers = [v for v in SPEEDS if v < cruise]
                    lower = lowers[-1] if lowers else cruise * 0.5
                    t_early = min(1.0, max(0.25, 0.15 * expected_triangle_time(100.0, cruise, accel)))
                    name = "speed_down %s->%g" % (tag, lower)
                    res = run_scenario(
                        name, cruise, accel,
                        [(0.0, lambda s: s.move_to(100.0)),
                         (t_early, lambda s, lo=lower: s.set_speed(lo))])
                    try:
                        # After slowdown, peak should not greatly exceed the start cruise.
                        v_phys = vmax_for_distance(50.0, accel)
                        v_cap = max(cruise, v_phys) * 1.15 + 0.5
                        self.assertTrue(res.ok, "%s: %s" % (name, res.detail))
                        self.assertLess(res.pos_err, 0.05)
                        self.assertFalse(_has_overshoot(res))
                        self.assertLessEqual(res.peak_v, v_cap + 1e-3)
                        self.assertAlmostEqual(res.target_mm, 100.0, places=2)
                        print("PASS %s %s" % (name, res.detail))
                        passed += 1
                    except AssertionError as e:
                        print("FAIL %s %s — %s" % (name, res.detail, e))
                        fails.append(str(e))

        print("\n========== SUMMARY ==========")
        print("passed=%d skipped=%d failed=%d" % (passed, skipped, len(fails)))
        if fails:
            self.fail("%d scenario(s) failed:\n  %s" % (len(fails), "\n  ".join(fails[:20])))


if __name__ == "__main__":
    unittest.main(verbosity=2)
