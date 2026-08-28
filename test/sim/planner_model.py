"""Offline twin of planner_fill_fifo + PIO FIFO drain (host tests / CLI smoke)."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Sequence, Tuple

SPMM = 320.0
SYSCLK = 125_000_000
FIXED = 192
RAMP_START_HZ = 1000
STOP_APPROACH_HZ = 400
DIR_PAUSE_S = 0.1
PACK_MIN_HZ = 2500
PACK_MAX = 64
BUDGET_MS = 3.0
FIFO_MAX = 8
SOFT_MIN_MM = 0.0
SOFT_MAX_MM = 600.0
MAX_SPEED_MM_S = 100.0
BRAKE_SCURVE = True

Action = Tuple[float, Callable[["Sim"], None]]


def vmax_for_distance(d: float, a: float) -> float:
    if d <= 0.0:
        return 0.0
    a = max(a, 1e-3)
    return math.sqrt(4.0 * a * d / math.pi)


def expected_triangle_time(dist_mm: float, cruise: float, accel: float) -> float:
    """Rough duration for a sine accel+decel (+cruise) over dist_mm."""
    if dist_mm <= 0.0 or cruise < 1e-3 or accel < 1e-3:
        return 1.0
    v_peak = min(cruise, vmax_for_distance(dist_mm * 0.5, accel))
    if v_peak < 1e-3:
        return 1.0
    t_ramp = math.pi * v_peak / (2.0 * accel)  # one half-sine
    d_ramp = math.pi * v_peak * v_peak / (4.0 * accel)
    if 2.0 * d_ramp >= dist_mm:
        return 2.0 * t_ramp
    return 2.0 * t_ramp + (dist_mm - 2.0 * d_ramp) / v_peak


def halfway_time(cruise: float, accel: float, dist_mm: float = 100.0) -> float:
    """Retarget time that lands mid-move for a range of profiles."""
    t_exp = expected_triangle_time(dist_mm, cruise, accel)
    if cruise >= 20.0 and accel >= 10.0:
        return min(3.0, max(0.4, 0.4 * t_exp))
    return min(3.0, max(0.3, 0.4 * t_exp))


@dataclass
class ScenarioResult:
    name: str
    ok: bool
    t: float
    peak_v: float
    pos_mm: float
    target_mm: float
    pos_err: float
    events: List[Tuple[float, float, float, str]] = field(default_factory=list)
    detail: str = ""
    retarget_v: float = 0.0
    skipped: bool = False


class Sim:
    def __init__(self, cruise: float = 20.0, accel: float = 10.0):
        self.cruise = cruise
        self.accel = accel
        self.pos = 0
        self.target = 0
        self.vel = 0.0
        self.v0 = 0.0
        self.v1 = 0.0
        self.phi = 0.0
        self.ramp_active = False
        self.stopping = False
        self.braking = False
        # Committed brake arc: total steps, start position, entry speed.
        self.brake_d = 0
        self.brake_pos0 = 0
        self.brake_v0 = 0.0
        self.acc_meas = 0.0
        self.dir_pause = False
        self.dir_pause_s = 0.0
        self.last_sign = 0
        self.moving = False
        self.fifo: List[Tuple[int, int, int]] = []
        self.cur: Optional[List[float]] = None
        self.t = 0.0
        self.hw_pos = 0
        self.events: List[Tuple[float, float, float, str]] = []

    @staticmethod
    def stop_rem_steps(v, a, spmm):
        if abs(v) < 0.01:
            return 0
        a = max(a, 1e-3)
        d = math.pi * v * v / (4.0 * a)
        return max(1, math.ceil(abs(d) * spmm))

    @staticmethod
    def needs_reverse(last_sign, tgt_sign, v):
        return last_sign != 0 and tgt_sign != 0 and tgt_sign != last_sign and abs(v) > 0.05

    @staticmethod
    def sine_vel(v0, v1, phi):
        if phi <= 0.0:
            return v0
        if phi >= 1.0:
            return v1
        return v0 + (v1 - v0) * 0.5 * (1.0 - math.cos(math.pi * phi))

    @staticmethod
    def advance_phi(phi, v0, v1, a, dt):
        dv = abs(v1 - v0)
        if dv < 1e-6 or a < 1e-6 or dt <= 0.0:
            return 1.0
        T = math.pi * dv / (2.0 * a)
        if T < 1e-6:
            return 1.0
        return min(1.0, phi + dt / T)

    @staticmethod
    def hz_to_delay(hz):
        hz = max(hz, 1.0)
        d = SYSCLK / hz - FIXED
        return max(1, min(int(d), 0x03FFFFFF))

    @staticmethod
    def pack_n(step_hz, rem, pending):
        if rem <= 0:
            return 0
        if step_hz < PACK_MIN_HZ:
            return 1
        bs = max(1, int(step_hz * BUDGET_MS / 1000.0))
        room = bs - pending
        if room < 1:
            return 0
        n = min(room, PACK_MAX, rem)
        return min(n, max(1, rem // 5))

    def reset_ramp(self):
        self.ramp_active = False
        self.phi = 0.0
        self.v0 = self.v1 = self.vel

    def begin_ramp(self, v_cmd):
        if not self.ramp_active or abs(v_cmd - self.v1) > 1e-4:
            self.v0 = self.vel
            self.v1 = v_cmd
            self.phi = 0.0
            self.ramp_active = True

    def remaining_steps_for_sign(self, sign):
        rem_t = self.target - self.pos
        if sign > 0 and rem_t < 0:
            rem_t = 0
        if sign < 0 and rem_t > 0:
            rem_t = 0
        rem = abs(rem_t)
        if sign > 0:
            rem = min(rem, max(0, int(SOFT_MAX_MM * SPMM) - self.pos))
        elif sign < 0:
            rem = min(rem, max(0, self.pos - int(SOFT_MIN_MM * SPMM)))
        return rem

    def pending_steps(self):
        p = sum(w[1] for w in self.fifo)
        if self.cur:
            p += int(self.cur[0])
        return p

    def tx_room(self):
        return FIFO_MAX - len(self.fifo)

    def put_word(self, delay, n, sign):
        if len(self.fifo) >= FIFO_MAX:
            return False
        self.fifo.append((delay, n, sign))
        return True

    def log(self, msg):
        self.events.append((round(self.t, 4), self.pos / SPMM, self.vel, msg))

    def fill(self):
        if not self.moving or self.dir_pause:
            return
        budget = 4
        while budget > 0 and self.tx_room() > 0:
            budget -= 1
            err = self.target - self.pos
            if self.stopping:
                if abs(self.vel) < 0.01:
                    break
                sign = 1 if self.vel >= 0 else -1
            elif err > 0:
                sign = 1
            elif err < 0:
                sign = -1
            else:
                v_stop = STOP_APPROACH_HZ / SPMM if STOP_APPROACH_HZ > 0 else 0.0
                if (abs(self.vel) <= v_stop + 1e-3
                        or self.stop_rem_steps(self.vel, self.accel, SPMM) <= 4):
                    self.vel = 0.0
                    break
                sign = 1 if self.vel >= 0 else -1

            reverse_decel = False
            if not self.stopping and self.needs_reverse(self.last_sign, sign, self.vel):
                self.begin_ramp(0.0)
                sign = 1 if self.vel >= 0 else -1
                reverse_decel = True

            if self.stopping or err == 0 or reverse_decel:
                rem = self.stop_rem_steps(self.vel, self.accel, SPMM)
            else:
                rem = self.remaining_steps_for_sign(sign)

            if rem <= 0:
                if err != 0 and not self.stopping and not reverse_decel:
                    self.log("OVERSHOOT rem<=0 (v forced 0)")
                self.vel = 0.0
                self.reset_ramp()
                self.brake_d = 0
                break

            rem_mm = rem / SPMM
            vmax = vmax_for_distance(rem_mm, self.accel)
            cruise_cap = min(self.cruise, MAX_SPEED_MM_S)
            need_brake = (not self.stopping and not reverse_decel and
                          rem <= self.stop_rem_steps(abs(self.vel), self.accel, SPMM) + 4)
            if (self.stopping or reverse_decel):
                v_cmd = 0.0
            elif BRAKE_SCURVE and (self.braking or need_brake):
                self.braking = True
                v_cmd = 0.0
            elif not BRAKE_SCURVE and abs(self.vel) > vmax + 0.05:
                v_cmd = 0.0
            else:
                v_cmd = sign * cruise_cap

            decel = self.stopping or reverse_decel or self.braking
            if decel:
                # Commit the arc once, then measure progress along it. rem for a
                # soft stop is re-derived from v each word and would drift.
                if self.brake_d <= 0:
                    self.brake_d = rem
                    self.brake_pos0 = self.pos
                    self.brake_v0 = abs(self.vel)
                rem = self.brake_d - abs(self.pos - self.brake_pos0)
                if rem <= 0 or self.brake_v0 <= 0.0:
                    self.vel = 0.0
                    self.reset_ramp()
                    self.brake_d = 0
                    break
            else:
                self.brake_d = 0

            if (not self.stopping and self.last_sign != 0 and sign != self.last_sign
                    and abs(self.vel) < 0.05):
                if DIR_PAUSE_S > 0.0:
                    self.dir_pause = True
                    self.dir_pause_s = DIR_PAUSE_S
                    self.vel = 0.0
                    self.reset_ramp()
                    self.brake_d = 0
                    self.log("dir_pause")
                    break

            # Seed ramp_start when leaving standstill (matches firmware).
            if (RAMP_START_HZ > 0 and not self.stopping and not reverse_decel
                    and not self.braking):
                vmin = RAMP_START_HZ / SPMM
                if vmin > 0.0 and abs(self.vel) < vmin and abs(v_cmd) > vmin:
                    self.vel = sign * vmin
                    self.reset_ramp()

            self.begin_ramp(v_cmd)

            # ramp_start is the launch floor only; braking is bounded by
            # stop_approach so the tail tapers instead of halting from ramp_start.
            min_hz = float(STOP_APPROACH_HZ) if (decel and STOP_APPROACH_HZ > 0) \
                else float(RAMP_START_HZ)

            hz_est = abs(self.vel) * SPMM
            if hz_est < 1.0:
                hz_est = min_hz
            if not decel and rem > 1 and hz_est < RAMP_START_HZ:
                hz_est = float(RAMP_START_HZ)
            if (decel or rem <= 4) and 0 < hz_est < STOP_APPROACH_HZ:
                hz_est = float(STOP_APPROACH_HZ)

            n = self.pack_n(hz_est, rem, self.pending_steps())
            if n <= 0:
                break
            n = min(n, rem)
            # Curvature cap while braking: bound word size against the peak
            # |dv/dR| of the sine brake arc (at R = D/2) so a mid-arc word
            # cannot hold enough pulses to make a large, audible speed step.
            if decel and self.brake_d > 0:
                brake_step_frac = 0.06
                n_curv = max(1, int(brake_step_frac * (2.0 / math.pi) * self.brake_d))
                n = min(n, n_curv)

            dt = n / hz_est
            v_prev = self.vel
            if decel:
                # Brake speed is a closed form of the distance still to run:
                #   v(R) = v_app + (v0 - v_app) * sin(pi*R / (2*D))
                # Integrating the ramp in time instead let it lag (dphi/ds =
                # 1/(v*T) is stiff as v -> 0); the v_cap clamp then took over and
                # stopped the axis at full accel. Keyed on distance it cannot
                # drift. Landing on v_app rather than 0 keeps the arc finite --
                # a tail that decays to zero takes exponentially long and ends
                # up crawling on the floor anyway.
                v_app = STOP_APPROACH_HZ / SPMM if STOP_APPROACH_HZ > 0 else 0.0
                dv = self.brake_v0 - v_app
                if dv < 0.0:
                    dv = 0.0
                r_after = rem - n
                if r_after <= 0:
                    self.vel = sign * v_app
                else:
                    self.vel = sign * (v_app + dv * math.sin(
                        math.pi * r_after / (2.0 * self.brake_d)))
            else:
                self.phi = self.advance_phi(self.phi, self.v0, self.v1, self.accel, dt)
                self.vel = self.sine_vel(self.v0, self.v1, self.phi)
                if self.phi >= 1.0:
                    self.vel = self.v1
                    self.ramp_active = False

            cap = vmax * 1.25 if self.braking else vmax
            if abs(self.vel) > cap:
                self.vel = cap if self.vel >= 0 else -cap
                if not self.braking:
                    self.reset_ramp()
            if dt > 1e-6:
                self.acc_meas += 0.25 * ((self.vel - v_prev) / dt - self.acc_meas)

            step_hz = abs(self.vel) * SPMM
            if not decel and rem > 1 and step_hz < RAMP_START_HZ:
                step_hz = float(RAMP_START_HZ)
            if (decel or rem <= 4) and 0 < step_hz < STOP_APPROACH_HZ:
                step_hz = float(STOP_APPROACH_HZ)
            if step_hz < 1.0:
                if self.stopping or reverse_decel or err == 0:
                    self.vel = 0.0
                    break
                step_hz = 1.0

            if not self.put_word(self.hz_to_delay(step_hz), n, sign):
                break
            pos_before = self.pos
            self.pos += n * sign
            self.last_sign = sign

            if not self.stopping and not reverse_decel and err != 0:
                crossed = ((sign > 0 and pos_before <= self.target < self.pos) or
                           (sign < 0 and pos_before >= self.target > self.pos))
                if crossed:
                    self.log("OVERSHOOT snap %d steps -> pos=target" % abs(self.pos - self.target))
                    self.pos = self.target
                    self.vel = 0.0
                    self.reset_ramp()
                    break

    def tick(self, dt):
        if self.dir_pause:
            self.dir_pause_s -= dt
            if self.dir_pause_s <= 0.0:
                self.dir_pause = False
                self.last_sign = 0
                self.log("dir_pause end")
        err = self.target - self.pos
        if not self.stopping and err == 0 and abs(self.vel) < 0.01:
            if self.moving:
                self.moving = False
                self.vel = 0.0
                self.reset_ramp()
                self.log("SETTLE -> idle")

    def move_to(self, mm):
        self.target = int(round(mm * SPMM))
        self.stopping = False
        self.braking = False
        self.brake_d = 0
        self.moving = True
        self.begin_ramp(self.vel)
        self.log("cmd mt%g" % mm)

    def set_speed(self, mm_s):
        self.cruise = mm_s
        self.braking = False
        self.brake_d = 0
        self.log("cmd ss%g" % mm_s)

    def drain(self, t_end):
        while True:
            if self.cur is None:
                if not self.fifo:
                    return
                delay, n, sign = self.fifo.pop(0)
                period = (FIXED + delay) / SYSCLK
                self.cur = [n, period, self.t + period, sign]
            if self.cur[2] > t_end:
                return
            self.t = self.cur[2]
            self.hw_pos += int(self.cur[3])
            self.cur[0] -= 1
            if self.cur[0] <= 0:
                self.cur = None
            else:
                self.cur[2] = self.t + self.cur[1]


def run_scenario(
    name: str,
    cruise: float,
    accel: float,
    actions: Sequence[Action],
    verbose: bool = False,
    t_end: float = 120.0,
) -> ScenarioResult:
    """Run scheduled actions. Returns ScenarioResult (ok flag for asserts)."""
    s = Sim(cruise=cruise, accel=accel)
    next_tick = 0.0
    next_verbose = 0.0
    peak_v = 0.0
    pending = list(actions)
    step = 0.0001
    retarget_v = 0.0
    first_action_done = False

    if verbose:
        print("\n=== %s ===" % name)
        print("  start: ss%g sa%g" % (cruise, accel))

    while s.t < t_end:
        while pending and s.t + 1e-9 >= pending[0][0]:
            _t_act, fn = pending.pop(0)
            if first_action_done:
                retarget_v = abs(s.vel)
            if verbose:
                print("  --- t=%.2f pos=%.2f v=%.2f : action ---" %
                      (s.t, s.pos / SPMM, s.vel))
            fn(s)
            first_action_done = True

        t_next = s.t + step
        s.drain(t_next)
        s.t = t_next
        s.fill()
        if s.t >= next_tick:
            s.tick(0.005)
            next_tick += 0.005
        peak_v = max(peak_v, abs(s.vel))
        if s.t >= next_verbose:
            if verbose:
                state = "M" if s.moving else "I"
                print("  %6.2f #%s %8.2f %6.2f %6.2f  tgt=%.1f" %
                      (s.t, state, s.pos / SPMM, abs(s.vel), abs(s.acc_meas),
                       s.target / SPMM))
            next_verbose += 0.333

        if not pending and not s.moving and s.t > 0.05:
            break

    final_tgt = s.target / SPMM
    pos_err = abs(s.pos / SPMM - final_tgt)
    ok = (not s.moving) and pos_err < 0.05 and peak_v > 0.1
    detail = "t=%.2fs peak_v=%.2f pos=%.3f tgt=%.1f err=%.3f" % (
        s.t, peak_v, s.pos / SPMM, final_tgt, pos_err)
    if verbose:
        print("  -> %s %s" % ("PASS" if ok else "FAIL", detail))
        for t, p, v, m in s.events:
            print("      t=%7.4f pos=%8.3f v=%6.2f  %s" % (t, p, v, m))

    return ScenarioResult(
        name=name,
        ok=ok,
        t=s.t,
        peak_v=peak_v,
        pos_mm=s.pos / SPMM,
        target_mm=final_tgt,
        pos_err=pos_err,
        events=list(s.events),
        detail=detail,
        retarget_v=retarget_v,
    )
