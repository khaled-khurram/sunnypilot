"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import math

from cereal import custom
from openpilot.common.realtime import DT_MDL
from openpilot.common.constants import CV
from openpilot.sunnypilot.selfdrive.controls.lib.phase3_shared import (
  STEP_MPH, ABSOLUTE_FLOOR_MPH, MIN_COMMAND_INTERVAL_S, GRACE_PERIOD_S, SETTLE_TIME_S, CURVE_ARM_FILE,
  BUTTON_SET_SHALLOW, BUTTON_RESUME_SHALLOW, Phase3OverrideLatch, Phase3CommandArbiter,
  is_armed, log_shadow_decision,
)

MapState = custom.LongitudinalPlanSP.SmartCruiseControl.MapState

# --- locked settings, research/phase3_controller_design.md §7 ---
DEADBAND_MPH = 1.5
DISTANCE_MARGIN = 1.25
DECEL_RATE_SEED_MPH_S = 1.94          # user's own test: 60->25mph in 18s, flat road
FT_PER_MPH_SEC = 1.4667               # mph * (5280/3600) = ft/s per mph
CURVE_EVENT_BUDGET = 60               # placeholder judgment call, revised 2026-07-24: the
                                       # original value of 10 was carried over from an
                                       # earlier *whole-session* cap discussion without
                                       # being reconsidered for what a single curve's own
                                       # delta needs - verification testing found it
                                       # couldn't even cover a 60->40mph curve target
                                       # (needs 20 steps), let alone 80->50mph (30 steps).
                                       # 60 comfortably covers the worst realistic single
                                       # curve delta (80mph down to the 25mph floor = 55mph)
                                       # while still being a finite defensive backstop, not
                                       # an open-ended one. In practice MIN_COMMAND_INTERVAL_S
                                       # below means this is rarely the binding constraint -
                                       # a curve would need to stay "turning" for a full
                                       # minute to ever exhaust it.
# STEP_MPH, ABSOLUTE_FLOOR_MPH: shared across every Phase 3 feature, see phase3_shared.py

# Shadow-mode-only: never read by carcontroller.py or any car-interface code. This run
# cannot affect real CAN output even in principle. (SHADOW_LOG_FILE lives in phase3_shared.py)
DECEL_STATE_FILE = "/data/phase3_decel_rate_mph.txt"

MS_TO_MPH = CV.MS_TO_MPH


def distance_needed_ft(v_current_mph: float, v_target_mph: float, decel_rate_mph_s: float) -> float:
  """Feet needed to decelerate from v_current to v_target at a constant rate.
  Reverse-derived and checked against the user's own 60->25mph/18s/1120ft test
  and its 80mph projections - see research/phase3_controller_design.md §7."""
  if v_current_mph <= v_target_mph or decel_rate_mph_s <= 0:
    return 0.0
  avg_speed_mph = (v_current_mph + v_target_mph) / 2.0
  t_s = (v_current_mph - v_target_mph) / decel_rate_mph_s
  return avg_speed_mph * FT_PER_MPH_SEC * t_s


def largest_reachable_target_mph(v_current_mph: float, decel_rate_mph_s: float,
                                  dist_available_ft: float, margin: float) -> float:
  """Inverse of distance_needed_ft: given the distance actually available, find the
  largest v_target (smallest drop) whose distance_needed_ft(...) * margin fits within
  it. Closed-form solve of distance_needed_ft as a quadratic in delta = v_current - v_target:

    dist_available/margin = (v_current - delta/2) * FT_PER_MPH_SEC * (delta / rate)
    => delta^2 - 2*v_current*delta + 2*k*(dist_available/margin) = 0, k = rate/FT_PER_MPH_SEC
    => delta = v_current - sqrt(v_current^2 - 2*k*(dist_available/margin))   [smaller root]

  This is genuinely fiddly closed-form algebra - worth a dedicated review/test pass
  before this is ever exercised against a live decision, not just trusted because it
  matches the docstring's derivation."""
  if decel_rate_mph_s <= 0 or dist_available_ft <= 0:
    return v_current_mph
  k = decel_rate_mph_s / FT_PER_MPH_SEC
  d = dist_available_ft / margin
  discriminant = v_current_mph ** 2 - 2 * k * d
  if discriminant <= 0:
    # Available distance covers even a full stop from v_current - no downgrade needed,
    # any requested target is reachable.
    return 0.0
  delta = v_current_mph - math.sqrt(discriminant)
  return max(0.0, min(v_current_mph, v_current_mph - delta))


def _load_decel_rate() -> float:
  try:
    with open(DECEL_STATE_FILE) as f:
      return float(f.read().strip())
  except (FileNotFoundError, ValueError, OSError):
    return DECEL_RATE_SEED_MPH_S


def _save_decel_rate(rate: float) -> None:
  try:
    with open(DECEL_STATE_FILE, "w") as f:
      f.write(f"{rate:.4f}")
  except OSError:
    pass  # persistence is best-effort; must never affect the decision loop


class Phase3CurveController:
  """
  Shadow-mode-only Phase 3 curve actuation (research/phase3_controller_design.md).
  Decides what SET/RESUME button presses *would* be sent to walk EyeSight's own
  cruise target down for an upcoming curve and back up afterward. Writes decisions
  to SHADOW_LOG_FILE only - never to any path carcontroller.py or any car-interface
  code reads. This run cannot affect real CAN output even in principle.
  """

  def __init__(self, override_latch: Phase3OverrideLatch, command_arbiter: Phase3CommandArbiter):
    self._override_latch = override_latch  # SHARED across every Phase 3 feature - see
                                              # phase3_shared.py's Phase3OverrideLatch
                                              # docstring for why this must not be private
    self._arbiter = command_arbiter  # SHARED with the lead controller - only one real
                                       # command can be written per planner cycle
    self.frame = -1
    self.armed = False
    self._read_arm_state()

    self.was_active = False    # curve rising-edge tracking, mirrors CurveAdvisoryHelper
    self.was_gated_on = False  # arm+engaged rising-edge, for baseline snapshot
    self.gated_on_since: float | None = None  # for GRACE_PERIOD_S - see phase3_shared.py

    self.baseline_v_cruise_mph: float | None = None  # driver's own pre-Phase-3 set
                                                       # speed, snapshotted once per
                                                       # arm-cycle, not per-curve
    self.sim_target_mph: float | None = None          # shadow-simulated commanded
                                                       # target - never actually sent
    self.budget_remaining = 0
    self.t = 0.0
    self.last_command_t = -1e9  # MIN_COMMAND_INTERVAL_S gate
    self.decel_rate_mph_s = _load_decel_rate()

    self.decision = "inert-not-armed"
    self._last_logged_decision = None

  def _read_arm_state(self) -> None:
    # Flag-file, not Params() - see phase3_shared.py's CURVE_ARM_FILE comment for why.
    # Checked every ~1s (PARAMS_UPDATE_PERIOD-equivalent cadence), not every frame - a
    # stat() call every 20ms is unnecessary overhead for a value that only changes when
    # a human explicitly touches/removes the file.
    if self.frame == -1 or self.frame % int(1.0 / DT_MDL) == 0:
      self.armed = is_armed(CURVE_ARM_FILE)

  def _log(self, v_current_mph: float, v_target_mph: float, dist_needed_ft: float,
            dist_available_ft: float) -> None:
    if self.decision == self._last_logged_decision:
      return
    self._last_logged_decision = self.decision
    log_shadow_decision(
      "curve",
      v_current_mph=round(v_current_mph, 2),
      v_target_mph=round(v_target_mph, 2),
      dist_needed_ft=round(dist_needed_ft, 1),
      dist_available_ft=round(dist_available_ft, 1),
      decision=self.decision,
      budget_remaining=self.budget_remaining,
      sim_target_mph=round(self.sim_target_mph, 2) if self.sim_target_mph is not None else None,
      override_reason=self._override_latch.trip_reason,  # added 2026-07-24, was undiagnosable from the first drive's log
    )

  def _step_toward(self, goal_mph: float, clamp_low: float | None, clamp_high: float | None) -> str:
    """Nudge sim_target_mph one shallow step toward goal_mph, respecting budget and
    deadband. Returns the decision string. Shared by both the curve fire-down path
    and the post-curve restore-up path - same primitive, different goal/direction."""
    if abs(goal_mph - self.sim_target_mph) < DEADBAND_MPH:
      return "hold"
    if self.budget_remaining <= 0:
      return "hold"
    if (self.t - self.last_command_t) < MIN_COMMAND_INTERVAL_S:
      return "hold-rate-limited"
    step = STEP_MPH if goal_mph > self.sim_target_mph else -STEP_MPH
    button_value = BUTTON_RESUME_SHALLOW if step > 0 else BUTTON_SET_SHALLOW
    if not self._arbiter.try_write(button_value):
      # Lead controller already wrote this cycle, or the whole-drive session cap is
      # hit - hold this step for one more ~50ms cycle rather than silently drop it;
      # the same decision will be re-evaluated and re-attempted next frame.
      return "hold-arbiter"
    new_target = self.sim_target_mph + step
    if clamp_low is not None:
      new_target = max(new_target, clamp_low)
    if clamp_high is not None:
      new_target = min(new_target, clamp_high)
    self.sim_target_mph = new_target
    self.budget_remaining -= 1
    self.last_command_t = self.t
    return "fire" if step < 0 else "restore"

  def update(self, map_state: int, distance_m: float, map_v_target: float,
             long_enabled: bool, v_ego: float, v_cruise: float,
             gas_pressed: bool, brake_pressed: bool, steering_pressed: bool) -> None:
    self._read_arm_state()
    self.t += DT_MDL

    v_current_mph = v_ego * MS_TO_MPH
    v_cruise_mph = v_cruise * MS_TO_MPH
    v_target_mph = map_v_target * MS_TO_MPH
    dist_available_ft = distance_m * 3.28084

    is_active = map_state == MapState.turning
    gated_on = self.armed and long_enabled

    if gated_on and not self.was_gated_on:
      self.gated_on_since = self.t  # fresh grace-period start on every new gated-on edge

    # Override latch - checked before any policy decision, session-long once tripped.
    # Shared instance: this also latches off any other Phase 3 feature using the same
    # Phase3OverrideLatch, and vice versa. GRACE_PERIOD_S (added 2026-07-24, after the
    # first live drive tripped this in the very first gated-on frame, most likely
    # residual gasPressed right as cruise engaged) suppresses the CHECK - not arming or
    # gating - for this long after first becoming gated-on, so an engagement-timing
    # artifact can't permanently kill the session before it ever ran once. Any override
    # after the grace window still latches off instantly and permanently, unchanged.
    if gated_on and self.gated_on_since is not None and (self.t - self.gated_on_since) >= GRACE_PERIOD_S:
      self._override_latch.check(gas_pressed, brake_pressed, steering_pressed)

    if not gated_on:
      self.decision = "inert-not-armed"
      self.was_active = is_active
      self.was_gated_on = False
      self.frame += 1
      return

    if self._override_latch.overridden:
      self.decision = "latched-off"
      self._log(v_current_mph, v_target_mph, 0.0, dist_available_ft)
      self.was_active = is_active
      self.was_gated_on = gated_on
      self.frame += 1
      return

    # Snapshot the driver's own pre-Phase-3 set speed once per arm-cycle, not per-curve -
    # a chained sequence of curves before a full restore completes still all target this
    # same one baseline (research/phase3_controller_design.md §7, "chained curves" row).
    if gated_on and not self.was_gated_on:
      self.baseline_v_cruise_mph = v_cruise_mph
      self.sim_target_mph = v_cruise_mph

    if self.sim_target_mph is None:
      self.sim_target_mph = v_cruise_mph

    # Resync both baseline and sim_target to the real, current set speed after a genuine
    # idle period (no active curve, nothing recently sent) - see phase3_shared.py's
    # SETTLE_TIME_S comment for the full story. Gating on time-since-last-command rather
    # than "sim_target == v_cruise" specifically so this self-heals even when BOTH were
    # wrong together from the start (the actual first-drive failure mode - comparing them
    # to each other never catches that), without corrupting an in-progress restore.
    if not is_active and (self.t - self.last_command_t) > SETTLE_TIME_S:
      self.baseline_v_cruise_mph = v_cruise_mph
      self.sim_target_mph = v_cruise_mph

    # Rising edge of a new curve resets the fire-down budget; falling edge (curve just
    # cleared) resets a fresh budget for the restore-up phase. These must NOT share one
    # pool - a big curve that spends most of its budget walking the target down would
    # otherwise leave too little to walk back up, stranding the car below the driver's
    # own set speed until the next curve happens to reset the counter. Restoring the
    # driver's own previously-set speed is a materially safer direction to under-bound
    # than leaving them stuck slower with no path back.
    if is_active and not self.was_active:
      self.budget_remaining = CURVE_EVENT_BUDGET
    elif not is_active and self.was_active:
      self.budget_remaining = CURVE_EVENT_BUDGET

    if is_active:
      dist_needed_ft = distance_needed_ft(v_current_mph, v_target_mph, self.decel_rate_mph_s)
      trigger_dist_ft = dist_needed_ft * DISTANCE_MARGIN

      effective_target_mph = max(v_target_mph, ABSOLUTE_FLOOR_MPH)
      downgraded = False
      if dist_needed_ft > 0 and dist_available_ft < trigger_dist_ft:
        effective_target_mph = max(
          largest_reachable_target_mph(v_current_mph, self.decel_rate_mph_s, dist_available_ft, DISTANCE_MARGIN),
          ABSOLUTE_FLOOR_MPH)
        downgraded = True

      self.decision = self._step_toward(effective_target_mph, clamp_low=ABSOLUTE_FLOOR_MPH, clamp_high=None)
      if downgraded and self.decision == "fire":
        self.decision = "downgrade"
      self._log(v_current_mph, effective_target_mph, dist_needed_ft, dist_available_ft)

    else:
      # Curve cleared - restore toward the baseline, never above it.
      goal = self.baseline_v_cruise_mph if self.baseline_v_cruise_mph is not None else v_cruise_mph
      self.decision = self._step_toward(goal, clamp_low=None, clamp_high=self.baseline_v_cruise_mph)
      self._log(v_current_mph, goal, 0.0, dist_available_ft)

    self.was_active = is_active
    self.was_gated_on = gated_on
    self.frame += 1

  def observe_decel_sample(self, isolated: bool, observed_rate_mph_s: float) -> None:
    """Update the decel-rate EMA from a real observed, isolated decel event (no lead
    tracked, no brake pressed, for the whole observation window following a commanded
    drop - research/phase3_controller_design.md §7). Not exercised this session: shadow
    mode never sends a real command, so there is nothing real to observe yet. The
    archive-mining pass for this was explicitly deferred (§7), so this has no backtest
    tonight either - implemented and gated correctly for when live commands eventually
    exist."""
    if not isolated:
      return
    alpha = 0.2
    self.decel_rate_mph_s = (1 - alpha) * self.decel_rate_mph_s + alpha * observed_rate_mph_s
    _save_decel_rate(self.decel_rate_mph_s)
