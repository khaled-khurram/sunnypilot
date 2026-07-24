"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.common.realtime import DT_MDL
from openpilot.common.constants import CV
from openpilot.sunnypilot.selfdrive.controls.lib.phase3_shared import (
  STEP_MPH, ABSOLUTE_FLOOR_MPH, MIN_COMMAND_INTERVAL_S, LEAD_ARM_FILE,
  BUTTON_SET_SHALLOW, BUTTON_RESUME_SHALLOW, Phase3OverrideLatch, Phase3CommandArbiter,
  is_armed, log_shadow_decision,
)

# --- reused as-is from the already-shipped/backtested advisory + guidance helpers,
# NOT reinvented - research/lead_closing_trigger_backtest.md found no single-feature
# threshold adjustment would improve precision, so these stay exactly what they were
# tuned to already. ---
CLOSING_VREL_THRESHOLD = -3.0     # m/s, lead_closing_advisory_helper.py
SUSTAIN_TIME = 0.5                # seconds, same file
NO_RECENT_PEDAL_TIME = 3.0        # seconds, same file - kept even though the shared
                                   # override latch below now also catches a pedal press
                                   # immediately and more strictly; this is the exact
                                   # combination the backtest validated, not re-derived
MIN_TRIGGER_SPEED_MPH = 50.0      # mph, same file's MIN_ADVISORY_SPEED
EPISODE_DEBOUNCE_TIME = 20.0      # seconds, same file's DEBOUNCE_TIME - min gap between
                                   # fresh episode starts
TARGET_MARGIN_MPH = 4.0           # lead_closing_test_guidance_helper.py
CONVERGED_TOLERANCE_MPH = 2.0     # same file

# New for actuation specifically - the advisory/guidance tools only ever alerted, they
# never had to decide when it's safe to walk speed back UP. That's a new question.
CLEAR_HYSTERESIS_TIME = 2.0        # seconds - deliberately longer than SUSTAIN_TIME's
                                    # 0.5s: trigger fast when a closing lead appears, but
                                    # require more sustained evidence the gap has really
                                    # reopened before restoring speed, so it doesn't
                                    # oscillate right at the threshold. A judgment call,
                                    # not independently backtested - flagged the same way
                                    # as the curve controller's own budget placeholder.
LEAD_EPISODE_BUDGET = 60           # mirrors CURVE_EVENT_BUDGET - revised 2026-07-24 for
                                    # the same reason (see phase3_curve_controller.py):
                                    # a value of 10 could only ever cover a 10mph total
                                    # adjustment, but a closing lead could be going 25mph+
                                    # slower than a highway ego speed. 60 comfortably
                                    # covers worst-case single-episode deltas while still
                                    # being a finite defensive backstop
ROLLING_WINDOW_S = 300.0           # 5 minutes
ROLLING_WINDOW_CAP = 20            # placeholder judgment call - backstop independent of
                                    # the per-episode budget, since lead-following
                                    # episodes can recur far more often per drive than
                                    # curves do (research/phase3_controller_design.md §9)

MS_TO_MPH = CV.MS_TO_MPH
MPH_TO_MS = CV.MPH_TO_MS


class Phase3LeadController:
  """
  Shadow-mode-only Phase 3 lead-vehicle actuation (research/phase3_controller_design.md
  §9). Decides what SET/RESUME button presses *would* be sent to walk EyeSight's own
  cruise target down when a vision-tracked lead is closing and EyeSight hasn't reacted
  yet, and back up once the gap reopens. Writes decisions to the same shadow log as
  curve actuation - never to any path carcontroller.py or any car-interface code reads.
  This run cannot affect real CAN output even in principle.

  Shares one Phase3OverrideLatch with Phase3CurveController - a single override event
  (brake/steering/gas/real-button-press) latches BOTH features off together, matching
  the user's own framing ("tap brakes once, everything goes dark"), not just whichever
  feature happened to be acting at that moment. The `lead_closing_advisory_helper.py`
  and `lead_closing_test_guidance_helper.py` advisories keep running independently of
  this - this class only adds actuation on top, gated separately.
  """

  def __init__(self, override_latch: Phase3OverrideLatch, command_arbiter: Phase3CommandArbiter):
    self._override_latch = override_latch
    self._arbiter = command_arbiter  # SHARED with the curve controller - only one real
                                       # command can be written per planner cycle
    self.frame = -1
    self.armed = False
    self._read_arm_state()

    self.closing_since: float | None = None
    self.clear_since: float | None = None
    self.last_pedal_t = -1e9
    self.last_episode_start_t = -1e9
    self.t = 0.0

    self.in_episode = False
    self.was_gated_on = False
    self.baseline_v_cruise_mph: float | None = None
    self.sim_target_mph: float | None = None
    self.budget_remaining = 0
    self.last_command_t = -1e9  # MIN_COMMAND_INTERVAL_S gate
    self.command_times: list[float] = []   # rolling-window backstop timestamps

    self.decision = "inert-not-armed"
    self._last_logged_decision = None

  def _read_arm_state(self) -> None:
    # Flag-file, not Params() - see phase3_shared.py's LEAD_ARM_FILE comment for why.
    # Independent of CURVE_ARM_FILE on purpose (§9: separate arm switch, so curve
    # actuation can be trusted/used before lead actuation is).
    if self.frame == -1 or self.frame % int(1.0 / DT_MDL) == 0:
      self.armed = is_armed(LEAD_ARM_FILE)

  def _rolling_window_ok(self) -> bool:
    cutoff = self.t - ROLLING_WINDOW_S
    self.command_times = [ct for ct in self.command_times if ct >= cutoff]
    return len(self.command_times) < ROLLING_WINDOW_CAP

  def _log(self, v_current_mph: float, v_target_mph: float) -> None:
    if self.decision == self._last_logged_decision:
      return
    self._last_logged_decision = self.decision
    log_shadow_decision(
      "lead",
      v_current_mph=round(v_current_mph, 2),
      v_target_mph=round(v_target_mph, 2),
      decision=self.decision,
      budget_remaining=self.budget_remaining,
      rolling_window_count=len(self.command_times),
      sim_target_mph=round(self.sim_target_mph, 2) if self.sim_target_mph is not None else None,
    )

  def _step_toward(self, goal_mph: float, clamp_low: float | None, clamp_high: float | None) -> str:
    """Same shallow-step primitive as Phase3CurveController, plus a rolling-window
    backstop check curves don't need (lead episodes can recur far more often per
    drive)."""
    if abs(goal_mph - self.sim_target_mph) < CONVERGED_TOLERANCE_MPH:
      return "hold"
    if self.budget_remaining <= 0:
      return "hold"
    if (self.t - self.last_command_t) < MIN_COMMAND_INTERVAL_S:
      return "hold-rate-limited"
    if not self._rolling_window_ok():
      return "hold-rolling-cap"
    step = STEP_MPH if goal_mph > self.sim_target_mph else -STEP_MPH
    button_value = BUTTON_RESUME_SHALLOW if step > 0 else BUTTON_SET_SHALLOW
    if not self._arbiter.try_write(button_value):
      # Curve controller already wrote this cycle (it goes first, wins ties on
      # purpose - see Phase3CommandArbiter's docstring), or the whole-drive session cap
      # is hit - hold for one more cycle rather than silently drop.
      return "hold-arbiter"
    new_target = self.sim_target_mph + step
    if clamp_low is not None:
      new_target = max(new_target, clamp_low)
    if clamp_high is not None:
      new_target = min(new_target, clamp_high)
    self.sim_target_mph = new_target
    self.budget_remaining -= 1
    self.last_command_t = self.t
    self.command_times.append(self.t)
    return "fire" if step < 0 else "restore"

  def update(self, lead, long_enabled: bool, v_ego: float, v_cruise: float,
             gas_pressed: bool, brake_pressed: bool, steering_pressed: bool,
             cruise_button: int) -> None:
    # `lead` is a radarState.leadOne capnp reader (cereal/log.capnp LeadData), same as
    # lead_closing_advisory_helper.py - not opendbc's CarState/CarParams structs.
    self._read_arm_state()
    self.t += DT_MDL

    v_current_mph = v_ego * MS_TO_MPH
    v_cruise_mph = v_cruise * MS_TO_MPH

    if gas_pressed or brake_pressed:
      self.last_pedal_t = self.t

    gated_on = self.armed and long_enabled

    # Override latch - shared instance: this also latches off curve actuation, and vice
    # versa, matching "tap brakes once, everything goes dark."
    if gated_on:
      self._override_latch.check(gas_pressed, brake_pressed, steering_pressed, cruise_button)

    if not gated_on:
      self.decision = "inert-not-armed"
      self.was_gated_on = False
      self.frame += 1
      return

    if self._override_latch.overridden:
      self.decision = "latched-off"
      self._log(v_current_mph, 0.0)
      self.was_gated_on = gated_on
      self.frame += 1
      return

    # Snapshot the driver's own pre-Phase-3 set speed once per arm-cycle, same semantics
    # as the curve controller's baseline (research/phase3_controller_design.md §7).
    if gated_on and not self.was_gated_on:
      self.baseline_v_cruise_mph = v_cruise_mph
      self.sim_target_mph = v_cruise_mph

    if self.sim_target_mph is None:
      self.sim_target_mph = v_cruise_mph

    closing = (lead.status and lead.vRel < CLOSING_VREL_THRESHOLD
               and v_ego >= MIN_TRIGGER_SPEED_MPH * MPH_TO_MS)
    if closing:
      if self.closing_since is None:
        self.closing_since = self.t
      self.clear_since = None
    else:
      self.closing_since = None
      if self.clear_since is None:
        self.clear_since = self.t

    sustained = self.closing_since is not None and (self.t - self.closing_since) >= SUSTAIN_TIME
    no_recent_pedal = (self.t - self.last_pedal_t) >= NO_RECENT_PEDAL_TIME
    cleared = self.clear_since is not None and (self.t - self.clear_since) >= CLEAR_HYSTERESIS_TIME

    # Rising edge of a new episode: fresh fire-down budget, gated by the 20s debounce.
    if sustained and no_recent_pedal and not self.in_episode:
      if (self.t - self.last_episode_start_t) >= EPISODE_DEBOUNCE_TIME:
        self.in_episode = True
        self.last_episode_start_t = self.t
        self.budget_remaining = LEAD_EPISODE_BUDGET

    # Falling edge (gap reopened): fresh restore-up budget - must NOT share the
    # fire-down pool, same reasoning as the curve controller's own fix (a long episode
    # could otherwise strand the car below the driver's own set speed with no path back
    # until the next episode happens to reset the counter).
    if self.in_episode and cleared:
      self.in_episode = False
      self.budget_remaining = LEAD_EPISODE_BUDGET

    if self.in_episode:
      v_target_mph = max(lead.vLeadK * MS_TO_MPH + TARGET_MARGIN_MPH, ABSOLUTE_FLOOR_MPH)
      self.decision = self._step_toward(v_target_mph, clamp_low=ABSOLUTE_FLOOR_MPH, clamp_high=None)
      self._log(v_current_mph, v_target_mph)
    else:
      goal = self.baseline_v_cruise_mph if self.baseline_v_cruise_mph is not None else v_cruise_mph
      self.decision = self._step_toward(goal, clamp_low=None, clamp_high=self.baseline_v_cruise_mph)
      self._log(v_current_mph, goal)

    self.was_gated_on = gated_on
    self.frame += 1
