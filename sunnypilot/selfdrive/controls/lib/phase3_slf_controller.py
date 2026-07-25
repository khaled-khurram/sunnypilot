"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.common.realtime import DT_MDL
from openpilot.common.constants import CV
from openpilot.sunnypilot.selfdrive.controls.lib.phase3_shared import (
  STEP_MPH, ABSOLUTE_FLOOR_MPH, MIN_COMMAND_INTERVAL_S, GRACE_PERIOD_S, SETTLE_TIME_S,
  SLF_ARM_FILE, SEGMENT_DEBOUNCE_S, DELTA_NOISE_FLOOR_MPH,
  SELF_ATTRIBUTION_WINDOW_S, BUTTON_SET_SHALLOW, BUTTON_RESUME_SHALLOW,
  Phase3OverrideLatch, Phase3CommandArbiter, is_armed, log_shadow_decision,
)

MS_TO_MPH = CV.MS_TO_MPH


class Phase3SlfController:
  """
  Shadow/live Phase 3 speed-limit-following (research/phase3_speed_limit_following_design.md).
  Third Phase 3 actuation feature, alongside curve and lead. Follows posted speed limits
  in BOTH directions as of the §2 v1.1 update (2026-07-24) - walks the target toward a
  newly-detected, debounced posted limit, using the same shallow-SET/RESUME/shared-
  arbiter primitive as curve/lead. The buffer is no longer this file's own concern
  (2026-07-25) - longitudinal_planner.py now feeds this controller sunnypilot's native
  speed_limit_final (posted limit + the driver's own on-screen Offset Type/Value
  setting), so the "posted limit" this controller sees already has the driver's chosen
  buffer baked in.

  Auto-raise ceiling, the actual point of the v1.1 design: `target = min(baseline, new
  limit + buffer)` - never exceeds the driver's own real set speed (`baseline_v_cruise_mph`,
  same concept and same self-healing settle-resync as curve/lead's own baseline), even
  when a new zone's limit+buffer would allow more. This is what resolves v1's original
  "surprise acceleration to a number the driver never chose" concern - it structurally
  cannot happen, because the ceiling is always something the driver actually dialed in
  themselves, never a number this controller invented.

  Owns external button-press detection (§3/§6): since none of the three controllers can
  see a real button press directly (CS.buttonEvents is never populated on this preglobal
  car - same gap the Q11/dba5d57 crash postmortem found), "a button was pressed" is
  inferred from an unexplained v_cruise delta, cross-checked against the shared
  Phase3CommandArbiter's own last_write_t (which controller wrote last, and when - known
  with certainty, not inferred from magnitude/direction, which is what made the design
  doc's original proposal wrong: curve's own restore phase legitimately writes upward/
  RESUME commands too, so "upward = external" was never a safe assumption).

  As of 2026-07-25 this only ever pins SLF at the driver's corrected speed - it used to
  also kill curve+lead+SLF together whenever either had something in flight, but real
  road-trip telemetry showed that was the dominant cause of the system going dark (54% of
  all override trips over one ~6hr drive) and it kept misfiring even after tightening the
  attribution window, since this is always an inference, never a certainty. Only a real
  pedal press (brake/gas/steering) still latches everything off now. This detection only
  runs at all once Phase3SlfArmed is true; unarmed, SLF is fully inert and curve/lead's
  own independent pedal-only override.check() calls are unaffected either way.
  """

  def __init__(self, override_latch: Phase3OverrideLatch, command_arbiter: Phase3CommandArbiter):
    self._override_latch = override_latch
    self._arbiter = command_arbiter
    self.frame = -1
    self.armed = False
    self._read_arm_state()

    self.was_gated_on = False
    self.gated_on_since: float | None = None
    self.baseline_v_cruise_mph: float | None = None  # driver's own real set speed - the
                                                       # ceiling auto-raise can never exceed
    self.sim_target_mph: float | None = None
    self.last_command_t = -1e9
    self.t = 0.0
    self.last_v_cruise_mph: float | None = None

    self.current_segment_limit_mph: float | None = None
    self.segment_pending_limit_mph: float | None = None
    self.segment_pending_since: float | None = None
    self.segment_pinned = False
    self.slf_target_mph: float | None = None  # None = no active pursuit this segment

    self.decision = "inert-not-armed"
    self._last_logged_decision = None

  def _read_arm_state(self) -> None:
    if self.frame == -1 or self.frame % int(1.0 / DT_MDL) == 0:
      self.armed = is_armed(SLF_ARM_FILE)

  def _log(self, v_current_mph: float, v_target_mph: float) -> None:
    if self.decision == self._last_logged_decision:
      return
    self._last_logged_decision = self.decision
    log_shadow_decision(
      "slf",
      v_current_mph=round(v_current_mph, 2),
      v_target_mph=round(v_target_mph, 2) if v_target_mph is not None else None,
      decision=self.decision,
      segment_limit_mph=round(self.current_segment_limit_mph, 1) if self.current_segment_limit_mph is not None else None,
      segment_pinned=self.segment_pinned,
      baseline_v_cruise_mph=round(self.baseline_v_cruise_mph, 2) if self.baseline_v_cruise_mph is not None else None,
      sim_target_mph=round(self.sim_target_mph, 2) if self.sim_target_mph is not None else None,
      override_reason=self._override_latch.trip_reason,
    )

  def _step_toward(self, goal_mph: float) -> str:
    """Bidirectional as of v1.1 - same shared shallow-step primitive curve/lead use.
    The goal itself is always already capped at baseline by the caller (§1's
    min(baseline, limit+buffer) formula), so no separate clamp_high is needed here -
    unlike curve/lead's restore path, which clamps because its goal IS the baseline."""
    if abs(goal_mph - self.sim_target_mph) < DELTA_NOISE_FLOOR_MPH:
      return "hold"
    if (self.t - self.last_command_t) < MIN_COMMAND_INTERVAL_S:
      return "hold-rate-limited"
    step = STEP_MPH if goal_mph > self.sim_target_mph else -STEP_MPH
    button_value = BUTTON_RESUME_SHALLOW if step > 0 else BUTTON_SET_SHALLOW
    if not self._arbiter.try_write(button_value):
      return "hold-arbiter"
    new_target = self.sim_target_mph + step
    self.sim_target_mph = new_target
    self.last_command_t = self.t
    return "restore" if step > 0 else "fire"

  def update(self, speed_limit_mph: float | None, long_enabled: bool, v_ego: float, v_cruise: float,
             gas_pressed: bool, brake_pressed: bool, steering_pressed: bool,
             curve_active: bool, lead_active: bool) -> None:
    self._read_arm_state()
    self.t += DT_MDL

    v_current_mph = v_ego * MS_TO_MPH
    v_cruise_mph = v_cruise * MS_TO_MPH

    gated_on = self.armed and long_enabled

    if gated_on and not self.was_gated_on:
      self.gated_on_since = self.t
      self.baseline_v_cruise_mph = v_cruise_mph  # initial snapshot - self-heals below if wrong

    # Pedal overrides: identical, unconditional, shared behavior with curve/lead - not
    # touched by this feature's own context-gating logic, which only applies to buttons.
    if gated_on and self.gated_on_since is not None and (self.t - self.gated_on_since) >= GRACE_PERIOD_S:
      self._override_latch.check(gas_pressed, brake_pressed, steering_pressed)

    if not gated_on:
      self.decision = "inert-not-armed"
      self.was_gated_on = False
      # Don't let any state leak across an arm-cycle boundary into an unrelated later
      # one (cruise disengage/re-engage, or the arm flag toggling) - a pin, a remembered
      # segment limit, or a stale baseline from one drive context has no business
      # influencing a completely different later one.
      self.last_v_cruise_mph = None
      self.baseline_v_cruise_mph = None
      self.current_segment_limit_mph = None
      self.segment_pending_limit_mph = None
      self.segment_pending_since = None
      self.segment_pinned = False
      self.slf_target_mph = None
      self.frame += 1
      return

    if self._override_latch.overridden:
      self.decision = "latched-off"
      self._log(v_current_mph, v_cruise_mph)
      self.was_gated_on = gated_on
      self.frame += 1
      return

    if self.sim_target_mph is None:
      self.sim_target_mph = v_cruise_mph

    # Resync sim_target AND baseline after a genuine idle period, same self-healing fix
    # as curve/lead's own V_CRUISE_MAX-race postmortem - but ONLY before the first-ever
    # confirmed segment (self.current_segment_limit_mph is None). Two real bugs caught
    # by testing before shipping, both from over-generalizing that self-heal window:
    # (1) gating on "converged to slf_target" instead overwrote baseline with the
    # already-descended value the moment a held-down segment converged (sitting at 50 in
    # a 45mph town is the ongoing correct state for that whole zone, not a transient
    # that's "done" like curve's restore is); (2) using slf_target_mph itself as the gate
    # still broke once auto-raise was added, because releasing a constraint set
    # slf_target_mph back toward baseline rather than to None, so gating on "is it None"
    # never actually re-opened after the first segment. Gating on "has any segment EVER
    # been confirmed" instead closes this self-heal window permanently after the first
    # one - by design: once real per-segment logic is running, baseline only ever
    # changes again via an explicit driver correction (the pin-capture path below), never
    # by silently re-sampling whatever v_cruise happens to read at an idle moment.
    if self.current_segment_limit_mph is None and (self.t - self.last_command_t) > SETTLE_TIME_S:
      self.sim_target_mph = v_cruise_mph
      self.baseline_v_cruise_mph = v_cruise_mph

    # --- External button-press detection -> pin, never kill (2026-07-25) ---
    # Used to branch on curve_active/lead_active and call self._override_latch.trip_button()
    # to kill all three features when either had something in flight - removed after a real
    # ~6hr road-trip drive showed this was the dominant cause of the system going dark (54%
    # of all override trips) and was STILL misfiring even after tightening the attribution
    # window, because CS.buttonEvents is never populated on this car: "a button was pressed"
    # is always an inference here, never a certainty, and three live features shouldn't die
    # on an unreliable inference. Now unconditional: any detected external press pins SLF at
    # the driver's corrected speed (updating baseline too, same as the old dormant-only
    # path) regardless of what curve/lead are doing - the correction still registers, curve/
    # lead simply keep pursuing their own goals uninterrupted. Only a real pedal press
    # (brake/gas/steering, via override_latch.check()) still latches everything off.
    if self.last_v_cruise_mph is not None:
      delta = v_cruise_mph - self.last_v_cruise_mph
      if abs(delta) >= DELTA_NOISE_FLOOR_MPH:
        self_caused = (self.t - self._arbiter.last_write_t) < SELF_ATTRIBUTION_WINDOW_S
        if not self_caused:
          self.segment_pinned = True
          self.slf_target_mph = v_cruise_mph
          self.sim_target_mph = v_cruise_mph
          self.baseline_v_cruise_mph = v_cruise_mph
    self.last_v_cruise_mph = v_cruise_mph

    if self._override_latch.overridden:
      # Nothing between here and the earlier check() call can trip this anymore (button
      # detection above only pins now, never trips the latch) - kept as a cheap defensive
      # re-check rather than assumed safe to remove.
      self.decision = "latched-off"
      self._log(v_current_mph, v_cruise_mph)
      self.was_gated_on = gated_on
      self.frame += 1
      return

    # --- Segment/limit tracking (§1/§5, bidirectional as of v1.1) ---
    if speed_limit_mph is not None:
      is_new_reading = (self.current_segment_limit_mph is None
                         or abs(speed_limit_mph - self.current_segment_limit_mph) > 0.5)
      if is_new_reading:
        if (self.segment_pending_limit_mph is None
            or abs(speed_limit_mph - self.segment_pending_limit_mph) > 0.5):
          # A different candidate than what we were already debouncing - restart the
          # debounce window rather than counting time toward the old candidate.
          self.segment_pending_limit_mph = speed_limit_mph
          self.segment_pending_since = self.t
        elif (self.t - self.segment_pending_since) >= SEGMENT_DEBOUNCE_S:
          # Confirmed real segment change - fresh, unconstrained recompute (§5), any
          # earlier pin is explicitly released here, not carried over. v1.1: works for
          # a limit going up OR down - the min(baseline, ...) ceiling is what keeps the
          # up direction safe, not a one-directional guard on the delta's sign.
          self.current_segment_limit_mph = self.segment_pending_limit_mph
          self.segment_pending_limit_mph = None
          self.segment_pinned = False
          # No buffer added here (2026-07-25, was SLF_BUFFER_MPH=5.0 hardcoded) - the
          # incoming speed_limit_mph is now longitudinal_planner.py's speed_limit_final,
          # which already includes sunnypilot's own native, on-screen Offset Type/Value
          # setting. Adding a second buffer here would double it.
          candidate = self.current_segment_limit_mph
          ceiling = self.baseline_v_cruise_mph if self.baseline_v_cruise_mph is not None else candidate
          # min(candidate, ceiling) already handles both directions in one formula: if
          # this zone's own limit+buffer is genuinely lower than the ceiling, that's the
          # real constraint to hold at; if it's at or above the ceiling, this reduces to
          # exactly the ceiling itself (= baseline) - meaning "pursue a full return to
          # what the driver actually set," which is auto-raise, not a special case.
          self.slf_target_mph = max(min(candidate, ceiling), ABSOLUTE_FLOOR_MPH)
      else:
        self.segment_pending_limit_mph = None  # reading matches current segment, nothing pending

    # Yield entirely to a higher-priority in-flight episode (2026-07-24 fix): curve/lead
    # each maintain their own sim_target independent of SLF's, and the arbiter only ever
    # blocked same-cycle collisions - it never stopped SLF and lead/curve alternating
    # writes in adjacent cycles, each chasing a different goal on the same physical
    # Cruise_Set_Speed. Real telemetry showed this exact oscillation (74->78 restore vs.
    # 73->70 descent, same few seconds, 2026-07-24 20:10:12-16). Skipping SLF's own step
    # completely while curve or lead has something active - not just losing ties - closes
    # that gap; SLF resumes on its own goal the moment both go dormant again.
    if curve_active or lead_active:
      self.decision = "hold-yielding"
    elif self.slf_target_mph is not None and not self.segment_pinned:
      self.decision = self._step_toward(self.slf_target_mph)
    else:
      self.decision = "hold"
    self._log(v_current_mph, self.slf_target_mph if self.slf_target_mph is not None else v_cruise_mph)

    self.was_gated_on = gated_on
    self.frame += 1
