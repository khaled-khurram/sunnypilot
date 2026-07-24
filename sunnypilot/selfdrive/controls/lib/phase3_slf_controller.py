"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.common.realtime import DT_MDL
from openpilot.common.constants import CV
from openpilot.sunnypilot.selfdrive.controls.lib.phase3_shared import (
  STEP_MPH, ABSOLUTE_FLOOR_MPH, MIN_COMMAND_INTERVAL_S, GRACE_PERIOD_S, SETTLE_TIME_S,
  SLF_ARM_FILE, SLF_BUFFER_MPH, SEGMENT_DEBOUNCE_S, DELTA_NOISE_FLOOR_MPH,
  SELF_ATTRIBUTION_WINDOW_S, BUTTON_SET_SHALLOW, Phase3OverrideLatch, Phase3CommandArbiter,
  is_armed, log_shadow_decision,
)

MS_TO_MPH = CV.MS_TO_MPH


class Phase3SlfController:
  """
  Shadow/live Phase 3 speed-limit-following (research/phase3_speed_limit_following_design.md).
  Third Phase 3 actuation feature, alongside curve and lead. Decrease-only for v1 (§2 of
  the design doc): walks the target down toward a newly-detected, debounced posted speed
  limit + SLF_BUFFER_MPH, using the same shallow-SET/shared-arbiter primitive as curve/lead.

  Owns the new context-gated button-press routing (§3/§6): since none of the three
  controllers can see a real button press directly (CS.buttonEvents is never populated
  on this preglobal car - same gap the Q11/dba5d57 crash postmortem found), "a button was
  pressed" is inferred from an unexplained v_cruise delta, cross-checked against the
  shared Phase3CommandArbiter's own last_write_t (which controller wrote last, and when -
  known with certainty, not inferred from magnitude/direction, which is what made the
  design doc's original proposal wrong: curve's own restore phase legitimately writes
  upward/RESUME commands too, so "upward = external" was never a safe assumption).

  Safety choice made during implementation, not in the original design doc: this new
  routing only changes behavior once Phase3SlfArmed is actually true. If SLF is unarmed,
  a detected button press always falls through to the existing unconditional
  Phase3OverrideLatch.check()-equivalent kill - curve/lead's already-tested override
  behavior is byte-for-byte unchanged until this feature is deliberately turned on.
  """

  def __init__(self, override_latch: Phase3OverrideLatch, command_arbiter: Phase3CommandArbiter):
    self._override_latch = override_latch
    self._arbiter = command_arbiter
    self.frame = -1
    self.armed = False
    self._read_arm_state()

    self.was_gated_on = False
    self.gated_on_since: float | None = None
    self.sim_target_mph: float | None = None
    self.last_command_t = -1e9
    self.t = 0.0
    self.last_v_cruise_mph: float | None = None

    self.current_segment_limit_mph: float | None = None
    self.segment_pending_limit_mph: float | None = None
    self.segment_pending_since: float | None = None
    self.segment_pinned = False
    self.slf_target_mph: float | None = None  # None = no active descent this segment

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
      sim_target_mph=round(self.sim_target_mph, 2) if self.sim_target_mph is not None else None,
      override_reason=self._override_latch.trip_reason,
    )

  def _step_toward(self, goal_mph: float) -> str:
    if abs(goal_mph - self.sim_target_mph) < DELTA_NOISE_FLOOR_MPH:
      return "hold"
    if (self.t - self.last_command_t) < MIN_COMMAND_INTERVAL_S:
      return "hold-rate-limited"
    # Decrease-only v1 (§2): SLF only ever steps down. If goal is somehow above
    # sim_target (segment recompute raced with a pin release), just stop, don't climb -
    # that would be the deferred v1.1 auto-raise behavior, not shipped here.
    if goal_mph >= self.sim_target_mph:
      return "hold"
    if not self._arbiter.try_write(BUTTON_SET_SHALLOW):
      return "hold-arbiter"
    self.sim_target_mph -= STEP_MPH
    if self.sim_target_mph < goal_mph:
      self.sim_target_mph = goal_mph
    self.last_command_t = self.t
    return "fire"

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

    # Pedal overrides: identical, unconditional, shared behavior with curve/lead - not
    # touched by this feature's own context-gating logic, which only applies to buttons.
    if gated_on and self.gated_on_since is not None and (self.t - self.gated_on_since) >= GRACE_PERIOD_S:
      self._override_latch.check(gas_pressed, brake_pressed, steering_pressed)

    if not gated_on:
      self.decision = "inert-not-armed"
      self.was_gated_on = False
      # Don't let any state leak across an arm-cycle boundary into an unrelated later
      # one (cruise disengage/re-engage, or the arm flag toggling) - a pin or a
      # remembered segment limit from one drive context has no business influencing a
      # completely different later one. Found during final review, not the original
      # design pass - only last_v_cruise_mph had this treatment initially.
      self.last_v_cruise_mph = None
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

    # Resync sim_target after a genuine idle period - same fix as curve/lead's own
    # V_CRUISE_MAX-race postmortem, same reasoning: self-heals regardless of how it got
    # wrong, and is the only way this controller notices a real, unrelated set-speed
    # change that happened while SLF had no active descent of its own.
    descending = self.slf_target_mph is not None and not self.segment_pinned
    if not descending and (self.t - self.last_command_t) > SETTLE_TIME_S:
      self.sim_target_mph = v_cruise_mph

    # --- External button-press detection + context-gated routing (§3/§4/§6) ---
    if self.last_v_cruise_mph is not None:
      delta = v_cruise_mph - self.last_v_cruise_mph
      if abs(delta) >= DELTA_NOISE_FLOOR_MPH:
        self_caused = (self.t - self._arbiter.last_write_t) < SELF_ATTRIBUTION_WINDOW_S
        if not self_caused:
          if curve_active or lead_active:
            # A transient curve/lead event is actually in flight right now - this press
            # is treated exactly as today's unconditional behavior: full, session-long
            # kill of curve+lead+SLF together. Only reachable at all when SLF is armed;
            # when unarmed, gated_on is False above and this whole block never runs, so
            # curve/lead's own pedal-only override.check() calls are the only path -
            # unchanged, exactly as before this feature existed.
            self._override_latch.trip_button()
          else:
            # Both dormant - this can't be "about" an active curve/lead event, because
            # neither has one running. Pin it: hold here for the rest of this segment,
            # don't fight the driver's correction, and don't touch the shared latch.
            self.segment_pinned = True
            self.slf_target_mph = v_cruise_mph
            self.sim_target_mph = v_cruise_mph
    self.last_v_cruise_mph = v_cruise_mph

    if self._override_latch.overridden:
      # trip_button() above may have just fired this frame - re-check before proceeding,
      # same discipline as the early-return above, not a redundant no-op.
      self.decision = "latched-off"
      self._log(v_current_mph, v_cruise_mph)
      self.was_gated_on = gated_on
      self.frame += 1
      return

    # --- Segment/limit tracking (§1/§5) ---
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
          # earlier pin is explicitly released here, not carried over.
          self.current_segment_limit_mph = self.segment_pending_limit_mph
          self.segment_pending_limit_mph = None
          self.segment_pinned = False
          candidate = self.current_segment_limit_mph + SLF_BUFFER_MPH
          if candidate < self.sim_target_mph:  # decrease-only (§2)
            self.slf_target_mph = max(candidate, ABSOLUTE_FLOOR_MPH)
          else:
            self.slf_target_mph = None
      else:
        self.segment_pending_limit_mph = None  # reading matches current segment, nothing pending

    if self.slf_target_mph is not None and not self.segment_pinned:
      self.decision = self._step_toward(self.slf_target_mph)
    else:
      self.decision = "hold"
    self._log(v_current_mph, self.slf_target_mph if self.slf_target_mph is not None else v_cruise_mph)

    self.was_gated_on = gated_on
    self.frame += 1
