"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import os
import json
import time

# Shadow log: written by every Phase 3 feature regardless of arm state, purely for
# observability. Never read by carcontroller.py or any car-interface code.
SHADOW_LOG_FILE = "/data/phase3_shadow_log.jsonl"

# Real command path (2026-07-24, first live-test authorization): carcontroller.py's
# PREGLOBAL button block reads THIS file and, if fresh and the override-guarantee
# passes, overrides the real cruise_button value. This is the one file in this whole
# feature that actually matters for CAN output - everything upstream of it is decision
# logic, this is the only thing with real-world effect.
COMMAND_FILE = "/data/phase3_button_command"

# Observability-only (2026-07-24): written by carcontroller.py's PREGLOBAL block from
# EyeSight's own real, unmodified ES_Distance message every 5 frames - Car_Follow is
# EyeSight's own lead-lock bit, Close_Distance a bounded (0-5m per the DBC scale, likely
# a dash-icon proximity value, not a true following-gap distance) closeness reading.
# Read into every shadow log entry below so real drives answer, with data instead of a
# guess, whether Phase 3 acted before or after EyeSight already had its own lock - see
# the 2026-07-24 drive postmortem (80->71 lead-closing drop, EyeSight locked throughout).
# Purely additive: nothing here changes any controller's decision or gating.
EYESIGHT_STATE_FILE = "/data/phase3_eyesight_state.txt"
EYESIGHT_STATE_STALENESS_S = 1.0  # generous - shadow-log-only, nothing time-critical reads this

# Flag-file arming (2026-07-24) - NOT Params(), which hits a compiled-allowlist landmine
# on this prebuilt branch that always falls back to a hardcoded default no matter what
# gets set (same landmine CurveSpeedAdvisory/Phase3Armed/Phase3LeadArmed all hit - see
# research/phase3_controller_design.md §7.5). This is the exact same flag-file arming
# pattern already proven live during tonight's Q10 test protocol - existence-based, no
# rebuild required, no allowlist involved at all.
CURVE_ARM_FILE = "/data/phase3_curve_armed"
LEAD_ARM_FILE = "/data/phase3_lead_armed"
SLF_ARM_FILE = "/data/phase3_slf_armed"  # speed-limit-following, added 2026-07-24 -
                                           # same isolated-rollout-risk pattern as lead

# UI status transport (2026-07-26): plannerd writes, both the UI process (status dots)
# and selfdrived (one-shot override alert text) read - independently, plain-file, same
# reasoning as CURVE_ARM_FILE above. Not Params, not a new capnp field - both hit the
# same compiled-allowlist/codegen landmine on this prebuilt branch.
UI_STATUS_FILE = "/data/phase3_ui_status.json"
UI_STATUS_WRITE_INTERVAL_FRAMES = 5   # ~0.25s @ 20Hz plannerd - cheap, still sub-300ms
                                       # latency for dot transitions and the trip_seq
                                       # edge signal
UI_STATUS_STALENESS_S = 2.0

# Speed-limit-following constants (2026-07-24). The buffer itself (formerly a hardcoded
# SLF_BUFFER_MPH=5.0 here, hitting the same compiled-Params-allowlist landmine every other
# new Phase3 param has on this prebuilt branch) was removed 2026-07-25 in favor of
# reusing sunnypilot's own native, already-on-screen, already-compiled "Speed Limit"
# settings page (Offset Type/Value) - see longitudinal_planner.py, which now feeds this
# controller speed_limit_final instead of the raw posted limit. No allowlist workaround
# needed since those Params keys are already native/known.
SEGMENT_DEBOUNCE_S = 2.5  # guessed, not measured - see the design doc's own §7/§10 flag
DELTA_NOISE_FLOOR_MPH = 0.4  # below this, a v_cruise frame-to-frame change isn't real
SELF_ATTRIBUTION_WINDOW_S = 0.7  # was 0.3 (Q6's single-press ~100ms round-trip) - too
                                   # tight for a rapid multi-step burst's settling tail.
                                   # Real telemetry (2026-07-24 drive) caught 2 false
                                   # "button pressed" trips at 0.40-0.45s after the last
                                   # burst write, both misattributed as external and
                                   # session-killing all three features. Widened to
                                   # MIN_COMMAND_INTERVAL_S (0.4s) + ~0.3s margin so a
                                   # burst's last step's real CAN-level settling has room
                                   # to land before being read as unexplained.

STEP_MPH = 1.0             # confirmed real shallow-press effect (Q10)
ABSOLUTE_FLOOR_MPH = 25.0  # EyeSight's own ACC floor (research/phase3_controller_design.md
                            # §3 hard safety bounds) - shared by every actuation feature,
                            # it's a property of the car, not of any one trigger source

# research/phase3_controller_design.md §2, tightened for the first live test specifically
# per explicit user request (2026-07-24) - "turned down further than defaults" language
# from the design doc's own Stage 2 rollout section.
#
# Tightened again 2026-07-24, same night: quantified that at shallow-only 1mph/2.0s, the
# TARGET itself descends at 0.5 mph/s - slower than EyeSight's own physical comfort-tuned
# decel ceiling of ~1.94 mph/s (research/phase3_controller_design.md §7) - meaning button
# cadence, not EyeSight's own braking, was the actual bottleneck on how fast the car could
# respond to a closing situation. Archive-mined (research/button_cadence_response_curve.md,
# 87 clean same-magnitude bursts, 200ms-several seconds, zero debounce collapse or
# overshoot anywhere in that range) before changing this - not guessed. 0.4s chosen from
# the well-supported 300-500ms range (not the single 200ms data point, which isn't a
# margin, just one clean observation): 1mph/0.4s = 2.5 mph/s, comfortably above EyeSight's
# own ceiling with real headroom, while sitting nowhere near the confirmed-bad ~50ms zone
# or the untested 50-200ms gap between Q10's live test and this archive read.
MIN_COMMAND_INTERVAL_S = 0.4

# Added 2026-07-24, post-first-live-test finding: on the actual first drive, the override
# latch tripped in the very first gated-on frame for both features, before either ever
# got a chance to evaluate a real decision - "latched-off" was the first and only thing
# ever logged. Most likely cause: gas is very often still marginally pressed in the exact
# same frame a driver hits set/resume (you're usually still lightly accelerating right up
# to the moment of engaging) - an artifact of engagement timing, not a deliberate
# override. GRACE_PERIOD_S suppresses the override CHECK (not the feature itself - arming
# and gating are unaffected) for this long after cruise first becomes gated-on, so
# residual pedal/steering state from the engagement moment can't immediately and
# permanently kill the session before it ever ran once. Any override AFTER the grace
# window still latches off instantly and permanently, same as before - this only changes
# the first ~1.5s after engagement, not the "tap once, everything goes dark" behavior for
# a genuine mid-drive reaction.
GRACE_PERIOD_S = 1.5

# Added 2026-07-24, same first-drive postmortem as SESSION_COMMAND_CAP below: the
# baseline/sim_target snapshot taken once at the gated-on rising edge captured
# V_CRUISE_MAX (145kph/90.1mph, a fallback ceiling) instead of the real set speed,
# because v_cruise hadn't settled in that exact first frame - same race-condition
# category as GRACE_PERIOD_S above, different piece of state. Fix: after this long with
# no active intervention AND no recently-sent command (i.e. genuinely idle, not mid-
# restore), both baseline_v_cruise_mph and sim_target_mph resync to the real, current
# v_cruise - self-healing regardless of how either got wrong, and also tracks the driver
# adjusting their real set speed mid-drive via real button presses (which neither
# controller has any other way to observe - see the override-latch docstring for why
# real button presses aren't visible at this layer at all).
SETTLE_TIME_S = 2.0

# Real CAN staleness bound for the command file - tighter than the original design
# doc's "e.g. 500ms" sketch, since MIN_COMMAND_INTERVAL_S is now 2.0s: a fresh command
# should always be well under 300ms old by the time carcontroller.py's next 5-frame
# cycle reads it, so anything staler than that is either a stuck/delayed writer or a
# stale leftover file, not a live decision - fall back to plain relay either way.
COMMAND_STALENESS_S = 0.3

# Whole-drive hard backstop (2026-07-24, first live test only) - independent of and in
# ADDITION to each controller's own per-event budget, not a replacement for it (the
# per-event budgets were just fixed tonight to be realistically sized for a single
# curve/episode's own delta - shrinking them back down would reintroduce that exact bug,
# live this time). This is the old original §2 "MAX_COMMANDS_PER_SESSION" concept,
# reintroduced specifically as extra defense-in-depth for the very first live test, on
# top of (not instead of) the per-event sizing.
#
# Raised 2026-07-24, after the first real live drive: 30 was hit and exhausted by a
# SINGLE curve (~20+ writes) because MTSC's own live distance/target estimate keeps
# shifting slightly on approach, and the controller re-corrects on every shift, not
# just once per curve - a real, legitimate usage pattern, not a bug. With no reset
# mechanism for the rest of the drive, this silently blocked every subsequent curve
# and lead event for the remainder of the session (confirmed in the shadow log:
# curve 3 computed a correct, sensible request and logged "hold-arbiter" instead of
# firing it). 500 gives real headroom for many curves/episodes across a full drive
# (500 * MIN_COMMAND_INTERVAL_S = 200s of continuous firing before it would even
# trigger - still a real backstop against an actually-runaway policy bug, not a
# limit that legitimate use bumps into).
SESSION_COMMAND_CAP = 500

# cruise_button values, opendbc/car/subaru/carcontroller.py's own comment:
# 1 = main, 2 = set shallow, 3 = set deep, 4 = resume shallow, 5 = resume deep
BUTTON_SET_SHALLOW = 2
BUTTON_RESUME_SHALLOW = 4


def is_armed(flag_file: str) -> bool:
  return os.path.exists(flag_file)


def read_eyesight_state() -> tuple[bool | None, float | None]:
  """EyeSight's own real Car_Follow/Close_Distance, written by carcontroller.py's
  PREGLOBAL block (see EYESIGHT_STATE_FILE above). Returns (None, None) if the file is
  missing or stale - callers must treat that as "unknown," not "False"/"0", since a
  missing file just means carcontroller.py hasn't written one yet (e.g. right after
  boot) or the car isn't a PREGLOBAL Subaru at all."""
  try:
    with open(EYESIGHT_STATE_FILE) as f:
      raw = f.read().strip().split()
    car_follow, close_distance_m, ts = int(raw[0]), float(raw[1]), float(raw[2])
  except (FileNotFoundError, ValueError, IndexError, OSError):
    return None, None
  if time.time() - ts > EYESIGHT_STATE_STALENESS_S:
    return None, None
  return bool(car_follow), close_distance_m


class Phase3CommandArbiter:
  """Both curve and lead controllers can want to write a real command in the same
  planner cycle now that this is live, not shadow-only - only one physical button press
  can happen per cycle. Curve controller is called first in longitudinal_planner.py's
  update order and wins ties for free via `written_this_cycle` - a bounded, time-critical
  geometric event takes priority over a continuous one that can simply wait one more
  ~50ms cycle. Also enforces SESSION_COMMAND_CAP as a whole-drive backstop independent of
  either controller's own per-event budget."""

  def __init__(self):
    self.written_this_cycle = False
    self.total_commands_this_session = 0
    self.t = 0.0            # shared per-cycle clock, ticked once per planner frame -
                              # closely aligned with every controller's own self.t since
                              # all are constructed together and stepped in the same loop
    self.last_write_t = -1e9  # when ANY controller last wrote a real command - added
                                # 2026-07-24 for SLF's self-vs-external button-press
                                # attribution (research/phase3_speed_limit_following_design.md
                                # §4). This is deliberately NOT inferred from observed
                                # v_cruise deltas/magnitude matching - the design doc's
                                # original proposal ("any upward delta is unambiguously
                                # external") was factually wrong, since curve's own
                                # restore phase legitimately issues upward/RESUME writes
                                # too. The arbiter already knows with certainty whether
                                # ANY controller just wrote something - no inference needed.

  def new_cycle(self, dt: float) -> None:
    self.written_this_cycle = False
    self.t += dt

  def try_write(self, value: int) -> bool:
    if self.written_this_cycle or self.total_commands_this_session >= SESSION_COMMAND_CAP:
      return False
    try:
      with open(COMMAND_FILE, "w") as f:
        f.write(f"{value} {time.time()}\n")
    except OSError:
      return False
    self.written_this_cycle = True
    self.total_commands_this_session += 1
    self.last_write_t = self.t
    return True


def read_command_if_safe(gas_pressed: bool, brake_pressed: bool, steering_pressed: bool,
                          real_button_pressed: bool) -> int | None:
  """carcontroller.py's own independent final gate - re-checks the override guarantee
  here too, not just trusting plannerd's upstream decision (research/phase3_controller_design.md
  §3: "the override check must be the single first gate wrapping the entire 'maybe send
  a command' block"). Returns None (meaning: fall back to plain relay of CS.cruise_button,
  today's exact shipped behavior) unless the file exists, is fresh, and nothing overrides."""
  if gas_pressed or brake_pressed or steering_pressed or real_button_pressed:
    return None
  try:
    with open(COMMAND_FILE) as f:
      raw = f.read().strip().split()
    value, ts = int(raw[0]), float(raw[1])
  except (FileNotFoundError, ValueError, IndexError, OSError):
    return None
  if time.time() - ts > COMMAND_STALENESS_S:
    return None
  return value


class Phase3OverrideLatch:
  """
  Shared, session-long driver-override latch for ALL Phase 3 actuation features.
  User's own words (2026-07-24): "if I just tap brakes once, everything goes dark" - a
  single override event (brake/steering/gas) must latch off every Phase 3 feature
  together, not just whichever one happened to be acting at that moment. Every
  controller must hold a reference to the SAME instance, not its own private copy.
  Re-arms on a fresh cruise engagement (2026-07-24, corrected same night after the user
  clarified this in plain terms: "if I tap those override latches everything goes dark
  UNTIL I actually set the cruise again then it all comes back" - NOT "off for the rest
  of the drive, only clears on a full ignition cycle," which is what an earlier reading
  of "everything goes dark" had been implemented as. Real, named tradeoff: this is less
  sticky than a whole-drive lockout - a quick disengage/re-engage brings Phase 3 straight
  back regardless of whether whatever caused the override is still true. Built this way
  anyway because that's the explicit, twice-clarified ask, not a default worth assuming.

  Pedal-only as of 2026-07-25: an inferred external button press (`trip_button()`, now
  removed) used to latch everything off too when curve/lead had something in flight -
  real road-trip telemetry showed this was the dominant cause of the system going dark
  (54% of all trips over a ~6hr drive) and was still misfiring even after tightening its
  attribution window, because CS.buttonEvents is never populated on this car - "a button
  was pressed" was always an inference, never a certainty, and killing three live
  features on that inference cost more than it protected. A detected button press now
  routes to SLF's pin-and-hold behavior unconditionally instead (see
  phase3_slf_controller.py) - the correction still registers, but pedal input remains
  the only thing that latches everything off.
  """

  def __init__(self):
    self.overridden = False
    self.trip_reason: str | None = None  # e.g. "gas", "brake+steering" - which signal(s)
                                           # actually tripped it, added 2026-07-24 after
                                           # the first live drive left this undiagnosable
                                           # from the shadow log alone
    self.trip_seq = 0  # added 2026-07-26: monotonic, incremented only on the false->true
                         # transition - lets a consumer in a different process (the
                         # one-shot alert trigger) detect "this is a NEW trip" vs "still
                         # overridden from a previous cycle" without re-deriving edges
                         # from overridden alone, which can't distinguish the two.

  def clear_on_reengage(self) -> None:
    """Called once per planner cycle from longitudinal_planner.py on a real rising edge
    of cruise-enabled (not every frame it's on) - the actual "set the cruise again"
    moment. Safe to call even when nothing is tripped (no-op)."""
    self.overridden = False
    self.trip_reason = None

  def check(self, gas_pressed: bool, brake_pressed: bool, steering_pressed: bool) -> None:
    # No real-button-press check here (2026-07-24 postmortem): CS.buttonEvents, the
    # schema-correct signal, is never populated at all for this preglobal car (same gap
    # tonight's earlier MADS investigation already found). The raw cruise_button value
    # only exists inside carcontroller.py's own scope, on a different object than the
    # capnp-published CarState plannerd reads here - crashed plannerd outright when this
    # tried to read it (AttributeError: struct has no such member). NOT a safety gap:
    # carcontroller.py's own independent, unconditional final gate (phase3_shared
    # constants duplicated there, see that file's own _phase3_read_command_if_safe)
    # already re-checks the real button-press condition correctly every cycle from the
    # object that actually has it - this latch not seeing it doesn't weaken that.
    if self.overridden:
      return  # already tripped - don't overwrite the reason that actually caused it
    reasons = [name for name, pressed in
               (("gas", gas_pressed), ("brake", brake_pressed), ("steering", steering_pressed)) if pressed]
    if reasons:
      self.overridden = True
      self.trip_reason = "+".join(reasons)
      self.trip_seq += 1


def write_ui_status(curve_armed: bool, curve_active: bool, lead_armed: bool, lead_active: bool,
                     slf_armed: bool, slf_active: bool, overridden: bool,
                     trip_reason: str | None, trip_seq: int) -> None:
  """Best-effort, atomic write of the UI/alert status blob. Same 'never raise into the
  control loop' contract as log_shadow_decision below - a failed write here must never
  affect a planner cycle."""
  payload = {
    "t": time.time(),
    "curve": {"armed": curve_armed, "active": curve_active},
    "lead": {"armed": lead_armed, "active": lead_active},
    "slf": {"armed": slf_armed, "active": slf_active},
    "overridden": overridden, "trip_reason": trip_reason, "trip_seq": trip_seq,
  }
  try:
    tmp = UI_STATUS_FILE + ".tmp"
    with open(tmp, "w") as f:
      f.write(json.dumps(payload))
    os.replace(tmp, UI_STATUS_FILE)  # atomic rename - no half-written reads
  except OSError:
    pass


def read_ui_status() -> dict | None:
  """Used by selfdrived's one-shot alert text lookup (a different process than the
  writer). Returns None if the file is missing, malformed, or stale - callers must
  treat that as 'no status available,' not 'everything off,' though in practice a
  missing/stale file does mean plannerd isn't running Phase 3 right now either way."""
  try:
    with open(UI_STATUS_FILE) as f:
      status = json.loads(f.read())
    if time.time() - status["t"] > UI_STATUS_STALENESS_S:
      return None
  except (FileNotFoundError, ValueError, KeyError, OSError):
    return None
  return status


def log_shadow_decision(feature: str, **fields) -> None:
  """Append one JSONL entry to the shared shadow log. Never raises into the control
  loop - a logging failure must never affect a decision."""
  car_follow, close_distance_m = read_eyesight_state()
  entry = {"t": time.time(), "feature": feature, "eyesight_car_follow": car_follow,
           "eyesight_close_distance_m": close_distance_m, **fields}
  try:
    with open(SHADOW_LOG_FILE, "a") as f:
      f.write(json.dumps(entry) + "\n")
  except OSError:
    pass
