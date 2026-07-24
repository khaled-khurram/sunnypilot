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

# Flag-file arming (2026-07-24) - NOT Params(), which hits a compiled-allowlist landmine
# on this prebuilt branch that always falls back to a hardcoded default no matter what
# gets set (same landmine CurveSpeedAdvisory/Phase3Armed/Phase3LeadArmed all hit - see
# research/phase3_controller_design.md §7.5). This is the exact same flag-file arming
# pattern already proven live during tonight's Q10 test protocol - existence-based, no
# rebuild required, no allowlist involved at all.
CURVE_ARM_FILE = "/data/phase3_curve_armed"
LEAD_ARM_FILE = "/data/phase3_lead_armed"

STEP_MPH = 1.0             # confirmed real shallow-press effect (Q10)
ABSOLUTE_FLOOR_MPH = 25.0  # EyeSight's own ACC floor (research/phase3_controller_design.md
                            # §3 hard safety bounds) - shared by every actuation feature,
                            # it's a property of the car, not of any one trigger source

# research/phase3_controller_design.md §2, tightened for the first live test specifically
# per explicit user request (2026-07-24) - "turned down further than defaults" language
# from the design doc's own Stage 2 rollout section. Loosen back toward 1.0s once this
# has held up over more than one session.
MIN_COMMAND_INTERVAL_S = 2.0

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
SESSION_COMMAND_CAP = 30

# cruise_button values, opendbc/car/subaru/carcontroller.py's own comment:
# 1 = main, 2 = set shallow, 3 = set deep, 4 = resume shallow, 5 = resume deep
BUTTON_SET_SHALLOW = 2
BUTTON_RESUME_SHALLOW = 4


def is_armed(flag_file: str) -> bool:
  return os.path.exists(flag_file)


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

  def new_cycle(self) -> None:
    self.written_this_cycle = False

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
  single override event (brake/steering/gas/real-button-press) must latch off every
  Phase 3 feature together, not just whichever one happened to be acting at that moment.
  Every controller must hold a reference to the SAME instance, not its own private copy.
  Re-arming requires a fresh explicit arm (new onroad session/process restart) - there is
  deliberately no reset() method here, matching research/phase3_controller_design.md §3's
  "not just skip one frame and silently resume the next" requirement.
  """

  def __init__(self):
    self.overridden = False

  def check(self, gas_pressed: bool, brake_pressed: bool, steering_pressed: bool, cruise_button: int) -> None:
    if gas_pressed or brake_pressed or steering_pressed or cruise_button != 0:
      self.overridden = True


def log_shadow_decision(feature: str, **fields) -> None:
  """Append one JSONL entry to the shared shadow log. Never raises into the control
  loop - a logging failure must never affect a decision."""
  entry = {"t": time.time(), "feature": feature, **fields}
  try:
    with open(SHADOW_LOG_FILE, "a") as f:
      f.write(json.dumps(entry) + "\n")
  except OSError:
    pass
