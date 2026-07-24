"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import json
import time

# Shadow-mode-only: this path is never read by carcontroller.py or any car-interface
# code, by any Phase 3 feature. This run cannot affect real CAN output even in principle.
SHADOW_LOG_FILE = "/data/phase3_shadow_log.jsonl"

STEP_MPH = 1.0             # confirmed real shallow-press effect (Q10)
ABSOLUTE_FLOOR_MPH = 25.0  # EyeSight's own ACC floor (research/phase3_controller_design.md
                            # §3 hard safety bounds) - shared by every actuation feature,
                            # it's a property of the car, not of any one trigger source
MIN_COMMAND_INTERVAL_S = 1.0  # research/phase3_controller_design.md §2 - one simulated
                                # command per second max, shared by every actuation
                                # feature. Caught missing entirely during verification
                                # testing (2026-07-24): without this, steps fired every
                                # single planner frame (~50ms), not once per second.


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
