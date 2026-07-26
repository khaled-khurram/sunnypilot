"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from cereal import custom

from openpilot.sunnypilot.selfdrive.controls.lib.phase3_shared import Phase3OverrideLatch
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP

EventNameSP = custom.OnroadEventSP.EventName

# Repurposed 2026-07-26: this was a standalone closing-lead validation-tool prompt,
# opt-in via Params().get_bool("LeadClosingTestGuidance"). That Params key was never
# added to the compiled allowlist on this prebuilt branch (confirmed absent from
# common/params_keys.h, same landmine as CURVE_ARM_FILE's docstring describes for
# Phase3Armed/CurveSpeedAdvisory) - self.enabled always silently resolved to False, so
# this helper's original trigger has never actually fired on this build. Nothing live
# is being removed by repurposing it.
#
# New job: fire the Phase 3 override-trip one-shot alert (see
# phase3_shared.Phase3OverrideLatch's own docstring - "if I just tap brakes once,
# everything goes dark," with no on-screen feedback of any kind until this). Reuses
# this EventNameSP.leadClosingTestGuidance slot specifically because it's the one
# already-compiled, already-unused enum member available - a brand-new EventNameSP
# member needs capnp codegen, which needs a build system (SConstruct) that doesn't
# exist anywhere in this tree.


class LeadClosingTestGuidanceHelper:
  def __init__(self):
    self._last_seen_trip_seq = 0
    # Inert - kept only so longitudinal_planner.py's publish_longitudinal_plan_sp()
    # doesn't need to change; these fields' original closing-speed meaning no longer
    # applies and nothing reads them for real anymore.
    self.v_target = 0.0
    self.active = False

  def update(self, override_latch: Phase3OverrideLatch, events_sp: EventsSP) -> None:
    """Fires once per NEW override-latch trip (trip_seq edge), not every cycle the
    latch stays tripped - see Phase3OverrideLatch.trip_seq's own docstring. A fresh
    trip can only happen after clear_on_reengage() resets overridden, so this can't
    re-fire mid-latch."""
    if override_latch.overridden and override_latch.trip_seq != self._last_seen_trip_seq:
      events_sp.add(EventNameSP.leadClosingTestGuidance)
      self._last_seen_trip_seq = override_latch.trip_seq
