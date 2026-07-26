"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from cereal import custom

from openpilot.common.params import Params, UnknownKeyName
from openpilot.common.realtime import DT_MDL
from openpilot.common.constants import CV
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP

EventNameSP = custom.OnroadEventSP.EventName

# NOT part of the shipped LeadClosingAdvisory feature (that stays a one-time
# nudge, unchanged). This is a standalone validation tool: before spending
# money/effort on a physical button-press microcontroller (Phase 3 hardware),
# repeatedly surface exactly what an automated system would want done -
# "ease off, target ~X mph" - so the driver can execute it manually with the
# real buttons and judge, in real driving, whether the underlying decision
# logic actually produces good outcomes (smooth convergence, EyeSight
# actually locking on) before ever building an actuator to do it for real.
#
# Off by default - opt in per drive via:
#   Params().put_bool("LeadClosingTestGuidance", True)

MIN_ADVISORY_SPEED = 50 * CV.MPH_TO_MS
CLOSING_VREL_THRESHOLD = -3.0    # m/s, same detection threshold as the shipped advisory
SUSTAIN_TIME = 0.5               # seconds
NO_RECENT_PEDAL_TIME = 3.0       # seconds
TARGET_MARGIN = 4 * CV.MPH_TO_MS  # buffer above the lead's estimated speed - not asking
                                   # to match it exactly, just enough for EyeSight to get a lock
CONVERGED_TOLERANCE = 2 * CV.MPH_TO_MS  # stop prompting once set-speed is this close to target
REPEAT_INTERVAL = 5.0             # seconds between repeated prompts while still above target -
                                   # short enough to be useful in real time, long enough to see
                                   # the effect of the last press before the next prompt


class LeadClosingTestGuidanceHelper:
  def __init__(self):
    self._params = Params()
    self.frame = -1
    try:
      self.enabled = self._params.get_bool("LeadClosingTestGuidance")
    except UnknownKeyName:
      self.enabled = False  # opt-in test tool, unlike the shipped advisory
    self.closing_since: float | None = None
    self.last_pedal_t = -1e9
    self.last_fire_t = -1e9
    self.t = 0.0
    self.v_target = 0.0
    self.active = False

  def _read_params(self) -> None:
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      try:
        self.enabled = self._params.get_bool("LeadClosingTestGuidance")
      except UnknownKeyName:
        self.enabled = False

  def update(self, lead, long_enabled: bool, v_ego: float, v_cruise_cluster: float,
             gas_pressed: bool, brake_pressed: bool, events_sp: EventsSP) -> None:
    self._read_params()
    self.t += DT_MDL

    if gas_pressed or brake_pressed:
      self.last_pedal_t = self.t

    closing = lead.status and lead.vRel < CLOSING_VREL_THRESHOLD
    if closing:
      if self.closing_since is None:
        self.closing_since = self.t
    else:
      self.closing_since = None

    sustained = self.closing_since is not None and (self.t - self.closing_since) >= SUSTAIN_TIME
    no_recent_pedal = (self.t - self.last_pedal_t) >= NO_RECENT_PEDAL_TIME

    self.v_target = lead.vLeadK + TARGET_MARGIN
    above_target = v_cruise_cluster > (self.v_target + CONVERGED_TOLERANCE)

    self.active = bool(self.enabled and long_enabled and v_ego >= MIN_ADVISORY_SPEED
                        and sustained and no_recent_pedal and above_target)

    if self.active and (self.t - self.last_fire_t) >= REPEAT_INTERVAL:
      events_sp.add(EventNameSP.leadClosingTestGuidance)
      self.last_fire_t = self.t

    self.frame += 1
