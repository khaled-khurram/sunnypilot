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

# Highway-only - below this, following-distance behavior is a different problem
# and false-positive risk (merges, lane changes, stop-and-go) goes way up.
MIN_ADVISORY_SPEED = 50 * CV.MPH_TO_MS

# Tuned from real telemetry (research/lead_vehicle_warning_analysis.md, 2026-07-23):
# 14 real historical "driver had to brake" episodes all showed the vision
# model detecting the closing lead well beforehand at this threshold.
CLOSING_VREL_THRESHOLD = -3.0  # m/s (~6.7 mph closing)
SUSTAIN_TIME = 0.5             # seconds - filters single-frame noise
NO_RECENT_PEDAL_TIME = 3.0     # seconds - don't fire if already reacting
DEBOUNCE_TIME = 20.0           # seconds - once per encounter, matches the natural
                                # episode-clustering window found in the real data


class LeadClosingAdvisoryHelper:
  """
  Advisory alert when a vision-tracked lead vehicle is closing at highway
  speed with no recent driver reaction. Read-only, no actuation - same
  pattern as CurveAdvisoryHelper, different trigger source. Deliberately a
  calm nudge, not a collision alarm: real telemetry showed only ~22% of
  these episodes actually require a brake, so this can't reliably predict
  severity - it just surfaces data the vision model already computes but
  nothing currently uses.
  """

  def __init__(self):
    self._params = Params()
    self.frame = -1
    try:
      self.enabled = self._params.get_bool("LeadClosingAdvisory")
    except UnknownKeyName:
      self.enabled = True
    self.closing_since: float | None = None
    self.last_pedal_t = -1e9
    self.last_fire_t = -1e9
    self.t = 0.0

  def _read_params(self) -> None:
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      try:
        self.enabled = self._params.get_bool("LeadClosingAdvisory")
      except UnknownKeyName:
        self.enabled = True

  def update(self, lead, long_enabled: bool, v_ego: float,
             gas_pressed: bool, brake_pressed: bool, events_sp: EventsSP) -> None:
    # `lead` is a radarState.leadOne capnp reader (cereal/log.capnp LeadData) -
    # not opendbc's structs (that's for CarState/CarParams, a different schema)
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
    debounced = (self.t - self.last_fire_t) >= DEBOUNCE_TIME

    if (self.enabled and long_enabled and v_ego >= MIN_ADVISORY_SPEED
        and sustained and no_recent_pedal and debounced):
      events_sp.add(EventNameSP.leadClosingAdvisory)
      self.last_fire_t = self.t

    self.frame += 1
