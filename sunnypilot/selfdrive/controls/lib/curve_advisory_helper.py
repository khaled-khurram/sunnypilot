"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from cereal import custom

from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP

MapState = custom.LongitudinalPlanSP.SmartCruiseControl.MapState
EventNameSP = custom.OnroadEventSP.EventName


class CurveAdvisoryHelper:
  """
  Phase 1 curve-speed advisory: edge-triggers a driver-facing alert when
  SmartCruiseControlMap (MTSC) locks onto an upcoming curve, reusing the
  speed-target math it already computes. Read-only, no actuation.
  """

  def __init__(self):
    self._params = Params()
    self.frame = -1
    self.enabled = self._params.get_bool("CurveSpeedAdvisory")
    self.was_active = False

  def _read_params(self) -> None:
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self.enabled = self._params.get_bool("CurveSpeedAdvisory")

  def update(self, map_state: int, long_enabled: bool, events_sp: EventsSP) -> None:
    self._read_params()

    is_active = map_state == MapState.turning

    # Rising edge only - fire once per curve, not every frame while active.
    if self.enabled and long_enabled and is_active and not self.was_active:
      events_sp.add(EventNameSP.curveSpeedAdvisory)

    self.was_active = is_active
    self.frame += 1
