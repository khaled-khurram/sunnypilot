"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import time
from enum import StrEnum
from opendbc.car import Bus, structs

from opendbc.car.subaru.values import SubaruFlags
from opendbc.sunnypilot.mads_base import MadsCarStateBase
from opendbc.can.parser import CANParser

ButtonType = structs.CarState.ButtonEvent.Type

# The physical MAIN/power ACC button (CruiseControl 0x144 byte0 bit2, DBC "OnOffButton") mechanically
# double-pulses on every real press (~100ms between the two rising edges, empirically confirmed 8/8
# times) -- much faster than the SET/RES rocker's own occasional bounce. Feeding raw edges straight
# into MADS's toggle would flip it on-off-on within ~100ms of a real press, netting out to no visible
# change. This refractory window coalesces a bounce burst into one logical press.
MAIN_BUTTON_DEBOUNCE_S = 0.25


class MadsCarState(MadsCarStateBase):
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    super().__init__(CP, CP_SP)
    self.main_button_prev = 0
    self.main_button_last_press_t = 0.0

  @staticmethod
  def create_lkas_button_events(cur_btn: int, prev_btn: int,
                                buttons_dict: dict[int, structs.CarState.ButtonEvent.Type]) -> list[structs.CarState.ButtonEvent]:
    events: list[structs.CarState.ButtonEvent] = []

    if cur_btn == prev_btn:
      return events

    state_changes = [
      {"pressed": prev_btn != cur_btn and cur_btn != 2 and not (prev_btn == 2 and cur_btn == 1)},
      {"pressed": prev_btn != cur_btn and cur_btn == 2 and cur_btn != 1},
    ]

    for change in state_changes:
      if change["pressed"]:
        events.append(structs.CarState.ButtonEvent(pressed=change["pressed"],
                                                   type=buttons_dict.get(cur_btn, ButtonType.unknown)))
    return events

  def update_mads(self, ret: structs.CarState, can_parsers: dict[StrEnum, CANParser]) -> None:
    if self.CP.flags & SubaruFlags.PREGLOBAL:
      # Preglobal has no LKAS_Dash_State-equivalent signal. Use the physical MAIN/power ACC button
      # (debounced, see MAIN_BUTTON_DEBOUNCE_S) as MADS's on/off toggle instead.
      cp = can_parsers[Bus.pt]
      main_button = int(cp.vl["CruiseControl"]["OnOffButton"])

      ret.buttonEvents = []
      if main_button == 1 and self.main_button_prev == 0:
        now = time.monotonic()
        if now - self.main_button_last_press_t > MAIN_BUTTON_DEBOUNCE_S:
          self.main_button_last_press_t = now
          ret.buttonEvents = [structs.CarState.ButtonEvent(pressed=True, type=ButtonType.lkas)]
      self.main_button_prev = main_button
    else:
      cp_cam = can_parsers[Bus.cam]
      self.prev_lkas_button = self.lkas_button
      self.lkas_button = cp_cam.vl["ES_LKAS_State"]["LKAS_Dash_State"]
      ret.buttonEvents = self.create_lkas_button_events(self.lkas_button, self.prev_lkas_button, {1: ButtonType.lkas})
