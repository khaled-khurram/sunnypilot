"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pyray as rl

from openpilot.selfdrive.ui.mici.onroad.hud_renderer import HudRenderer
from openpilot.selfdrive.ui.sunnypilot.onroad.blind_spot_indicators import BlindSpotIndicators
from openpilot.selfdrive.ui.sunnypilot.onroad.speed_limit import SpeedLimitRenderer

# speed_limit.py sizes/positions its sign from UI_CONFIG (tuned for the larger tici/desktop
# screen) and only uses the passed rect's x/y as an anchor. On mici's much shorter 240px-tall
# canvas the unmodified anchor clips ~15px off the bottom of the sign, so nudge it up here
# rather than touching the shared speed_limit.py sizing used by every other screen.
MICI_SPEED_LIMIT_Y_NUDGE = -15


class HudRendererSP(HudRenderer):
  def __init__(self):
    super().__init__()
    self.blind_spot_indicators = BlindSpotIndicators()
    self.speed_limit_renderer = SpeedLimitRenderer()

  def _update_state(self) -> None:
    super()._update_state()
    self.blind_spot_indicators.update()
    self.speed_limit_renderer.update()

  def _render(self, rect: rl.Rectangle) -> None:
    super()._render(rect)
    self.blind_spot_indicators.render(rect)
    self.speed_limit_renderer.render(rl.Rectangle(rect.x, rect.y + MICI_SPEED_LIMIT_Y_NUDGE, rect.width, rect.height))

  def _has_blind_spot_detected(self) -> bool:

    return self.blind_spot_indicators.detected
