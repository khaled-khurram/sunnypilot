"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pyray as rl

from openpilot.selfdrive.ui.mici.onroad.hud_renderer import HudRenderer
from openpilot.selfdrive.ui.sunnypilot.onroad.blind_spot_indicators import BlindSpotIndicators
from openpilot.selfdrive.ui.sunnypilot.onroad.speed_limit import SpeedLimitRenderer
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached

# Plain number only - no sign background/border/label. Same 60x60 footprint and x-span (16-76)
# as the driver-camera icon directly above it, and same horizontal center (x=46) as the steering
# wheel icon below - all three share one column. Vertically centered in the gap between them
# (camera bottom ~y70, wheel top ~y176).
GAP_RECT = rl.Rectangle(16, 93, 60, 60)
NUMBER_FONT_SIZE = 40
ORANGE = rl.Color(255, 165, 0, 229)  # 255 * 0.9, alpha baked in since we don't animate opacity


class MiciSpeedLimitRenderer(SpeedLimitRenderer):
  """Reuses SpeedLimitRenderer's update()/data-fetch logic, replaces its sign-drawing _render()
  entirely with a single plain number - no MUTCD/Vienna sign, no offset badge, no ahead pill,
  no assist arrows (Assist mode isn't available on this car anyway)."""

  def __init__(self):
    super().__init__()
    self._font = gui_app.font(FontWeight.BOLD)
    self._hidden = False

  def set_hidden(self, hidden: bool) -> None:
    # Hidden whenever the MAX badge is showing in the same top-left area, so they never overlap -
    # same mechanism already used to hide the driver-camera icon in the same situation.
    self._hidden = hidden

  def _render(self, rect: rl.Rectangle) -> None:
    if self._hidden or not (self.speed_limit_valid or self.speed_limit_last_valid):
      return

    text = str(round(self.speed_limit_last))
    size = measure_text_cached(self._font, text, NUMBER_FONT_SIZE)
    pos = rl.Vector2(
      rect.x + GAP_RECT.x + (GAP_RECT.width - size.x) / 2,
      rect.y + GAP_RECT.y + (GAP_RECT.height - size.y) / 2,
    )
    rl.draw_text_ex(self._font, text, pos, NUMBER_FONT_SIZE, 0, ORANGE)


class HudRendererSP(HudRenderer):
  def __init__(self):
    super().__init__()
    self.blind_spot_indicators = BlindSpotIndicators()
    self.speed_limit_renderer = MiciSpeedLimitRenderer()

  def _update_state(self) -> None:
    super()._update_state()
    self.blind_spot_indicators.update()
    self.speed_limit_renderer.update()

  def _render(self, rect: rl.Rectangle) -> None:
    super()._render(rect)
    self.blind_spot_indicators.render(rect)
    self.speed_limit_renderer.set_hidden(self.drawing_top_icons())
    self.speed_limit_renderer.render(rect)

  def _has_blind_spot_detected(self) -> bool:

    return self.blind_spot_indicators.detected
