import json
import time

import pyray as rl
from openpilot.system.ui.widgets import Widget
from openpilot.selfdrive.ui.mici.onroad.confidence_ball import draw_circle_gradient, CONFIDENCE_BALL_RADIUS

# Duplicated from phase3_shared.py (plannerd-side control code, not imported here on
# purpose - the UI shouldn't depend on control code even though nothing mechanically
# prevents it). Same path constant, same defensive read/staleness pattern.
UI_STATUS_FILE = "/data/phase3_ui_status.json"
UI_STATUS_STALENESS_S = 2.0

DOT_SIZE_RATIO = 0.7  # dot diameter as a fraction of the confidence ball's diameter
DOT_RADIUS = round(CONFIDENCE_BALL_RADIUS * DOT_SIZE_RATIO)
DOT_COLOR = rl.Color(255, 255, 255, 255)  # white - matches confidence ball's own
                                            # OVERRIDE-state color; identity is carried
                                            # by fixed left-to-right position, not hue


def _read_ui_status_for_ui() -> dict | None:
  try:
    with open(UI_STATUS_FILE) as f:
      status = json.loads(f.read())
    if time.time() - status["t"] > UI_STATUS_STALENESS_S:
      return None
  except (FileNotFoundError, ValueError, KeyError, OSError):
    return None
  return status


def _render_dot(cx: float, cy: float, state: str) -> None:
  if state == "idle":
    draw_circle_gradient(cx, cy, DOT_RADIUS, rl.BLANK, rl.BLANK)     # ring only, no fill
  elif state == "active":
    draw_circle_gradient(cx, cy, DOT_RADIUS, DOT_COLOR, DOT_COLOR)   # flat solid fill
  # "off": not called at all


class DotsRenderer(Widget):
  """Three small peripheral status dots (curve, lead, SLF - fixed left-to-right order)
  between the DM icon and the confidence ball. off/idle/active only - off covers both
  "not armed" and "latched off" (the one-shot alert already carries the why, see
  lead_closing_test_guidance_helper.py). Currently-on dots auto-center themselves
  across the DM<->ball span; off dots occupy no space."""

  POLL_INTERVAL_S = 0.15  # UI render loop runs 20-60fps; a stat+open+json.loads every
                            # frame is unnecessary
  LEFT_MARGIN = 20   # clear of whichever is wider: DM icon (60px) or the MAX badge (162px)
  RIGHT_MARGIN = 20  # clear of the confidence ball + its own black contrast ring

  def __init__(self):
    super().__init__()
    self._last_poll = 0.0
    self._status: dict | None = None

  def _update_state(self):
    now = rl.get_time()
    if now - self._last_poll >= self.POLL_INTERVAL_S:
      self._last_poll = now
      self._status = _read_ui_status_for_ui()

  def _render(self, rect: rl.Rectangle):
    if not self._status:
      return  # missing/stale plannerd -> render nothing, same as "all off"

    # A shared override-latch trip suppresses every currently-armed feature at once
    # (see Phase3OverrideLatch's own docstring) - so while overridden, every dot reads
    # as off regardless of its individual armed/active value, not just whichever
    # feature's own "active" flag happens to be stale from the frame the latch tripped.
    if self._status.get("overridden"):
      return

    states = [
      self._dot_state(self._status["curve"]),
      self._dot_state(self._status["lead"]),
      self._dot_state(self._status["slf"]),
    ]
    on_indices = [i for i, s in enumerate(states) if s != "off"]
    if not on_indices:
      return

    left_bound = rect.x + max(76, 162) + self.LEFT_MARGIN
    ball_center_x = rect.x + rect.width - CONFIDENCE_BALL_RADIUS
    right_bound = ball_center_x - CONFIDENCE_BALL_RADIUS * 1.5 - self.RIGHT_MARGIN
    span = right_bound - left_bound
    y = rect.y + 10 + 30  # DM icon's own vertical center (rect.y+10, 60px tall) - fixed,
                            # independent of the confidence ball's animating y

    k = len(on_indices)
    for slot, i in enumerate(on_indices):
      x = left_bound + (slot + 1) / (k + 1) * span
      _render_dot(x, y, states[i])

  @staticmethod
  def _dot_state(feature: dict) -> str:
    if not feature["armed"]:
      return "off"
    return "active" if feature["active"] else "idle"
