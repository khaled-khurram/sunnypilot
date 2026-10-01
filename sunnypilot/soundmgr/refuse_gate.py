STARTUP_WINDOW = 120.0  # seconds after soundd starts in which a wrong-gear refuse plays the startup sound
DELIBERATE_WINDOW = 2.0  # seconds after the driver turns cruise on in which a refuse always plays

PLAY = "play"
STARTUP = "startup"
SILENT = "silent"


class RefuseGate:
  """Decides what a 'refuse' alert should sound like.

  - wrongGear within the startup window: the startup sound.
  - Automatic MADS resume attempts (MADS paused, no recent cruise-on): once per blocker, then silent.
  - Everything else, including the driver turning cruise on: the normal refuse sound.
  The decision is made once, when the refuse alert starts, so a sound is never cut off mid-play.
  """

  def __init__(self, start_time: float):
    self.start_time = start_time
    self.cruise_prev: bool | None = None
    self.deliberate_until = float("-inf")
    self.auto_played: set[str] = set()
    self.prev_refuse = False
    self.decision = PLAY

  def update_cruise(self, cruise_enabled: bool, now: float):
    if self.cruise_prev is False and cruise_enabled:
      self.deliberate_until = now + DELIBERATE_WINDOW
    self.cruise_prev = cruise_enabled

  def decide(self, is_refuse: bool, alert_type: str, mads_paused: bool, now: float) -> str:
    if is_refuse and not self.prev_refuse:
      blocker = alert_type.split("/")[0]
      if blocker == "wrongGear" and now - self.start_time < STARTUP_WINDOW:
        self.decision = STARTUP
      elif mads_paused and now > self.deliberate_until:
        if blocker in self.auto_played:
          self.decision = SILENT
        else:
          self.auto_played.add(blocker)
          self.decision = PLAY
      else:
        self.decision = PLAY
    self.prev_refuse = is_refuse
    return self.decision if is_refuse else PLAY
