from openpilot.sunnypilot.soundmgr.refuse_gate import PLAY, SILENT, STARTUP, STARTUP_WINDOW, DELIBERATE_WINDOW, RefuseGate


def run(g, alert_type, mads_paused, t, frames=5):
  """One refuse alert lasting `frames` updates, followed by a quiet frame. Returns the decisions."""
  out = [g.decide(True, alert_type, mads_paused, t + i * 0.01) for i in range(frames)]
  g.decide(False, "", mads_paused, t + frames * 0.01)
  return out


def test_gear_not_d_at_startup_plays_startup_sound():
  g = RefuseGate(start_time=100.0)
  assert set(run(g, "wrongGear/noEntry", False, 106.6)) == {STARTUP}


def test_gear_not_d_after_window_is_normal_refuse():
  g = RefuseGate(start_time=100.0)
  assert set(run(g, "wrongGear/noEntry", False, 100.0 + STARTUP_WINDOW + 1)) == {PLAY}


def test_other_refuse_during_startup_window_stays_refuse():
  g = RefuseGate(start_time=100.0)
  assert set(run(g, "seatbeltNotLatched/noEntry", False, 105.0)) == {PLAY}


def test_automatic_resume_plays_once_per_blocker_then_silent():
  g = RefuseGate(start_time=0.0)
  assert set(run(g, "seatbeltNotLatched/noEntry", True, 500.0)) == {PLAY}
  assert set(run(g, "seatbeltNotLatched/noEntry", True, 512.0)) == {SILENT}
  assert set(run(g, "seatbeltNotLatched/noEntry", True, 530.0)) == {SILENT}
  assert set(run(g, "doorOpen/noEntry", True, 540.0)) == {PLAY}
  assert set(run(g, "doorOpen/noEntry", True, 552.0)) == {SILENT}


def test_decision_is_stable_within_one_alert():
  g = RefuseGate(start_time=0.0)
  run(g, "seatbeltNotLatched/noEntry", True, 500.0)
  assert set(run(g, "seatbeltNotLatched/noEntry", True, 512.0, frames=300)) == {SILENT}
  g2 = RefuseGate(start_time=0.0)
  assert set(run(g2, "seatbeltNotLatched/noEntry", True, 500.0, frames=300)) == {PLAY}


def test_deliberate_cruise_on_always_plays():
  g = RefuseGate(start_time=0.0)
  run(g, "seatbeltNotLatched/noEntry", True, 500.0)
  g.update_cruise(False, 599.0)
  g.update_cruise(True, 600.0)
  assert set(run(g, "seatbeltNotLatched/noEntry", True, 600.5)) == {PLAY}
  assert set(run(g, "seatbeltNotLatched/noEntry", True, 600.0 + DELIBERATE_WINDOW + 5)) == {SILENT}


def test_cruise_already_on_at_start_is_not_a_deliberate_press():
  g = RefuseGate(start_time=0.0)
  g.update_cruise(True, 1.0)
  run(g, "seatbeltNotLatched/noEntry", True, 500.0)
  assert set(run(g, "seatbeltNotLatched/noEntry", True, 512.0)) == {SILENT}


def test_refuse_when_mads_not_paused_is_normal():
  g = RefuseGate(start_time=0.0)
  for t in (500.0, 512.0, 524.0):
    assert set(run(g, "seatbeltNotLatched/noEntry", False, t)) == {PLAY}


def test_non_refuse_frames_are_play():
  g = RefuseGate(start_time=0.0)
  assert g.decide(False, "", True, 5.0) == PLAY
