import wave
from pathlib import Path

import numpy as np
import pytest

from openpilot.sunnypilot.soundmgr import leveling as L

ROOT = Path(__file__).resolve().parents[3]
SLOTS = sorted(L.SLOTS)


def load(path) -> np.ndarray:
  with wave.open(str(path), "rb") as w:
    return np.frombuffer(w.readframes(w.getnframes()), dtype=np.int16).astype(np.float32) / 32768


def tone(freq, seconds=0.5, amp=0.5):
  t = np.arange(int(seconds * L.SAMPLE_RATE)) / L.SAMPLE_RATE
  return (amp * np.sin(2 * np.pi * freq * t)).astype(np.float32)


@pytest.mark.parametrize("slot", SLOTS)
def test_shipped_sounds_match_stock_loudness(slot):
  x = load(ROOT / "selfdrive/assets/sounds" / f"{slot}.wav")
  y = L.level_like_stock(x, slot)
  assert abs(L.a_weighted_level_db(y.astype(np.float64)) - L.stock_level_db(slot)) < 0.5
  assert np.isfinite(y).all() and np.abs(y).max() <= 1.0
  assert len(y) == len(x) and y.dtype == np.float32


@pytest.mark.parametrize("slot", [s for s in SLOTS if s != "startup"])
def test_stock_sounds_are_left_alone(slot):
  x = load(L.STOCK_DIR / f"{slot}.wav")
  y = L.level_like_stock(x, slot)
  assert abs(L.a_weighted_level_db(y.astype(np.float64)) - L.stock_level_db(slot)) < 0.3


def test_quiet_sound_gain_is_capped():
  x = tone(1500, amp=0.002)
  y = L.level_like_stock(x, "engage")
  gain = 20 * np.log10(np.abs(y).max() / np.abs(x).max())
  assert gain <= L.MAX_GAIN_DB + 0.5


def test_loud_sound_is_not_clipped_and_is_reduced_at_most_6db():
  x = tone(1500, amp=1.0)
  y = L.level_like_stock(x, "engage")
  assert np.abs(y).max() <= 1.0
  assert 20 * np.log10(np.abs(y).max() / np.abs(x).max()) >= L.MIN_GAIN_DB - 0.5


def test_sub_speaker_bass_is_removed():
  x = tone(100)
  assert np.sqrt((L.highpass(x.astype(np.float64)) ** 2).mean()) < 0.01 * np.sqrt((x ** 2).mean())
  x = tone(2000)
  assert np.sqrt((L.highpass(x.astype(np.float64)) ** 2).mean()) > 0.98 * np.sqrt((x ** 2).mean())


def test_untouched_cases():
  silent = np.zeros(4800, dtype=np.float32)
  assert L.level_like_stock(silent, "engage") is silent
  x = tone(1000)
  assert L.level_like_stock(x, "prompt_single_low") is x
  empty = np.zeros(0, dtype=np.float32)
  assert L.level_like_stock(empty, "engage") is empty


def test_soft_limit_is_monotonic_and_bounded():
  y = np.linspace(-5, 5, 1001)
  out = L.soft_limit(y)
  assert np.all(np.diff(out) >= 0) and np.abs(out).max() <= 1.0
  assert np.allclose(out[np.abs(y) <= L.LIMIT_THRESHOLD], y[np.abs(y) <= L.LIMIT_THRESHOLD])
