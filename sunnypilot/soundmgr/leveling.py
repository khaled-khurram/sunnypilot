import numpy as np
from pathlib import Path
import wave

SAMPLE_RATE = 48000
STOCK_DIR = Path(__file__).resolve().parent / "stock"
HIGHPASS_HZ = 350.0  # the speaker can't reproduce much below this and cabin noise masks it
HIGHPASS_ORDER = 4
MIN_GAIN_DB = -6.0
MAX_GAIN_DB = 12.0
LIMIT_THRESHOLD = 0.7
GATE = 0.05
SLOTS = {"prompt_distracted", "warning_soft", "warning_immediate", "prompt", "engage", "disengage", "refuse", "startup"}

_ref_cache: dict[str, float] = {}


def _a_weight(f: np.ndarray) -> np.ndarray:
  f2 = np.maximum(f, 1e-3) ** 2
  ra = (12194 ** 2 * f2 ** 2) / ((f2 + 20.6 ** 2) * np.sqrt((f2 + 107.7 ** 2) * (f2 + 737.9 ** 2)) * (f2 + 12194 ** 2))
  ref = (12194 ** 2 * 1000 ** 4) / ((1000 ** 2 + 20.6 ** 2) * np.sqrt((1000 ** 2 + 107.7 ** 2) * (1000 ** 2 + 737.9 ** 2)) * (1000 ** 2 + 12194 ** 2))
  return ra / ref


def a_weighted_level_db(x: np.ndarray) -> float:
  """RMS level (dB) of the A-weighted signal, measured over the audible parts only."""
  n = len(x)
  weighted = np.fft.irfft(np.fft.rfft(x) * _a_weight(np.fft.rfftfreq(n, 1 / SAMPLE_RATE)), n)
  peak = float(np.max(np.abs(weighted)))
  if peak <= 0:
    return -120.0
  active = weighted[np.abs(weighted) >= GATE * peak]
  return float(10 * np.log10(np.mean(active ** 2)))


def highpass(x: np.ndarray) -> np.ndarray:
  n = len(x)
  f = np.fft.rfftfreq(n, 1 / SAMPLE_RATE)
  h = 1 / np.sqrt(1 + (HIGHPASS_HZ / np.maximum(f, 1e-3)) ** (2 * HIGHPASS_ORDER))
  return np.fft.irfft(np.fft.rfft(x) * h, n)


def soft_limit(y: np.ndarray) -> np.ndarray:
  a = np.abs(y)
  room = 1.0 - LIMIT_THRESHOLD
  over = LIMIT_THRESHOLD + room * np.tanh((a - LIMIT_THRESHOLD) / room)
  return np.sign(y) * np.where(a > LIMIT_THRESHOLD, over, a)


def _read_stock(slot: str) -> np.ndarray:
  name = "refuse" if slot == "startup" else slot
  with wave.open(str(STOCK_DIR / f"{name}.wav"), "rb") as w:
    return np.frombuffer(w.readframes(w.getnframes()), dtype=np.int16).astype(np.float64) / 32768


def stock_level_db(slot: str) -> float:
  if slot not in _ref_cache:
    _ref_cache[slot] = a_weighted_level_db(_read_stock(slot))
  return _ref_cache[slot]


def level_like_stock(samples: np.ndarray, slot: str) -> np.ndarray:
  """Make a sound as loud to the ear as the stock sound for its slot. Returns float32 in [-1, 1]."""
  if slot not in SLOTS or len(samples) == 0:
    return samples
  target = stock_level_db(slot)
  y = highpass(samples.astype(np.float64))
  total_db = 0.0
  for _ in range(3):
    current = a_weighted_level_db(y)
    if current <= -100:
      return samples
    step = float(np.clip(target - current, MIN_GAIN_DB - total_db, MAX_GAIN_DB - total_db))
    if abs(step) < 0.25:
      break
    total_db += step
    y = soft_limit(highpass(samples.astype(np.float64)) * 10 ** (total_db / 20))
  return np.clip(y, -1.0, 1.0).astype(np.float32)
