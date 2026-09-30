import io
import json
import os
import re
import tempfile
import wave
from dataclasses import dataclass
from pathlib import Path

SAMPLE_RATE = 48000
PEAK_TARGET_DBFS = -1.0
MIN_SECONDS = 0.1
HARD_MAX_SECONDS = 10.0
MAX_UPLOAD_BYTES = 2_000_000
MAX_LIBRARY_ITEMS = 50
MAX_NAME_LEN = 40

PKG_DIR = Path(__file__).resolve().parent
BASEDIR = PKG_DIR.parent.parent
BUILTIN_DIR = BASEDIR / "selfdrive" / "assets" / "sounds"
STOCK_DIR = PKG_DIR / "stock"
CATALOG_DIR = PKG_DIR / "catalog"
DATA_DIR = Path(os.environ.get("SOUNDMGR_DIR", "/data/soundmgr"))


@dataclass(frozen=True)
class Slot:
  id: str
  label: str
  help: str
  loop: bool
  max_seconds: float
  safety: str = ""


SLOTS = [
  Slot("prompt_distracted", "Pay Attention", "Plays on repeat while driver monitoring wants your eyes on the road.", True, 2.0,
       "Keep this loud and attention-grabbing. It is the warning that gets your eyes back on the road."),
  Slot("warning_soft", "Warning", "Plays on repeat for a warning that needs you to take over soon.", True, 2.0,
       "Keep this loud and attention-grabbing."),
  Slot("warning_immediate", "Disengage immediately", "Plays on repeat, getting louder, when you must take over right now.", True, 2.0,
       "This is the last-resort alarm. It should be jarring, not funny."),
  Slot("prompt", "Prompt", "Short chime for notices like a lane change or a speed limit change.", True, 2.0),
  Slot("engage", "Engage", "Plays when the system turns on.", False, 4.0),
  Slot("disengage", "Disengage", "Plays when the system turns off.", False, 4.0),
  Slot("refuse", "Can't engage", "Plays when the system refuses to engage.", False, 6.0),
  Slot("startup", "Startup", "Plays instead of the can't-engage sound in the first 2 minutes after the car starts.", False, 6.0),
]
SLOT_IDS = {s.id: s for s in SLOTS}

REF_RE = re.compile(r"^(catalog:[a-z0-9_]+/[A-Za-z0-9_]+|lib:[a-f0-9]{12}|builtin|stock)$")


class SoundError(ValueError):
  pass


def slot_for_filename(filename: str) -> str | None:
  stem = filename[:-4] if filename.endswith(".wav") else filename
  stem = stem.removesuffix("_tizi")
  return stem if stem in SLOT_IDS else None


def read_wav_bytes(data: bytes):
  """Validate a 48 kHz mono 16-bit PCM WAV; returns (int16 numpy array)."""
  import numpy as np
  try:
    with wave.open(io.BytesIO(data), "rb") as w:
      if w.getnchannels() != 1 or w.getsampwidth() != 2 or w.getframerate() != SAMPLE_RATE:
        raise SoundError("Sound must be a mono, 16-bit, 48 kHz WAV.")
      n = w.getnframes()
      if n > HARD_MAX_SECONDS * SAMPLE_RATE:
        raise SoundError(f"Sound is longer than {HARD_MAX_SECONDS:.0f} seconds.")
      return np.frombuffer(w.readframes(n), dtype=np.int16)
  except (wave.Error, EOFError) as e:
    raise SoundError("That is not a valid WAV file.") from e


def peak_normalize(samples):
  import numpy as np
  peak = int(np.max(np.abs(samples.astype(np.int32)))) if len(samples) else 0
  if peak == 0:
    raise SoundError("That sound is silent.")
  target = (2 ** 15 - 1) * (10 ** (PEAK_TARGET_DBFS / 20))
  return np.clip(np.round(samples.astype(np.float64) * (target / peak)), -32768, 32767).astype(np.int16)


def wav_bytes(samples) -> bytes:
  buf = io.BytesIO()
  with wave.open(buf, "wb") as w:
    w.setnchannels(1)
    w.setsampwidth(2)
    w.setframerate(SAMPLE_RATE)
    w.writeframes(samples.tobytes())
  return buf.getvalue()


def describe_file(path: Path, bars: int = 48):
  import numpy as np
  samples = read_wav_bytes(path.read_bytes())
  a = np.abs(samples.astype(np.int32))
  step = max(1, len(a) // bars)
  pk = [int(a[i * step:(i + 1) * step].max()) if len(a[i * step:(i + 1) * step]) else 0 for i in range(bars)]
  m = max(pk) or 1
  return round(len(samples) / SAMPLE_RATE, 2), [round(p / m, 2) for p in pk]


def atomic_write(path: Path, data: bytes):
  path.parent.mkdir(parents=True, exist_ok=True)
  fd, tmp = tempfile.mkstemp(dir=path.parent, prefix=".tmp-")
  try:
    with os.fdopen(fd, "wb") as f:
      f.write(data)
      f.flush()
      os.fsync(f.fileno())
    os.replace(tmp, path)
  except BaseException:
    if os.path.exists(tmp):
      os.unlink(tmp)
    raise


class Store:
  """Persistent state under DATA_DIR: config.json, library/<id>.wav, library/index.json."""

  def __init__(self, root: Path | None = None):
    self.root = Path(root) if root else DATA_DIR
    self.config_path = self.root / "config.json"
    self.library_dir = self.root / "library"
    self.library_index_path = self.library_dir / "index.json"

  def load_config(self) -> dict:
    try:
      cfg = json.loads(self.config_path.read_text())
      if not isinstance(cfg, dict):
        raise ValueError
    except (OSError, ValueError):
      cfg = {}
    slots = cfg.get("slots", {})
    return {"version": 1, "enabled": bool(cfg.get("enabled", True)),
            "slots": {k: v for k, v in slots.items() if k in SLOT_IDS and isinstance(v, str) and REF_RE.match(v)}}

  def save_config(self, cfg: dict):
    atomic_write(self.config_path, json.dumps(cfg, indent=1).encode())

  def load_library(self) -> dict:
    try:
      idx = json.loads(self.library_index_path.read_text())
      return idx if isinstance(idx, dict) else {}
    except (OSError, ValueError):
      return {}

  def save_library(self, idx: dict):
    atomic_write(self.library_index_path, json.dumps(idx, indent=1).encode())

  def lib_path(self, lib_id: str) -> Path:
    return self.library_dir / f"{lib_id}.wav"

  def resolve_ref(self, ref: str, slot: str | None = None) -> Path | None:
    if not isinstance(ref, str) or not REF_RE.match(ref):
      return None
    if ref.startswith("catalog:"):
      pack, stem = ref[len("catalog:"):].split("/")
      p = CATALOG_DIR / pack / f"{stem}.wav"
    elif ref.startswith("lib:"):
      p = self.lib_path(ref[4:])
    elif ref == "builtin" and slot:
      p = BUILTIN_DIR / f"{slot}.wav"
    elif ref == "stock" and slot:
      p = STOCK_DIR / f"{'refuse' if slot == 'startup' else slot}.wav"
    else:
      return None
    return p if p.is_file() else None


def get_override_path(filename: str, store: Store | None = None) -> str | None:
  """Used by soundd at load time. Must never raise; any problem means 'use the built-in sound'."""
  try:
    slot = slot_for_filename(filename)
    if slot is None:
      return None
    store = store or Store()
    cfg = store.load_config()
    if not cfg["enabled"] or slot not in cfg["slots"]:
      return None
    path = store.resolve_ref(cfg["slots"][slot], slot)
    if path is None:
      return None
    if path.stat().st_size > MAX_UPLOAD_BYTES * 5:
      return None
    samples = read_wav_bytes(path.read_bytes())
    if len(samples) > SLOT_IDS[slot].max_seconds * SAMPLE_RATE or len(samples) < MIN_SECONDS * SAMPLE_RATE:
      return None
    return str(path)
  except Exception:
    return None
