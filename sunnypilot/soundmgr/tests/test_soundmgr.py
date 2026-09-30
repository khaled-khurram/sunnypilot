import io
import json
import wave

import numpy as np
import pytest
from aiohttp import FormData
from aiohttp.test_utils import TestClient, TestServer

from openpilot.sunnypilot.soundmgr import common as c
from openpilot.sunnypilot.soundmgr import server

TOKEN = "t" * 32
AUTH = {"Authorization": f"Bearer {TOKEN}"}


def make_wav(seconds=0.5, amp=8000, rate=c.SAMPLE_RATE, channels=1, width=2, freq=440.0) -> bytes:
  t = np.arange(int(seconds * rate)) / rate
  s = (amp * np.sin(2 * np.pi * freq * t)).astype(np.int16)
  if channels == 2:
    s = np.repeat(s, 2)
  buf = io.BytesIO()
  with wave.open(buf, "wb") as w:
    w.setnchannels(channels)
    w.setsampwidth(width)
    w.setframerate(rate)
    w.writeframes(s.tobytes() if width == 2 else (s >> 8).astype(np.int8).tobytes())
  return buf.getvalue()


def form(data: bytes, name="My goat") -> FormData:
  f = FormData()
  f.add_field("file", data, filename="x.wav", content_type="audio/wav")
  f.add_field("name", name)
  return f


@pytest.fixture
def store(tmp_path):
  return c.Store(tmp_path)


@pytest.fixture
def state():
  return {"onroad": False}


@pytest.fixture
async def client(store, state):
  app = server.make_app(store=store, token=TOKEN, onroad=lambda: state["onroad"])
  async with TestClient(TestServer(app)) as cl:
    yield cl


async def test_health_needs_no_auth(client):
  r = await client.get("/api/health")
  assert r.status == 200


@pytest.mark.parametrize("headers", [{}, {"Authorization": "Bearer nope"}, {"Authorization": TOKEN}])
async def test_api_requires_token(client, headers):
  assert (await client.get("/api/state", headers=headers)).status == 401
  assert (await client.put("/api/enabled", json={"enabled": False}, headers=headers)).status == 401


async def test_state_shape(client):
  st = await (await client.get("/api/state", headers=AUTH)).json()
  assert [s["id"] for s in st["slots"]] == [s.id for s in c.SLOTS]
  assert st["enabled"] is True and st["onroad"] is False
  assert all(s["current"]["source"] == "builtin" and s["current"]["ref"] == f"builtin:{s['id']}" for s in st["slots"])
  assert all(len(s["current"]["peaks"]) == 48 and s["current"]["duration"] > 0 for s in st["slots"])
  assert sum(len(p["items"]) for p in st["catalog"]) == 61
  assert st["library"] == []


async def test_builtin_files_fit_their_slots():
  for s in c.SLOTS:
    d, _ = c.describe_file(c.BUILTIN_DIR / f"{s.id}.wav")
    assert d <= s.max_seconds, s.id


async def test_every_catalog_file_exists_and_is_valid_format():
  for pack in json.loads((c.CATALOG_DIR / "index.json").read_text()):
    for it in pack["items"]:
      p = c.Store().resolve_ref(it["ref"])
      assert p is not None, it["ref"]
      c.read_wav_bytes(p.read_bytes())


async def test_audio_endpoint(client):
  r = await client.get("/api/audio?ref=catalog:tesla/prompt_distracted", headers=AUTH)
  assert r.status == 200 and r.content_type == "audio/wav"
  c.read_wav_bytes(await r.read())
  assert (await client.get("/api/audio?ref=builtin:prompt_distracted", headers=AUTH)).status == 200
  assert (await client.get("/api/audio?ref=stock:startup", headers=AUTH)).status == 200
  for bad in ("catalog:../../etc/passwd", "lib:zzzzzzzzzzzz", "builtin:evil", "nonsense", ""):
    assert (await client.get(f"/api/audio?ref={bad}", headers=AUTH)).status == 404


async def test_upload_normalizes_and_lists(client, store):
  r = await client.post("/api/upload", data=form(make_wav(amp=500), "  quiet   goat \n"), headers=AUTH)
  assert r.status == 200
  item = await r.json()
  assert item["name"] == "quiet goat" and item["ref"] == f"lib:{item['id']}"
  samples = c.read_wav_bytes(store.lib_path(item["id"]).read_bytes())
  assert abs(int(np.abs(samples.astype(np.int32)).max()) - 29204) < 5  # -1 dBFS
  st = await (await client.get("/api/state", headers=AUTH)).json()
  assert [m["id"] for m in st["library"]] == [item["id"]]


@pytest.mark.parametrize("wav,frag", [
  (make_wav(rate=44100), "48 kHz"),
  (make_wav(channels=2), "mono"),
  (make_wav(amp=0), "silent"),
  (make_wav(seconds=0.02), "too short"),
  (make_wav(seconds=11), "10 seconds"),
  (b"RIFFnotawav", "valid WAV"),
])
async def test_upload_rejects_bad_files(client, store, wav, frag):
  r = await client.post("/api/upload", data=form(wav), headers=AUTH)
  assert r.status == 400 and frag in (await r.json())["error"]
  assert store.load_library() == {}


async def test_upload_too_big(client):
  r = await client.post("/api/upload", data=form(b"\0" * (c.MAX_UPLOAD_BYTES + 200_000)), headers=AUTH)
  assert r.status in (400, 413)


async def test_assign_catalog_and_reset(client, store):
  r = await client.put("/api/slot/prompt_distracted", json={"ref": "catalog:tesla/prompt_distracted"}, headers=AUTH)
  assert r.status == 200
  cur = (await r.json())["current"]
  assert cur["source"] == "catalog" and cur["name"] == "Tesla Pay Attention"
  assert store.load_config()["slots"] == {"prompt_distracted": "catalog:tesla/prompt_distracted"}
  r = await client.put("/api/slot/prompt_distracted", json={"ref": "builtin"}, headers=AUTH)
  assert (await r.json())["current"]["source"] == "builtin"
  assert store.load_config()["slots"] == {}


async def test_assign_stock_names_it(client):
  r = await client.put("/api/slot/startup", json={"ref": "stock"}, headers=AUTH)
  cur = (await r.json())["current"]
  assert cur["source"] == "stock" and cur["ref"] == "stock:startup"
  r = await client.get(f"/api/audio?ref={cur['ref']}", headers=AUTH)
  assert r.status == 200
  r = await client.put("/api/slot/startup", json={"ref": "builtin:startup"}, headers=AUTH)
  assert (await r.json())["current"]["ref"] == "builtin:startup"
  r = await client.put("/api/slot/engage", json={"ref": "stock:engage"}, headers=AUTH)
  assert (await r.json())["current"]["source"] == "stock"
  r = await client.put("/api/slot/engage", json={"ref": "stock:disengage"}, headers=AUTH)
  assert r.status == 404


async def test_assign_too_long_for_slot_is_rejected(client, store):
  r = await client.put("/api/slot/prompt_distracted", json={"ref": "catalog:stalin/warning_soft"}, headers=AUTH)
  assert r.status == 400 and "at most" in (await r.json())["error"]
  assert store.load_config()["slots"] == {}


@pytest.mark.parametrize("slot,body", [("nope", {"ref": "builtin"}), ("engage", {"ref": "catalog:nope/nope"}),
                                        ("engage", {"ref": "../../etc/passwd"}), ("engage", {}), ("engage", None)])
async def test_assign_bad_input(client, slot, body):
  r = await client.put(f"/api/slot/{slot}", json=body, headers=AUTH)
  assert r.status in (400, 404)


async def test_delete_library_reverts_slots(client, store):
  item = await (await client.post("/api/upload", data=form(make_wav()), headers=AUTH)).json()
  await client.put("/api/slot/engage", json={"ref": item["ref"]}, headers=AUTH)
  assert store.load_config()["slots"]["engage"] == item["ref"]
  assert (await client.delete(f"/api/library/{item['id']}", headers=AUTH)).status == 200
  assert store.load_config()["slots"] == {} and not store.lib_path(item["id"]).exists()
  assert (await client.delete(f"/api/library/{item['id']}", headers=AUTH)).status == 404


async def test_enabled_toggle(client, store):
  r = await client.put("/api/enabled", json={"enabled": False}, headers=AUTH)
  assert (await r.json()) == {"enabled": False} and store.load_config()["enabled"] is False
  assert (await client.put("/api/enabled", json={"enabled": "yes"}, headers=AUTH)).status == 400


async def test_onroad_locks_edits_but_not_reads(client, state):
  state["onroad"] = True
  assert (await client.get("/api/state", headers=AUTH)).status == 200
  assert (await client.get("/api/audio?ref=builtin:engage", headers=AUTH)).status == 200
  for r in (await client.put("/api/enabled", json={"enabled": False}, headers=AUTH),
            await client.put("/api/slot/engage", json={"ref": "builtin"}, headers=AUTH),
            await client.post("/api/upload", data=form(make_wav()), headers=AUTH),
            await client.delete("/api/library/abcdefabcdef", headers=AUTH)):
    assert r.status == 423


async def test_static_serving_and_traversal(client):
  r = await client.get("/")
  assert r.status == 200 and r.content_type == "text/html"
  for bad in ("/../common.py", "/%2e%2e/common.py", "/fonts/../../common.py", "/server.py", "/fonts/%2e%2e%2fserver.py", "/config.json"):
    assert (await client.get(bad)).status == 404, bad


# ---- soundd override hook ----

def cfg(store, slots, enabled=True):
  store.save_config({"version": 1, "enabled": enabled, "slots": slots})


def test_override_none_without_config(store):
  assert c.get_override_path("engage.wav", store) is None


def test_override_returns_path_and_handles_tizi_names(store):
  cfg(store, {"engage": "catalog:tesla/engage"})
  assert c.get_override_path("engage.wav", store).endswith("tesla/engage.wav")
  assert c.get_override_path("engage_tizi.wav", store).endswith("tesla/engage.wav")
  assert c.get_override_path("disengage.wav", store) is None
  assert c.get_override_path("prompt_single_low.wav", store) is None


def test_override_disabled(store):
  cfg(store, {"engage": "catalog:tesla/engage"}, enabled=False)
  assert c.get_override_path("engage.wav", store) is None


def test_override_stock_startup_uses_stock_refuse(store):
  cfg(store, {"startup": "stock"})
  assert c.get_override_path("startup.wav", store).endswith("stock/refuse.wav")


def test_override_ignores_too_long_corrupt_missing_and_garbage(store):
  cfg(store, {"prompt_distracted": "catalog:stalin/warning_soft"})
  assert c.get_override_path("prompt_distracted.wav", store) is None
  cfg(store, {"engage": "lib:aaaaaaaaaaaa"})
  assert c.get_override_path("engage.wav", store) is None
  store.library_dir.mkdir(parents=True)
  store.lib_path("bbbbbbbbbbbb").write_bytes(b"not a wav")
  cfg(store, {"engage": "lib:bbbbbbbbbbbb"})
  assert c.get_override_path("engage.wav", store) is None
  store.lib_path("cccccccccccc").write_bytes(make_wav(rate=44100))
  cfg(store, {"engage": "lib:cccccccccccc"})
  assert c.get_override_path("engage.wav", store) is None
  store.config_path.write_text("{ this is not json")
  assert c.get_override_path("engage.wav", store) is None
  store.config_path.write_text(json.dumps({"enabled": True, "slots": {"engage": "../../etc/passwd", "nope": "builtin"}}))
  assert c.get_override_path("engage.wav", store) is None


def test_override_never_raises_on_broken_store():
  class Boom:
    def load_config(self):
      raise RuntimeError("disk on fire")
  assert c.get_override_path("engage.wav", Boom()) is None
