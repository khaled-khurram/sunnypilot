import hmac
import json
import logging
import os
import secrets
import uuid
from pathlib import Path

from aiohttp import web

from openpilot.sunnypilot.soundmgr import common as c

log = logging.getLogger("soundmgr")

PORT = int(os.environ.get("SOUNDMGR_PORT", "8090"))
WEB_DIR = c.PKG_DIR / "web"
MIME = {".html": "text/html", ".js": "text/javascript", ".css": "text/css", ".webmanifest": "application/manifest+json",
        ".svg": "image/svg+xml", ".png": "image/png", ".woff": "font/woff"}

STORE_KEY = web.AppKey("store", c.Store)
TOKEN_KEY = web.AppKey("token", str)
ONROAD_KEY = web.AppKey("onroad", object)
CATALOG_KEY = web.AppKey("catalog", list)
DESC_CACHE_KEY = web.AppKey("desc_cache", dict)


def params_onroad() -> bool:
  from openpilot.common.params import Params
  return Params().get_bool("IsOnroad")


def load_or_create_token(root: Path) -> str:
  p = root / "token"
  try:
    t = p.read_text().strip()
    if len(t) >= 20:
      return t
  except OSError:
    pass
  t = secrets.token_urlsafe(24)
  root.mkdir(parents=True, exist_ok=True)
  fd = os.open(p, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
  with os.fdopen(fd, "w") as f:
    f.write(t)
  return t


def err(status: int, msg: str) -> web.Response:
  return web.json_response({"error": msg}, status=status)


@web.middleware
async def guard(request: web.Request, handler):
  path = request.path
  if path.startswith("/api/") and path != "/api/health":
    supplied = request.headers.get("Authorization", "")
    supplied = supplied[7:] if supplied.startswith("Bearer ") else ""
    if not hmac.compare_digest(supplied.encode(), request.app[TOKEN_KEY].encode()):
      return err(401, "Wrong or missing access token.")
    if request.method not in ("GET", "HEAD") and request.app[ONROAD_KEY]():
      return err(423, "The car is on. Sounds can only be changed while parked.")
  resp = await handler(request)
  resp.headers["Cache-Control"] = "no-store" if path.startswith("/api/") else "no-cache"
  resp.headers["X-Content-Type-Options"] = "nosniff"
  return resp


def describe(app: web.Application, path: Path):
  key = (str(path), path.stat().st_mtime_ns)
  cache = app[DESC_CACHE_KEY]
  if key not in cache:
    cache[key] = c.describe_file(path)
  return cache[key]


def catalog_name(app, ref: str):
  for pack in app[CATALOG_KEY]:
    for it in pack["items"]:
      if it["ref"] == ref:
        return it["name"]
  return ref.split("/")[-1].replace("_", " ").title()


def slot_json(app: web.Application, slot: c.Slot, cfg: dict) -> dict:
  store = app[STORE_KEY]
  ref = cfg["slots"].get(slot.id)
  source, name, path = "builtin", f"Built-in {slot.label}", store.resolve_ref("builtin", slot.id)
  if ref:
    p = store.resolve_ref(ref, slot.id)
    if p is not None:
      path = p
      if ref.startswith("catalog:"):
        source, name = "catalog", catalog_name(app, ref)
      elif ref.startswith("lib:"):
        source = "upload"
        name = store.load_library().get(ref[4:], {}).get("name", "Your upload")
      elif ref == "stock":
        source, name = "stock", f"openpilot stock {slot.label}"
    else:
      ref = None
  duration, peaks = describe(app, path)
  return {"id": slot.id, "label": slot.label, "help": slot.help, "loop": slot.loop, "maxSeconds": slot.max_seconds,
          "safety": slot.safety, "current": {"ref": f"{ref}:{slot.id}" if ref in ("stock",) else (ref or f"builtin:{slot.id}"), "source": source, "name": name,
                                             "duration": duration, "peaks": peaks}}


def library_json(store: c.Store) -> list:
  idx = store.load_library()
  items = [{"id": i, "ref": f"lib:{i}", "name": m["name"], "duration": m["duration"], "peaks": m["peaks"]}
           for i, m in idx.items() if store.lib_path(i).is_file()]
  return sorted(items, key=lambda m: store.lib_path(m["id"]).stat().st_mtime_ns, reverse=True)


async def health(request):
  return web.json_response({"ok": True})


async def get_state(request):
  app, store = request.app, request.app[STORE_KEY]
  cfg = store.load_config()
  return web.json_response({
    "onroad": bool(app[ONROAD_KEY]()),
    "enabled": cfg["enabled"],
    "limits": {"maxUploadBytes": c.MAX_UPLOAD_BYTES, "loopMaxSeconds": 2.0},
    "slots": [slot_json(app, s, cfg) for s in c.SLOTS],
    "library": library_json(store),
    "catalog": app[CATALOG_KEY],
  })


async def get_audio(request):
  store = request.app[STORE_KEY]
  ref = request.query.get("ref", "")
  slot = None
  if ref.startswith(("builtin:", "stock:")):
    kind, slot = ref.split(":", 1)
    ref = kind
    if slot not in c.SLOT_IDS:
      return err(404, "Unknown sound.")
  path = store.resolve_ref(ref, slot)
  if path is None:
    return err(404, "Unknown sound.")
  return web.Response(body=path.read_bytes(), content_type="audio/wav")


async def put_slot(request):
  app, store = request.app, request.app[STORE_KEY]
  slot = c.SLOT_IDS.get(request.match_info["slot"])
  if slot is None:
    return err(404, "Unknown slot.")
  try:
    ref = (await request.json()).get("ref")
  except Exception:
    return err(400, "Send JSON like {\"ref\": \"builtin\"}.")
  if ref in (f"builtin:{slot.id}", f"stock:{slot.id}"):
    ref = ref.split(":")[0]
  cfg = store.load_config()
  if ref == "builtin":
    cfg["slots"].pop(slot.id, None)
  else:
    path = store.resolve_ref(ref, slot.id)
    if path is None:
      return err(404, "That sound no longer exists.")
    duration, _ = describe(app, path)
    if duration > slot.max_seconds:
      return err(400, f"{slot.label} can be at most {slot.max_seconds:g} seconds. This sound is {duration:g}.")
    cfg["slots"][slot.id] = ref
  store.save_config(cfg)
  return web.json_response(slot_json(app, slot, cfg))


async def put_enabled(request):
  store = request.app[STORE_KEY]
  try:
    enabled = (await request.json()).get("enabled")
  except Exception:
    enabled = None
  if not isinstance(enabled, bool):
    return err(400, "Send JSON like {\"enabled\": true}.")
  cfg = store.load_config()
  cfg["enabled"] = enabled
  store.save_config(cfg)
  return web.json_response({"enabled": enabled})


def clean_name(raw: str) -> str:
  name = " ".join(raw.replace("\n", " ").split())[:c.MAX_NAME_LEN]
  return name or "My sound"


async def post_upload(request):
  store = request.app[STORE_KEY]
  if not (request.content_type or "").startswith("multipart/"):
    return err(400, "Upload the file as multipart form data.")
  data, name = None, "My sound"
  reader = await request.multipart()
  async for part in reader:
    if part.name == "file":
      buf = bytearray()
      while True:
        chunk = await part.read_chunk(65536)
        if not chunk:
          break
        buf += chunk
        if len(buf) > c.MAX_UPLOAD_BYTES:
          return err(413, "That file is too big. Sounds must be under 2 MB.")
      data = bytes(buf)
    elif part.name == "name":
      name = clean_name((await part.text())[:200])
  if not data:
    return err(400, "No file was uploaded.")
  idx = store.load_library()
  if len(idx) >= c.MAX_LIBRARY_ITEMS:
    return err(400, f"You already have {c.MAX_LIBRARY_ITEMS} saved sounds. Delete one first.")
  try:
    samples = c.read_wav_bytes(data)
    if len(samples) < c.MIN_SECONDS * c.SAMPLE_RATE:
      raise c.SoundError("That sound is too short.")
    normalized = c.wav_bytes(c.peak_normalize(samples))
  except c.SoundError as e:
    return err(400, str(e))
  lib_id = uuid.uuid4().hex[:12]
  path = store.lib_path(lib_id)
  c.atomic_write(path, normalized)
  duration, peaks = c.describe_file(path)
  idx[lib_id] = {"name": name, "duration": duration, "peaks": peaks}
  store.save_library(idx)
  return web.json_response({"id": lib_id, "ref": f"lib:{lib_id}", "name": name, "duration": duration, "peaks": peaks})


async def delete_library(request):
  store = request.app[STORE_KEY]
  lib_id = request.match_info["id"]
  idx = store.load_library()
  if lib_id not in idx:
    return err(404, "That sound no longer exists.")
  del idx[lib_id]
  store.save_library(idx)
  cfg = store.load_config()
  ref = f"lib:{lib_id}"
  cfg["slots"] = {k: v for k, v in cfg["slots"].items() if v != ref}
  store.save_config(cfg)
  try:
    store.lib_path(lib_id).unlink()
  except OSError:
    pass
  return web.json_response({"ok": True})


async def static(request):
  name = request.match_info.get("name") or "index.html"
  try:
    path = (WEB_DIR / name).resolve()
  except (OSError, ValueError):
    return err(404, "Not found.")
  if WEB_DIR.resolve() not in path.parents or path.suffix not in MIME or path.name.startswith(".") or not path.is_file():
    return err(404, "Not found.")
  return web.Response(body=path.read_bytes(), content_type=MIME[path.suffix])


def make_app(store: c.Store | None = None, token: str | None = None, onroad=params_onroad) -> web.Application:
  store = store or c.Store()
  app = web.Application(middlewares=[guard], client_max_size=c.MAX_UPLOAD_BYTES + 100_000)
  app[STORE_KEY] = store
  app[TOKEN_KEY] = token or load_or_create_token(store.root)
  app[ONROAD_KEY] = onroad
  app[CATALOG_KEY] = json.loads((c.CATALOG_DIR / "index.json").read_text())
  app[DESC_CACHE_KEY] = {}
  app.add_routes([
    web.get("/api/health", health),
    web.get("/api/state", get_state),
    web.get("/api/audio", get_audio),
    web.put("/api/slot/{slot}", put_slot),
    web.put("/api/enabled", put_enabled),
    web.post("/api/upload", post_upload),
    web.delete("/api/library/{id}", delete_library),
    web.get("/", static),
    web.get("/{name:.+}", static),
  ])
  return app


def main():
  logging.basicConfig(level=logging.INFO)
  app = make_app()
  log.info("soundmgr listening on :%d (token in %s)", PORT, c.DATA_DIR / "token")
  web.run_app(app, host="0.0.0.0", port=PORT, print=None, access_log=None)


if __name__ == "__main__":
  main()
