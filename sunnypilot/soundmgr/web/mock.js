// Dev-only fake backend. Load the page with ?mock=1 (optional: &onroad=1, &auth=1, &offline=1).
// Implements the same contract as the real server so the UI can be exercised without a comma.
const Q = new URLSearchParams(location.search);
const RATE = 48000;

const SLOTS = [
  { id: "prompt_distracted", label: "Pay Attention", help: "Plays on repeat while driver monitoring wants your eyes on the road", loop: true, maxSeconds: 2.0, safety: "Keep this loud and attention-grabbing. It is the first nudge before the car escalates." },
  { id: "warning_soft", label: "Warning", help: "Repeats when you need to take over soon", loop: true, maxSeconds: 2.0, safety: "Keep this loud and attention-grabbing." },
  { id: "warning_immediate", label: "Disengage immediately", help: "Repeats and gets louder when you must take over right now", loop: true, maxSeconds: 2.0, safety: "This is the escalation for a driver who is not responding. Use something jarring." },
  { id: "prompt", label: "Prompt", help: "Short nudge for things like a lane change or a speed limit change", loop: true, maxSeconds: 2.0 },
  { id: "engage", label: "Engage", help: "Plays when sunnypilot turns on", loop: false, maxSeconds: 4.0 },
  { id: "disengage", label: "Disengage", help: "Plays when sunnypilot turns off", loop: false, maxSeconds: 4.0 },
  { id: "refuse", label: "Can't engage", help: "Plays when you try to engage and the car says no", loop: false, maxSeconds: 6.0 },
  { id: "startup", label: "Startup", help: "Plays instead of the refuse sound in the first 2 minutes after the car starts", loop: false, maxSeconds: 6.0 },
];

const CATALOG = [
  ["Tesla", [["prompt_distracted", "Tesla Pay Attention", 0.55, "prompt_distracted"], ["warning_soft", "Tesla Warning", 1.2, "warning_soft"], ["warning_immediate", "Tesla Immediate", 0.65, "warning_immediate"], ["engage", "Tesla Engage", 0.84, "engage"], ["disengage", "Tesla Disengage", 0.92, "disengage"]]],
  ["Hannah Montana", [["prompt", "Hannah Montana Prompt", 1.3, "prompt"], ["engage", "Hannah Montana Engage", 1.6, "engage"], ["disengage", "Hannah Montana Disengage", 2.0, "disengage"]]],
  ["Stalin", [["engage", "Stalin Engage", 3.4, "engage"], ["disengage", "Stalin Disengage", 3.29, "startup"], ["refuse", "Stalin Refuse", 2.1, "refuse"], ["warning_soft", "Stalin Warning", 3.3, null]]],
  ["Frog", [["engage", "Frog Engage", 0.6, "engage"], ["disengage", "Frog Disengage", 0.7, "disengage"]]],
  ["Random events", [["goat", "Goat scream", 0.9, null], ["this_is_fine", "This Is Fine", 1.4, null], ["hal9000", "HAL 9000", 3.6, "refuse"], ["noice", "Noice", 1.0, null], ["fart", "Fart", 1.1, null]]],
  ["Holiday", [["halloween_prompt", "Halloween Prompt", 1.7, "prompt"], ["xmas_engage", "Christmas Engage", 2.4, "engage"]]],
];

const store = {
  enabled: true,
  current: Object.fromEntries(SLOTS.map((s) => [s.id, "builtin"])),
  library: [],
  nextId: 1,
};

const hash = (s) => [...s].reduce((h, c) => (h * 31 + c.charCodeAt(0)) >>> 0, 7);

function synth(name, dur) {
  const n = Math.round(dur * RATE), out = new Float32Array(n);
  const h = hash(name);
  const f1 = 380 + (h % 700), f2 = f1 * (1.25 + ((h >> 3) % 5) * 0.1), pulses = 1 + (h % 4);
  for (let i = 0; i < n; i++) {
    const t = i / RATE, p = (t / dur) * pulses, ph = p - Math.floor(p);
    const env = Math.sin(Math.PI * Math.min(1, ph * 1.05)) * Math.exp(-ph * 1.2) * Math.min(1, (dur - t) * 40);
    out[i] = 0.6 * env * (Math.sin(2 * Math.PI * f1 * t) * 0.6 + Math.sin(2 * Math.PI * f2 * t) * 0.4);
  }
  return out;
}

function peaksOf(samples, n = 48) {
  const out = [], step = samples.length / n;
  let mx = 0;
  for (let c = 0; c < n; c++) {
    let m = 0;
    for (let i = Math.floor(c * step); i < Math.floor((c + 1) * step); i++) m = Math.max(m, Math.abs(samples[i]));
    out.push(m); mx = Math.max(mx, m);
  }
  return out.map((v) => +(mx ? v / mx : 0).toFixed(3));
}

function wavBytes(samples) {
  const n = samples.length, buf = new ArrayBuffer(44 + n * 2), dv = new DataView(buf);
  const str = (o, s) => [...s].forEach((c, i) => dv.setUint8(o + i, c.charCodeAt(0)));
  str(0, "RIFF"); dv.setUint32(4, 36 + n * 2, true); str(8, "WAVEfmt "); dv.setUint32(16, 16, true);
  dv.setUint16(20, 1, true); dv.setUint16(22, 1, true); dv.setUint32(24, RATE, true); dv.setUint32(28, RATE * 2, true);
  dv.setUint16(32, 2, true); dv.setUint16(34, 16, true); str(36, "data"); dv.setUint32(40, n * 2, true);
  for (let i = 0; i < n; i++) dv.setInt16(44 + i * 2, Math.max(-1, Math.min(1, samples[i])) * 32767, true);
  return new Uint8Array(buf);
}

const catalogItems = [];
for (const [pack, items] of CATALOG) {
  for (const [key, name, dur, hint] of items) {
    const samples = synth(name, dur);
    catalogItems.push({ pack, ref: `catalog:${pack.toLowerCase().replace(/\W+/g, "_")}/${key}`, name, duration: dur, peaks: peaksOf(samples), slotHint: hint, samples });
  }
}
const libSamples = new Map();
function addLibrary(name, samples) {
  const id = `u${store.nextId++}`;
  libSamples.set(id, samples);
  const item = { id, ref: `lib:${id}`, name, duration: +(samples.length / RATE).toFixed(2), peaks: peaksOf(samples) };
  store.library.push(item);
  return item;
}
addLibrary("my goat", synth("my goat", 1.2));

function resolve(slotId, ref) {
  if (ref === "builtin") { const s = synth("builtin" + slotId, 0.9); return { ref: `builtin:${slotId}`, source: "builtin", name: `Built-in ${SLOTS.find((x) => x.id === slotId).label}`, duration: 0.9, peaks: peaksOf(s) }; }
  if (ref === "stock") { const s = synth("stock" + slotId, 0.87); return { ref: `stock:${slotId}`, source: "stock", name: `openpilot stock ${SLOTS.find((x) => x.id === slotId).label}`, duration: 0.87, peaks: peaksOf(s) }; }
  if (ref.startsWith("lib:")) { const i = store.library.find((x) => x.ref === ref); return { ref, source: "upload", name: i.name, duration: i.duration, peaks: i.peaks }; }
  const c = catalogItems.find((x) => x.ref === ref);
  return { ref, source: "catalog", name: c.name, duration: c.duration, peaks: c.peaks };
}

const slotObj = (s) => ({ ...s, current: resolve(s.id, store.current[s.id]) });
const state = () => ({
  onroad: Q.has("onroad"),
  enabled: store.enabled,
  limits: { maxUploadBytes: 2000000, loopMaxSeconds: 2.0 },
  slots: SLOTS.map(slotObj),
  library: store.library,
  catalog: CATALOG.map(([pack]) => ({ pack, items: catalogItems.filter((i) => i.pack === pack).map(({ samples, ...rest }) => rest) })),
});

const json = (obj, status = 200) => new Response(JSON.stringify(obj), { status, headers: { "Content-Type": "application/json" } });
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

export async function mockFetch(url, opts = {}) {
  await sleep(60);
  if (Q.has("offline")) throw new TypeError("offline");
  const u = new URL(url, location.href), path = u.pathname.replace(/^.*\/api\//, "/api/");
  const method = opts.method || "GET";
  const auth = (opts.headers || {}).Authorization;
  if (path === "/api/health") return json({ ok: true });
  if (Q.has("auth") && auth !== "Bearer test") return json({ error: "bad token" }, 401);
  const write = method !== "GET";
  if (write && Q.has("onroad")) return json({ error: "The car is on." }, 423);

  if (path === "/api/state") return json(state());
  if (path === "/api/audio") {
    const ref = u.searchParams.get("ref");
    let samples;
    if (ref.startsWith("lib:")) samples = libSamples.get(ref.slice(4));
    else if (ref.startsWith("catalog:")) samples = catalogItems.find((x) => x.ref === ref).samples;
    else samples = synth(ref, 0.9);
    return new Response(wavBytes(samples), { headers: { "Content-Type": "audio/wav" } });
  }
  let m;
  if ((m = path.match(/^\/api\/slot\/(\w+)$/)) && method === "PUT") {
    const slot = SLOTS.find((s) => s.id === m[1]);
    const { ref } = JSON.parse(opts.body);
    const r = resolve(slot.id, ref);
    if (r.duration > slot.maxSeconds) return json({ error: `That sound is ${r.duration.toFixed(2)} s; ${slot.label} allows ${slot.maxSeconds.toFixed(1)} s.` }, 400);
    store.current[slot.id] = ref;
    return json(slotObj(slot));
  }
  if (path === "/api/upload" && method === "POST") {
    const file = opts.body.get("file"), name = opts.body.get("name") || "Untitled";
    const dv = new DataView(await file.arrayBuffer());
    const n = (dv.byteLength - 44) >> 1, samples = new Float32Array(n);
    for (let i = 0; i < n; i++) samples[i] = dv.getInt16(44 + i * 2, true) / 32768;
    if (n / RATE > 6.0001) return json({ error: "That clip is longer than 6 seconds." }, 400);
    return json(addLibrary(name, samples));
  }
  if ((m = path.match(/^\/api\/library\/(\w+)$/)) && method === "DELETE") {
    const ref = `lib:${m[1]}`;
    store.library = store.library.filter((i) => i.ref !== ref);
    for (const k of Object.keys(store.current)) if (store.current[k] === ref) store.current[k] = "builtin";
    return json({ ok: true });
  }
  if (path === "/api/enabled" && method === "PUT") {
    store.enabled = !!JSON.parse(opts.body).enabled;
    return json({ enabled: store.enabled });
  }
  return json({ error: "not found" }, 404);
}
