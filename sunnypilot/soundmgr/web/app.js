// Sound Manager frontend. No build step, no external requests.
const PARAMS = new URLSearchParams(location.search);
const MOCK = PARAMS.has("mock");
const TOKEN_KEY = "soundmgr.token";
const LIBRARY_MAX_SECONDS = 6;
const TARGET_RATE = 48000;

let rawFetch = (...a) => window.fetch(...a);

// ---------- token ----------
(function grabTokenFromFragment() {
  const m = /[#&]t=([^&]+)/.exec(location.hash);
  if (!m) return;
  try { localStorage.setItem(TOKEN_KEY, decodeURIComponent(m[1])); } catch { /* private mode */ }
  history.replaceState(null, "", location.pathname + location.search);
})();
const getToken = () => { try { return localStorage.getItem(TOKEN_KEY) || ""; } catch { return ""; } };
const setToken = (t) => { try { localStorage.setItem(TOKEN_KEY, t); } catch { /* private mode */ } };

// ---------- helpers ----------
const esc = (s) => String(s ?? "").replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));
const fmt = (s) => `${Number(s).toFixed(2)}\u00a0s`;
const fmtMax = (s) => `${Number(s).toFixed(1)}\u00a0s`;
const $ = (sel, root = document) => root.querySelector(sel);
const $$ = (sel, root = document) => [...root.querySelectorAll(sel)];
const clamp = (v, a, b) => Math.min(b, Math.max(a, v));

const SOURCE_LABEL = { builtin: "Built-in", stock: "openpilot stock", catalog: "Catalog", upload: "Your upload" };
const ICON = '<svg class="i-play" viewBox="0 0 16 16" aria-hidden="true"><path d="M3 1.5v13l11-6.5z"/></svg><svg class="i-stop" viewBox="0 0 16 16" aria-hidden="true"><rect x="3" y="3" width="10" height="10"/></svg>';

function resample(peaks, n) {
  if (!peaks || !peaks.length) return Array(n).fill(0.05);
  if (peaks.length === n) return peaks;
  const out = [];
  for (let i = 0; i < n; i++) {
    const a = Math.floor((i * peaks.length) / n), b = Math.max(a + 1, Math.floor(((i + 1) * peaks.length) / n));
    let m = 0;
    for (let j = a; j < b && j < peaks.length; j++) m = Math.max(m, peaks[j]);
    out.push(m);
  }
  return out;
}

function waveHtml(peaks, ref, n = 48) {
  const bars = resample(peaks, n).map((v) => `<i style="--h:${clamp(v, 0, 1).toFixed(2)}"></i>`).join("");
  return `<div class="wave" data-wave="${esc(ref)}" aria-hidden="true"><div class="bars">${bars}</div><div class="bars fill">${bars}</div></div>`;
}

function playBtn(ref, label, cls = "") {
  return `<button class="play ${cls}" type="button" data-action="play" data-ref="${esc(ref)}" aria-pressed="false" aria-label="Play ${esc(label)}">${ICON}</button>`;
}

// ---------- api ----------
class ApiError extends Error {
  constructor(status, message) { super(message); this.status = status; }
}

async function api(path, { method = "GET", json, form, raw } = {}) {
  const headers = {};
  const token = getToken();
  if (token) headers.Authorization = `Bearer ${token}`;
  let body;
  if (json !== undefined) { headers["Content-Type"] = "application/json"; body = JSON.stringify(json); }
  else if (form) body = form;
  let res;
  try { res = await rawFetch(path, { method, headers, body }); }
  catch { throw new ApiError(0, "Can't reach the comma."); }
  if (res.status === 401) throw new ApiError(401, "That access token was not accepted.");
  if (!res.ok) {
    let msg = `The comma answered with an error (${res.status}).`;
    try { const j = await res.json(); if (j && j.error) msg = j.error; } catch { /* keep default */ }
    throw new ApiError(res.status, msg);
  }
  return raw ? res : res.json();
}

// ---------- state ----------
const S = { phase: "loading", data: null, sheet: null, upload: null, error: "" };
const app = $("#app");
const dlg = $("#sheet");

const slotById = (id) => S.data && S.data.slots.find((s) => s.id === id);
const readOnly = () => !!(S.data && S.data.onroad);
const ro = () => (readOnly() ? " disabled" : "");

async function loadState() {
  try {
    S.data = await api("/api/state");
    S.phase = "ready";
  } catch (e) {
    if (e.status === 401) S.phase = "auth";
    else { S.phase = "offline"; S.error = e.status ? e.message : ""; }
  }
  render();
  if (S.sheet) { if (S.phase === "ready") renderSheet(); else closeSheet(); }
}

async function guard(fn) {
  try { return await fn(); }
  catch (e) {
    if (!(e instanceof ApiError)) { console.error(e); toast("Something went wrong in the page.", { bad: true }); return; }
    if (e.status === 401) { S.phase = "auth"; closeSheet(); render(); return; }
    if (e.status === 423) { toast("The car is on. Park it and try again.", { bad: true }); loadState(); return; }
    if (e.status === 0) { toast("Lost the connection to the comma.", { bad: true }); return; }
    toast(e.message, { bad: true });
  }
}

// ---------- audio playback ----------
const SILENCE = "data:audio/wav;base64,UklGRiQAAABXQVZFZm10IBAAAAABAAEAgLsAAIB3AQACABAAZGF0YQAAAAA=";
const player = { audio: new Audio(), key: null, cache: new Map(), raf: 0, unlocked: false, previewUrl: null };
player.audio.preload = "auto";
player.audio.addEventListener("ended", () => stopPlayback());

function unlockAudio() {
  // iOS only lets an <audio> element play after async work if a user gesture has touched it once.
  if (player.unlocked) return;
  player.unlocked = true;
  player.audio.src = SILENCE;
  player.audio.play().catch(() => { player.unlocked = false; });
}

async function audioUrl(ref) {
  if (ref.startsWith("blob:")) return ref;
  if (player.cache.has(ref)) return player.cache.get(ref);
  const res = await api(`/api/audio?ref=${encodeURIComponent(ref)}`, { raw: true });
  const url = URL.createObjectURL(await res.blob());
  player.cache.set(ref, url);
  return url;
}

function setPlayingUI() {
  $$('[data-action="play"]').forEach((b) => {
    const on = b.dataset.ref === player.key;
    b.classList.toggle("playing", on);
    b.setAttribute("aria-pressed", String(on));
  });
}

function setProgress(p) {
  $$("[data-wave]").forEach((el) => el.style.setProperty("--p", el.dataset.wave === player.key ? p : 0));
}

function tick() {
  if (player.key == null) return;
  const a = player.audio;
  setProgress(a.duration ? a.currentTime / a.duration : 0);
  player.raf = requestAnimationFrame(tick);
}

function stopPlayback() {
  cancelAnimationFrame(player.raf);
  player.audio.pause();
  player.key = null;
  setProgress(0);
  setPlayingUI();
}

async function togglePlay(ref, urlOverride) {
  if (player.key === ref && !player.audio.paused) { stopPlayback(); return; }
  stopPlayback();
  player.key = ref;
  setPlayingUI();
  try {
    const url = urlOverride || await audioUrl(ref);
    if (player.key !== ref) return;
    player.audio.src = url;
    player.audio.currentTime = 0;
    await player.audio.play();
    tick();
  } catch (e) {
    if (player.key === ref) stopPlayback();
    if (e instanceof ApiError) return guard(() => { throw e; });
    toast("Couldn't play that sound.", { bad: true });
  }
}

// ---------- toast ----------
let toastTimer = 0;
function toast(message, { undo, bad } = {}) {
  $$(".toast").forEach((t) => t.remove());
  clearTimeout(toastTimer);
  const el = document.createElement("div");
  el.className = `toast${bad ? " bad" : ""}`;
  el.setAttribute("role", "status");
  el.innerHTML = `<span>${esc(message)}</span>${undo ? '<button type="button">Undo</button>' : ""}`;
  if (undo) $("button", el).addEventListener("click", () => { el.remove(); undo(); });
  (dlg.open ? dlg : document.body).appendChild(el);
  toastTimer = setTimeout(() => el.remove(), undo ? 7000 : 4500);
}

// ---------- main view ----------
function hostLabel() {
  const h = location.hostname;
  if (!h) return "";
  return /^[\d.]+$/.test(h) || h.includes(":") ? h : h.split(".")[0];
}

function slotCard(slot, first) {
  const c = slot.current;
  const off = S.data && !S.data.enabled;
  const tags = [slot.loop ? "loops" : "plays once", `max ${fmtMax(slot.maxSeconds)}`].map((t) => `<span class="tag">${esc(t)}</span>`).join("");
  return `<article class="slot${first ? " hero" : ""}${off ? " off" : ""}">
    <button class="slot-main" type="button" data-action="open" data-slot="${esc(slot.id)}" aria-label="Change ${esc(slot.label)} sound">
      <span class="slot-top"><span class="slot-label">${esc(slot.label)}</span><span class="tags">${tags}</span></span>
      <span class="slot-help">${esc(slot.help)}</span>
      <span class="slot-cur">
        <span class="cur-name">${esc(c.name)}</span>
        <span class="tag src-${esc(c.source)}">${esc(SOURCE_LABEL[c.source] || c.source)}</span>
        <span class="dur">${fmt(c.duration)}</span>
      </span>
      ${waveHtml(c.peaks, c.ref)}
    </button>
    ${playBtn(c.ref, `${slot.label}: ${c.name}`)}
  </article>`;
}

function libraryRow(item) {
  return `<div class="lib-row">
    ${playBtn(item.ref, item.name)}
    <div><div class="lib-name">${esc(item.name)}</div><div class="lib-meta"><span class="dur">${fmt(item.duration)}</span></div></div>
    <button class="btn small danger" type="button" data-action="delete" data-id="${esc(item.id)}"${ro()}>Delete</button>
  </div>`;
}

function renderReady() {
  const d = S.data;
  const custom = d.slots.filter((s) => s.current.source !== "builtin").length;
  const loops = d.slots.filter((s) => s.loop);
  const once = d.slots.filter((s) => !s.loop);
  const firstId = d.slots[0] && d.slots[0].id;
  const host = hostLabel();
  app.innerHTML = `
    <header>
      <div class="eyebrow">Alert sounds${host ? ` &middot; ${esc(host)}` : ""}</div>
      <h1>Sound Manager</h1>
    </header>
    ${d.onroad ? `<div class="banner" role="alert"><b>Car is on</b>You can listen to sounds, but changes are locked until the car is parked and off.</div>` : ""}
    <section class="card status" aria-label="Status">
      <div class="status-row">
        <span class="dot${d.onroad ? " bad" : ""}" aria-hidden="true"></span>
        <div><b>${d.onroad ? "Driving &middot; editing locked" : "Parked &middot; editing on"}</b>
        <small>${custom} of ${d.slots.length} sounds customised</small></div>
      </div>
      <div class="switch-row">
        <span><b id="en-label">Custom sounds</b><small>Off plays the built-in sounds everywhere.</small></span>
        <button class="switch" type="button" role="switch" aria-checked="${d.enabled}" aria-labelledby="en-label" data-action="toggle-enabled"${ro()}></button>
      </div>
      <p class="note">Changes apply the next time the car starts.</p>
    </section>
    <section>
      <div class="sec-head"><h2>Repeating alerts</h2><p class="sec-sub">These play until you respond, so they stay short. Keep them loud and hard to ignore.</p></div>
      <div class="slots">${loops.map((s) => slotCard(s, s.id === firstId)).join("")}</div>
    </section>
    <section>
      <div class="sec-head"><h2>One-time sounds</h2><p class="sec-sub">Chimes that play once and stop.</p></div>
      <div class="slots">${once.map((s) => slotCard(s, false)).join("")}</div>
    </section>
    <section>
      <div class="sec-head"><h2>My sounds</h2><p class="sec-sub">Clips you added. Pick them from any slot.</p></div>
      ${d.library.length ? `<div class="lib">${d.library.map(libraryRow).join("")}</div>` : `<p class="lib-empty">Nothing here yet. Add an MP3, M4A or WAV and trim it to fit.</p>`}
      <div class="lib-actions"><button class="btn" type="button" data-action="add-sound"${ro()}>Add a sound</button></div>
    </section>`;
  setPlayingUI();
}

function renderAuth() {
  app.innerHTML = `
    <header><div class="eyebrow">Alert sounds</div><h1>Sound Manager</h1></header>
    <form class="card center-card" id="token-form">
      <h2>Enter access token</h2>
      <p>The comma needs a token before it lets this page change anything. Open the link with the token in it once, or paste it here.</p>
      <label class="field"><span>Access token</span><input id="token-input" type="password" autocomplete="off" autocapitalize="none" spellcheck="false" required></label>
      ${S.error ? `<p class="error-line">${esc(S.error)}</p>` : ""}
      <button class="btn primary" type="submit">Connect</button>
    </form>`;
}

function renderOffline() {
  app.innerHTML = `
    <header><div class="eyebrow">Alert sounds</div><h1>Sound Manager</h1></header>
    <section class="card center-card">
      <h2>Can't reach the comma</h2>
      <p>Is Tailscale on and the car off? The sound manager only runs while the car is parked.</p>
      ${S.error ? `<p class="error-line">${esc(S.error)}</p>` : ""}
      <button class="btn primary" type="button" data-action="retry">Try again</button>
    </section>`;
}

function render() {
  if (S.phase === "loading") app.innerHTML = `<header><div class="eyebrow">Alert sounds</div><h1>Sound Manager</h1></header><p class="loading">Connecting to the comma...</p>`;
  else if (S.phase === "auth") renderAuth();
  else if (S.phase === "offline") renderOffline();
  else renderReady();
}

// ---------- sheet ----------
function openSheet(slotId, tab) {
  unlockAudio();
  S.sheet = { slotId, tab: tab || (slotId ? "catalog" : "upload"), query: "", open: new Set() };
  S.upload = null;
  renderSheet();
  if (!dlg.open) dlg.showModal();
}

function closeSheet() {
  if (dlg.open) dlg.close();
  S.sheet = null;
  S.upload = null;
  stopPlayback();
}

dlg.addEventListener("close", () => { S.sheet = null; S.upload = null; stopPlayback(); });
dlg.addEventListener("click", (e) => { if (e.target === dlg) closeSheet(); });

function prevRef(slot) {
  const c = slot.current;
  return c.source === "builtin" ? "builtin" : c.source === "stock" ? "stock" : c.ref;
}

function maxSecondsForSheet() {
  const slot = S.sheet && slotById(S.sheet.slotId);
  return slot ? slot.maxSeconds : LIBRARY_MAX_SECONDS;
}

function itemRow(item, slot) {
  const tooLong = item.duration > slot.maxSeconds + 1e-6;
  const inUse = slot.current.ref === item.ref;
  return `<div class="item${inUse ? " inuse" : ""}">
    ${playBtn(item.ref, item.name)}
    <div class="item-main">
      <div class="item-name">${esc(item.name)}</div>
      <div class="item-meta"><span class="dur">${fmt(item.duration)}</span>${tooLong ? `<span class="warn">Too long for ${esc(slot.label)} (max ${fmtMax(slot.maxSeconds)})</span>` : ""}</div>
    </div>
    ${inUse ? '<span class="badge">In use</span>' : `<button class="btn small primary" type="button" data-action="use" data-ref="${esc(item.ref)}"${tooLong || readOnly() ? " disabled" : ""}>Use</button>`}
    ${waveHtml(item.peaks, item.ref, 32)}
  </div>`;
}

function renderCatalog(slot) {
  const q = (S.sheet.query || "").trim().toLowerCase();
  const packs = S.data.catalog.map((p) => ({ pack: p.pack, items: p.items.filter((i) => !q || i.name.toLowerCase().includes(q) || p.pack.toLowerCase().includes(q)) })).filter((p) => p.items.length);
  const suggested = q ? [] : S.data.catalog.flatMap((p) => p.items).filter((i) => i.slotHint === slot.id);
  if (!packs.length) return `<p class="empty">No sounds match &ldquo;${esc(S.sheet.query)}&rdquo;.</p>`;
  const suggestHtml = suggested.length ? `<div><p class="group-label">Made for ${esc(slot.label)}</p><div class="items">${suggested.map((i) => itemRow(i, slot)).join("")}</div></div>` : "";
  const packHtml = packs.map((p) => {
    const open = q || S.sheet.open.has(p.pack);
    return `<details class="pack" data-pack="${esc(p.pack)}"${open ? " open" : ""}><summary><span class="pack-title">${esc(p.pack)}</span><span class="count">${p.items.length}</span></summary><div class="items">${p.items.map((i) => itemRow(i, slot)).join("")}</div></details>`;
  }).join("");
  return `${suggestHtml}<div>${packHtml}</div>`;
}

function renderMine(slot) {
  const lib = S.data.library;
  if (!lib.length) return `<p class="empty">Nothing here yet. Upload a clip and it shows up here.</p><button class="btn block" type="button" data-action="tab" data-tab="upload">Upload a sound</button>`;
  return `<div class="items">${lib.map((i) => `<div class="item${slot.current.ref === i.ref ? " inuse" : ""}">
    ${playBtn(i.ref, i.name)}
    <div class="item-main"><div class="item-name">${esc(i.name)}</div><div class="item-meta"><span class="dur">${fmt(i.duration)}</span>${i.duration > slot.maxSeconds + 1e-6 ? `<span class="warn">Too long for ${esc(slot.label)} (max ${fmtMax(slot.maxSeconds)})</span>` : ""}</div></div>
    <div class="item-actions">
      ${slot.current.ref === i.ref ? '<span class="badge">In use</span>' : `<button class="btn small primary" type="button" data-action="use" data-ref="${esc(i.ref)}"${i.duration > slot.maxSeconds + 1e-6 || readOnly() ? " disabled" : ""}>Use</button>`}
      <button class="btn small danger" type="button" data-action="delete" data-id="${esc(i.id)}"${ro()} aria-label="Delete ${esc(i.name)}">Del</button>
    </div>
    ${waveHtml(i.peaks, i.ref, 32)}
  </div>`).join("")}</div>`;
}

function renderUploadPanel(slot) {
  const max = maxSecondsForSheet();
  const u = S.upload;
  const applyLabel = slot ? `Add &amp; use for ${esc(slot.label)}` : "Add to My sounds";
  const hasData = !!(u && u.data);
  const picker = `<div class="drop">
      <label class="btn"><input class="sr-only" id="file" type="file" accept="audio/*"${ro()}><span>${hasData ? "Choose a different file" : "Choose audio file"}</span></label>
      ${hasData ? "" : `<p>MP3, M4A, WAV or a Voice Memo. It is converted to the car's format here, and you can trim it next. ${slot ? `${esc(slot.label)} takes up to ${fmtMax(max)}.` : `Up to ${fmtMax(max)}.`}</p>`}
    </div>`;
  if (readOnly()) return `<p class="empty">Uploads are locked while the car is on.</p>`;
  if (!u) return picker;
  if (u.busy === "decoding") return `${picker}<p class="loading">Reading audio...</p>`;
  if (u.error) return `${picker}<p class="error-line" role="alert">${esc(u.error)}</p>`;
  if (!hasData) return picker;
  return `<div class="trim" id="trim">
      <canvas id="trim-canvas" aria-hidden="true"></canvas>
      <div class="trim-dim left"></div><div class="trim-dim right"></div>
      <div class="trim-sel" data-drag="move" data-wave="__preview"><div class="trim-prog"></div></div>
      <button class="trim-h start" type="button" role="slider" data-drag="start" aria-label="Trim start"></button>
      <button class="trim-h end" type="button" role="slider" data-drag="end" aria-label="Trim end"></button>
    </div>
    <div class="trim-read"><span id="trim-range"></span><span id="trim-len"></span></div>
    <div class="trim-tools">
      <button class="btn" type="button" data-action="play" data-ref="__preview" aria-pressed="false"><svg class="i-play" viewBox="0 0 16 16" aria-hidden="true"><path d="M3 1.5v13l11-6.5z"/></svg><svg class="i-stop" viewBox="0 0 16 16" aria-hidden="true"><rect x="3" y="3" width="10" height="10"/></svg><span class="lbl-play">Preview</span><span class="lbl-stop">Stop</span></button>
      <button class="btn" type="button" data-action="fit">Fit to ${fmtMax(max)}</button>
    </div>
    <label class="field"><span>Name</span><input id="up-name" type="text" maxlength="40" value="${esc(u.name)}" autocomplete="off"></label>
    <label class="check"><input id="up-norm" type="checkbox" ${u.normalize ? "checked" : ""}> Make it loud (normalize)</label>
    <p class="hint">Quiet clips get lost in a moving car. The comma also levels every upload to just under full scale.</p>
    <button class="btn primary block" id="up-go" type="button" data-action="upload-go">${applyLabel}</button>
    ${picker}`;
}

function renderSheet() {
  const sh = S.sheet;
  if (!sh || !S.data) return;
  const slot = sh.slotId ? slotById(sh.slotId) : null;
  if (sh.slotId && !slot) { closeSheet(); return; }
  const prevScroll = $(".sheet-scroll", dlg);
  const top = prevScroll ? prevScroll.scrollTop : 0;
  const tabs = slot ? [["catalog", "Catalog"], ["mine", "My sounds"], ["upload", "Upload"]] : [["upload", "Upload"]];
  const seg = `<div class="seg${slot ? "" : " single"}" role="tablist" aria-label="Sound source">${tabs.map(([id, label]) => `<button type="button" role="tab" data-action="tab" data-tab="${id}" aria-selected="${sh.tab === id}">${label}</button>`).join("")}</div>`;
  let panel = "";
  if (sh.tab === "catalog" && slot) panel = `<label class="field"><span>Search</span><input id="cat-q" type="search" value="${esc(sh.query)}" placeholder="goat, tesla, hal..." autocomplete="off"></label><div id="cat-list" class="sheet-list">${renderCatalog(slot)}</div>`;
  else if (sh.tab === "mine" && slot) panel = renderMine(slot);
  else panel = `<div id="upload-panel">${renderUploadPanel(slot)}</div>`;
  const c = slot && slot.current;
  const liveToast = $(".toast", dlg);
  dlg.innerHTML = `<div class="sheet-inner">
    <div class="sheet-head">
      <div>
        <div class="eyebrow">${slot ? `${slot.loop ? "Loops" : "Plays once"} &middot; max ${fmtMax(slot.maxSeconds)}` : "Library"}</div>
        <h2 id="sheet-title">${slot ? esc(slot.label) : "Add a sound"}</h2>
      </div>
      <button class="icon-btn" type="button" data-action="close" aria-label="Close">&times;</button>
    </div>
    <div class="sheet-scroll">
      ${slot ? `<p class="help">${esc(slot.help)}</p>${slot.safety ? `<p class="safety"><b>Safety</b>${esc(slot.safety)}</p>` : ""}
      <div class="now">
        ${playBtn(c.ref, `current ${slot.label} sound`)}
        <div class="now-name"><b>${esc(c.name)}</b><div class="meta"><span class="tag src-${esc(c.source)}">${esc(SOURCE_LABEL[c.source] || c.source)}</span><span class="dur">${fmt(c.duration)}</span></div></div>
      </div>` : ""}
      ${seg}
      ${panel}
    </div>
    ${slot ? `<div class="sheet-foot">
      <div class="row">
        <button class="btn" type="button" data-action="use" data-ref="builtin"${c.source === "builtin" || readOnly() ? " disabled" : ""}>Reset to built-in</button>
        <button class="btn" type="button" data-action="use" data-ref="stock"${c.source === "stock" || readOnly() ? " disabled" : ""}>openpilot stock</button>
      </div>
      <small>Built-in: this build's default. openpilot stock: the original comma sound.</small>
    </div>` : ""}
  </div>`;
  if (liveToast) dlg.appendChild(liveToast);
  const sc = $(".sheet-scroll", dlg);
  if (sc) sc.scrollTop = top;
  if (sh.tab === "upload") mountTrim();
  setPlayingUI();
}

async function useRef(slotId, ref, { restore } = {}) {
  const slot = slotById(slotId);
  if (!slot) return;
  const before = prevRef(slot);
  await guard(async () => {
    const upd = await api(`/api/slot/${encodeURIComponent(slotId)}`, { method: "PUT", json: { ref } });
    const i = S.data.slots.findIndex((s) => s.id === slotId);
    S.data.slots[i] = upd;
    render();
    if (S.sheet) renderSheet();
    if (restore) toast(`${upd.label} restored`);
    else toast(`${upd.label} set to ${upd.current.name}`, { undo: () => useRef(slotId, before, { restore: true }) });
  });
}

// ---------- upload: decode, trim, encode ----------
async function decodeFile(file) {
  if (file.size > 60e6) throw new Error("That file is over 60 MB. Pick a shorter clip.");
  const Ctx = window.AudioContext || window.webkitAudioContext;
  const OAC = window.OfflineAudioContext || window.webkitOfflineAudioContext;
  if (!Ctx || !OAC) throw new Error("This browser can't convert audio. Try Safari or Chrome.");
  const bytes = await file.arrayBuffer();
  const ctx = new Ctx();
  let decoded;
  try {
    decoded = await new Promise((res, rej) => {
      const p = ctx.decodeAudioData(bytes, res, rej);
      if (p && p.catch) p.catch(() => {}); // the callback form already reports the failure
    });
  }
  catch { throw new Error("Couldn't read that file as audio. Try an MP3, M4A or WAV."); }
  finally { if (ctx.close) ctx.close().catch(() => {}); }
  if (decoded.duration > 180) throw new Error("That clip is over 3 minutes. Cut it down in another app first.");
  const off = new OAC(1, Math.max(1, Math.ceil(decoded.duration * TARGET_RATE)), TARGET_RATE);
  const src = off.createBufferSource();
  src.buffer = decoded;
  src.connect(off.destination);
  src.start();
  const rendered = await off.startRendering();
  return rendered.getChannelData(0).slice();
}

function columnPeaks(data, cols) {
  const out = new Float32Array(cols);
  const step = data.length / cols;
  for (let c = 0; c < cols; c++) {
    let m = 0;
    const a = Math.floor(c * step), b = Math.min(data.length, Math.floor((c + 1) * step) + 1);
    for (let i = a; i < b; i++) { const v = Math.abs(data[i]); if (v > m) m = v; }
    out[c] = m;
  }
  let peak = 0;
  for (const v of out) if (v > peak) peak = v;
  if (peak > 0) for (let c = 0; c < cols; c++) out[c] /= peak;
  return out;
}

function encodeWav(samples, normalize) {
  let peak = 0;
  for (let i = 0; i < samples.length; i++) { const v = Math.abs(samples[i]); if (v > peak) peak = v; }
  const gain = normalize && peak > 0.001 ? 0.891 / peak : 1;
  const n = samples.length;
  const buf = new ArrayBuffer(44 + n * 2);
  const dv = new DataView(buf);
  const str = (o, s) => { for (let i = 0; i < s.length; i++) dv.setUint8(o + i, s.charCodeAt(i)); };
  str(0, "RIFF"); dv.setUint32(4, 36 + n * 2, true); str(8, "WAVE"); str(12, "fmt ");
  dv.setUint32(16, 16, true); dv.setUint16(20, 1, true); dv.setUint16(22, 1, true);
  dv.setUint32(24, TARGET_RATE, true); dv.setUint32(28, TARGET_RATE * 2, true); dv.setUint16(32, 2, true); dv.setUint16(34, 16, true);
  str(36, "data"); dv.setUint32(40, n * 2, true);
  const fadeIn = Math.min(48, n >> 2), fadeOut = Math.min(144, n >> 1);
  for (let i = 0; i < n; i++) {
    let v = samples[i] * gain;
    if (i < fadeIn) v *= i / fadeIn;
    if (i >= n - fadeOut) v *= (n - 1 - i) / fadeOut;
    v = clamp(v, -1, 1);
    dv.setInt16(44 + i * 2, Math.round(v < 0 ? v * 32768 : v * 32767), true);
  }
  return new Blob([buf], { type: "audio/wav" });
}

function trimmedSamples() {
  const u = S.upload;
  return u.data.subarray(Math.floor(u.start * TARGET_RATE), Math.ceil(u.end * TARGET_RATE));
}

function drawTrim() {
  const u = S.upload, cv = $("#trim-canvas");
  if (!u || !u.data || !cv) return;
  const dpr = window.devicePixelRatio || 1;
  const w = cv.clientWidth, h = cv.clientHeight;
  if (!w) return;
  cv.width = Math.round(w * dpr); cv.height = Math.round(h * dpr);
  const g = cv.getContext("2d");
  g.scale(dpr, dpr);
  const css = getComputedStyle(document.documentElement);
  g.fillStyle = css.getPropertyValue("--bar").trim() || "#888";
  const cols = Math.floor(w / 3);
  if (!u.cols || u.cols.length !== cols) u.cols = columnPeaks(u.data, cols);
  for (let c = 0; c < cols; c++) {
    const bh = Math.max(2, u.cols[c] * (h - 16));
    g.fillRect(c * 3, (h - bh) / 2, 2, bh);
  }
}

function updateTrimUI() {
  const u = S.upload;
  const trim = $("#trim");
  if (!u || !trim) return;
  const dur = u.data.length / TARGET_RATE;
  trim.style.setProperty("--s", u.start / dur);
  trim.style.setProperty("--e", u.end / dur);
  const len = u.end - u.start, max = maxSecondsForSheet();
  const over = len > max + 1e-6;
  $("#trim-range").textContent = `${u.start.toFixed(2)} s to ${u.end.toFixed(2)} s of ${dur.toFixed(2)} s`;
  $("#trim-len").innerHTML = `<span class="${over ? "over" : ""}">${len.toFixed(2)} s of ${fmtMax(max)} max${over ? " (too long)" : ""}</span>`;
  const s = $(".trim-h.start", trim), e = $(".trim-h.end", trim);
  s.setAttribute("aria-valuemin", "0"); s.setAttribute("aria-valuemax", String(dur.toFixed(2))); s.setAttribute("aria-valuenow", u.start.toFixed(2));
  e.setAttribute("aria-valuemin", "0"); e.setAttribute("aria-valuemax", String(dur.toFixed(2))); e.setAttribute("aria-valuenow", u.end.toFixed(2));
  const go = $("#up-go");
  if (go) go.disabled = over || u.posting || readOnly() || !$("#up-name").value.trim();
}

function mountTrim() {
  if (S.upload && S.upload.data) { drawTrim(); updateTrimUI(); }
}

async function handleFile(file) {
  if (!file) return;
  stopPlayback();
  S.upload = { busy: "decoding" };
  refreshUploadPanel();
  try {
    const data = await decodeFile(file);
    if (!data.length) throw new Error("That file has no audio in it.");
    const dur = data.length / TARGET_RATE;
    const base = file.name.replace(/\.[^.]+$/, "").replace(/[_-]+/g, " ").trim().slice(0, 40) || "My sound";
    S.upload = { data, name: base, start: 0, end: Math.min(dur, maxSecondsForSheet()), normalize: true };
  } catch (e) {
    S.upload = { error: e.message || "Couldn't read that file." };
  }
  refreshUploadPanel();
}

function refreshUploadPanel() {
  const host = $("#upload-panel");
  if (!host || !S.sheet) return;
  const slot = S.sheet.slotId ? slotById(S.sheet.slotId) : null;
  host.innerHTML = renderUploadPanel(slot);
  mountTrim();
  setPlayingUI();
}

async function uploadGo() {
  const u = S.upload;
  if (!u || !u.data || u.posting) return;
  const slotId = S.sheet && S.sheet.slotId;
  const name = $("#up-name").value.trim().slice(0, 40);
  if (!name) return;
  stopPlayback();
  u.posting = true;
  const go = $("#up-go");
  go.disabled = true; go.textContent = "Uploading...";
  await guard(async () => {
    const wav = encodeWav(trimmedSamples(), u.normalize);
    const form = new FormData();
    form.append("file", wav, `${name}.wav`);
    form.append("name", name);
    const item = await api("/api/upload", { method: "POST", form });
    S.data.library.push(item);
    S.upload = null;
    if (slotId) { if (S.sheet) S.sheet.tab = "mine"; await useRef(slotId, item.ref); }
    else { render(); closeSheet(); toast(`Added ${item.name} to My sounds`); }
  });
  if (S.upload) { S.upload.posting = false; refreshUploadPanel(); }
}

// trim dragging
let drag = null;
function onDragStart(e) {
  const el = e.target.closest("[data-drag]");
  const trim = $("#trim");
  if (!el || !trim || !S.upload) return;
  e.preventDefault();
  stopPlayback();
  el.setPointerCapture && el.setPointerCapture(e.pointerId);
  const u = S.upload;
  drag = { mode: el.dataset.drag, x: e.clientX, start: u.start, end: u.end, width: trim.getBoundingClientRect().width, dur: u.data.length / TARGET_RATE, id: e.pointerId, el };
}
function onDragMove(e) {
  if (!drag || e.pointerId !== drag.id) return;
  const u = S.upload, dt = ((e.clientX - drag.x) / drag.width) * drag.dur, min = 0.05;
  if (drag.mode === "start") u.start = clamp(drag.start + dt, 0, drag.end - min);
  else if (drag.mode === "end") u.end = clamp(drag.end + dt, drag.start + min, drag.dur);
  else { const len = drag.end - drag.start; const s = clamp(drag.start + dt, 0, drag.dur - len); u.start = s; u.end = s + len; }
  updateTrimUI();
}
function onDragEnd(e) { if (drag && e.pointerId === drag.id) drag = null; }

function onTrimKey(e) {
  const h = e.target.closest(".trim-h");
  if (!h || !S.upload) return;
  const u = S.upload, dur = u.data.length / TARGET_RATE;
  const step = (e.shiftKey ? 0.1 : 0.01) * (e.key === "ArrowLeft" || e.key === "ArrowDown" ? -1 : e.key === "ArrowRight" || e.key === "ArrowUp" ? 1 : 0);
  if (!step) return;
  e.preventDefault();
  stopPlayback();
  if (h.dataset.drag === "start") u.start = clamp(u.start + step, 0, u.end - 0.05);
  else u.end = clamp(u.end + step, u.start + 0.05, dur);
  updateTrimUI();
}

async function previewUpload() {
  const u = S.upload;
  if (!u || !u.data) return;
  if (player.key === "__preview") { stopPlayback(); return; }
  if (player.previewUrl) URL.revokeObjectURL(player.previewUrl);
  player.previewUrl = URL.createObjectURL(encodeWav(trimmedSamples(), u.normalize));
  togglePlay("__preview", player.previewUrl);
}

// ---------- events ----------
document.addEventListener("pointerdown", unlockAudio, { once: true, capture: true });
document.addEventListener("pointerdown", onDragStart);
document.addEventListener("pointermove", onDragMove);
document.addEventListener("pointerup", onDragEnd);
document.addEventListener("pointercancel", onDragEnd);
document.addEventListener("keydown", onTrimKey);

document.addEventListener("click", async (e) => {
  const t = e.target.closest("[data-action]");
  if (!t || t.disabled) return;
  const a = t.dataset.action;
  if (a === "play") {
    unlockAudio();
    if (t.dataset.ref === "__preview") previewUpload(); else togglePlay(t.dataset.ref);
  } else if (a === "open") openSheet(t.dataset.slot);
  else if (a === "close") closeSheet();
  else if (a === "add-sound") openSheet(null, "upload");
  else if (a === "tab") { stopPlayback(); S.sheet.tab = t.dataset.tab; renderSheet(); }
  else if (a === "use") useRef(S.sheet.slotId, t.dataset.ref);
  else if (a === "fit") {
    const u = S.upload, dur = u.data.length / TARGET_RATE;
    u.end = Math.min(dur, u.start + maxSecondsForSheet());
    if (u.end - u.start < 0.05) u.start = Math.max(0, u.end - 0.05);
    stopPlayback(); updateTrimUI();
  }
  else if (a === "upload-go") uploadGo();
  else if (a === "retry") { S.phase = "loading"; render(); loadState(); }
  else if (a === "toggle-enabled") {
    await guard(async () => {
      const r = await api("/api/enabled", { method: "PUT", json: { enabled: !S.data.enabled } });
      S.data.enabled = r.enabled;
      render();
      toast(r.enabled ? "Custom sounds are on" : "Custom sounds are off. Built-in sounds will play.");
    });
  } else if (a === "delete") {
    if (!t.classList.contains("armed")) {
      t.classList.add("armed");
      t.dataset.label = t.textContent;
      t.textContent = "Confirm";
      setTimeout(() => { if (t.isConnected) { t.classList.remove("armed"); t.textContent = t.dataset.label; } }, 3000);
      return;
    }
    await guard(async () => {
      await api(`/api/library/${encodeURIComponent(t.dataset.id)}`, { method: "DELETE" });
      await loadState();
      toast("Sound deleted");
    });
  }
});

document.addEventListener("input", (e) => {
  if (e.target.id === "cat-q") {
    S.sheet.query = e.target.value;
    const slot = slotById(S.sheet.slotId);
    $("#cat-list").innerHTML = renderCatalog(slot);
    setPlayingUI();
  } else if (e.target.id === "up-name" && S.upload) { S.upload.name = e.target.value; updateTrimUI(); }
});

document.addEventListener("change", (e) => {
  if (e.target.id === "file") handleFile(e.target.files[0]);
  else if (e.target.id === "up-norm" && S.upload) S.upload.normalize = e.target.checked;
});

document.addEventListener("toggle", (e) => {
  const d = e.target;
  if (d.classList && d.classList.contains("pack") && S.sheet && !S.sheet.query) {
    if (d.open) S.sheet.open.add(d.dataset.pack); else S.sheet.open.delete(d.dataset.pack);
  }
}, true);

document.addEventListener("submit", (e) => {
  if (e.target.id !== "token-form") return;
  e.preventDefault();
  const v = $("#token-input").value.trim();
  if (!v) return;
  setToken(v);
  S.error = "";
  S.phase = "loading";
  render();
  loadState().then(() => { if (S.phase === "auth") { S.error = "That access token was not accepted."; render(); } });
});

window.addEventListener("resize", () => { if (S.upload && S.upload.data) { S.upload.cols = null; drawTrim(); } });
window.matchMedia("(prefers-color-scheme: dark)").addEventListener("change", () => { if (S.upload && S.upload.data) drawTrim(); });

document.addEventListener("visibilitychange", () => {
  if (document.visibilityState === "visible" && S.phase === "ready" && !S.sheet) loadState();
});
setInterval(() => { if (S.phase === "ready" && !S.sheet && document.visibilityState === "visible") loadState(); }, 20000);

// ---------- boot ----------
async function boot() {
  render();
  if (MOCK) rawFetch = (await import("./mock.js")).mockFetch;
  else if ("serviceWorker" in navigator && (location.protocol === "https:" || location.hostname === "localhost")) {
    navigator.serviceWorker.register("sw.js").catch(() => {});
  }
  await loadState();
}
boot();
