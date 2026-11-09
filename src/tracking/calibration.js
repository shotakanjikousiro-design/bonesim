// =====================================================================
//  calibration.js — "stand still, face the camera, arms slightly away from
//  the body" capture (2–3 s) -> per-person segment lengths in the SAME keys
//  as conventions.DEFAULT_DIMS (+ height estimate, neutral head pitch,
//  gravity direction). Pure; frames are body points in the BoneSim camera
//  frame (angles.prepareBody), ideally already landmark-filtered but NOT
//  length-enforced.
// =====================================================================
import { DEFAULT_DIMS } from '../shared/conventions.js';
import { computePose } from './angles.js';
import { dist, mid, sub, dot, normalize, perp, median, len } from './math3.js';

const VIS = 0.5;

export function createCalibrator({ durationMs = 2500, minFrames = 15 } = {}) {
  const frames = []; let t0;
  return {
    durationMs,
    /** add one frame; returns progress 0..1 */
    add(frame, tMs) {
      if (t0 === undefined) t0 = tMs;
      if (frame) frames.push(frame);
      return Math.min(1, (tMs - t0) / durationMs);
    },
    get frameCount() { return frames.length; },
    isDone(tMs) { return t0 !== undefined && tMs - t0 >= durationMs && frames.length >= minFrames; },
    finish() { return estimateDims(frames); },
  };
}

/** per-frame segment measurements (NaN where the landmarks are not visible enough) */
export function measureFrame(f) {
  const ok = (...idx) => idx.every((i) => (f.vis?.[i] ?? 1) >= VIS);
  const P = (i) => f.points[i];
  const m = (cond, fn) => (cond ? fn() : NaN);
  const avg = (a, b) => (Number.isFinite(a) && Number.isFinite(b) ? (a + b) / 2 : Number.isFinite(a) ? a : b);
  const ankleH = (a, h, t) => { const ax = normalize(sub(P(t), P(h))); return len(perp(sub(P(a), P(h)), ax)); };
  const r = {
    hipHalfWidth: m(ok(23, 24), () => dist(P(23), P(24)) / 2),
    shoulderHalfWidth: m(ok(11, 12), () => dist(P(11), P(12)) / 2),
    thoraxHeight: m(ok(11, 12, 23, 24), () => dist(mid(P(11), P(12)), mid(P(23), P(24)))),
    upperArm: avg(m(ok(11, 13), () => dist(P(11), P(13))), m(ok(12, 14), () => dist(P(12), P(14)))),
    forearm: avg(m(ok(13, 15), () => dist(P(13), P(15))), m(ok(14, 16), () => dist(P(14), P(16)))),
    thigh: avg(m(ok(23, 25), () => dist(P(23), P(25))), m(ok(24, 26), () => dist(P(24), P(26)))),
    shank: avg(m(ok(25, 27), () => dist(P(25), P(27))), m(ok(26, 28), () => dist(P(26), P(28)))),
    footLength: avg(m(ok(29, 31), () => dist(P(29), P(31))), m(ok(30, 32), () => dist(P(30), P(32)))),
    ankleHeight: avg(m(ok(27, 29, 31), () => ankleH(27, 29, 31)), m(ok(28, 30, 32), () => ankleH(28, 30, 32))),
    _earAboveShoulders: m(ok(7, 8, 11, 12, 23, 24), () => {
      const sm = mid(P(11), P(12)), up = normalize(sub(sm, mid(P(23), P(24))));
      return dot(sub(mid(P(7), P(8)), sm), up);
    }),
  };
  // hand: wrist -> middle fingertip from hand landmarks
  const hl = [];
  for (const sd of ['left', 'right']) {
    const h = f.hands?.[sd]?.points; if (!h) continue;
    hl.push(dist(h[0], h[9]) + dist(h[9], h[10]) + dist(h[10], h[11]) + dist(h[11], h[12]));
  }
  r.hand = hl.length ? hl.reduce((a, b) => a + b) / hl.length : NaN;
  return r;
}

/** medians of measureFrame() results -> dims with plausibility checks & proportional defaults */
export function dimsFromMeasurements(list) {
  const measured = [], defaulted = [];
  const raw = {};
  for (const k of [...Object.keys(DEFAULT_DIMS), '_earAboveShoulders']) raw[k] = median(list.map((m) => m[k]));
  // overall scale relative to defaults from the most reliable torso measures
  const ratios = ['shoulderHalfWidth', 'thoraxHeight', 'upperArm', 'thigh']
    .filter((k) => Number.isFinite(raw[k])).map((k) => raw[k] / DEFAULT_DIMS[k]);
  const k = ratios.length ? median(ratios) : 1;
  const dims = {};
  for (const key of Object.keys(DEFAULT_DIMS)) {
    const v = raw[key];
    const plausible = Number.isFinite(v) && v > DEFAULT_DIMS[key] * k * 0.5 && v < DEFAULT_DIMS[key] * k * 1.8;
    if (plausible) { dims[key] = v; measured.push(key); } else { dims[key] = DEFAULT_DIMS[key] * k; defaulted.push(key); }
  }
  // hand from hand landmarks only; otherwise proportional to forearm
  if (!measured.includes('hand')) dims.hand = DEFAULT_DIMS.hand * (dims.forearm / DEFAULT_DIMS.forearm);
  dims.head = DEFAULT_DIMS.head * k;
  if (Number.isFinite(raw._earAboveShoulders)) {
    const n = raw._earAboveShoulders - 0.45 * dims.head;
    if (n > 0.02 && n < 0.25) { dims.neck = n; if (!measured.includes('neck')) measured.push('neck'); }
  }
  const height = dims.ankleHeight + dims.shank + dims.thigh + dims.thoraxHeight + dims.neck + dims.head;

  return { dims, height, quality: { scale: k, measured, defaulted } };
}

/**
 * @param frames [{points:[[x,y,z]]*33, vis:[], hands?:{left?:{points}, right?:{points}}}]
 * @returns {dims, height, neckPitchOffset, gravityUp, quality:{frames, stillnessMm, scale, measured:[], defaulted:[]}}
 */
export function estimateDims(frames) {
  const ok = (f, ...idx) => idx.every((i) => (f.vis?.[i] ?? 1) >= VIS);
  const P = (f, i) => f.points[i];
  const { dims, height, quality: q } = dimsFromMeasurements(frames.map(measureFrame));
  // gravity = mean trunk direction while standing upright
  let g = [0, 0, 0];
  for (const f of frames) if (ok(f, 11, 12, 23, 24)) {
    const u = normalize(sub(mid(P(f, 11), P(f, 12)), mid(P(f, 23), P(f, 24))));
    g = [g[0] + u[0], g[1] + u[1], g[2] + u[2]];
  }
  const gravityUp = len(g) > 0 ? normalize(g) : [0, 1, 0];

  // neutral head pitch: neck flexion measured with offset 0 -> offset = -median
  const neckF = frames.filter((f) => ok(f, 2, 5, 7, 8, 11, 12))
    .map((f) => computePose(f, { neckPitchOffset: 0, lumbopelvicRatio: 0, lumbopelvicExtensionRatio: 0 }).pose.neck.flexion);
  const neckPitchOffset = neckF.length ? -median(neckF) : undefined;

  // stillness: median per-frame displacement of torso+limb points (mm)
  const moves = [];
  for (let i = 1; i < frames.length; i++) {
    const a = frames[i - 1].points, b = frames[i].points; let s = 0, n = 0;
    for (const j of [11, 12, 13, 14, 15, 16, 23, 24, 25, 26]) { s += dist(a[j], b[j]); n++; }
    moves.push(s / n * 1000);
  }
  return {
    dims, height, neckPitchOffset, gravityUp,
    quality: { frames: frames.length, stillnessMm: median(moves), ...q },
  };
}

/**
 * Continuous (implicit) calibration used until the user runs calibrate():
 * rolling medians of segment lengths over frames where the segment is clearly
 * visible. MediaPipe world segment lengths are fairly stable, so this quickly
 * adapts the rig to the person (better overlay) without a calibration pose.
 */
export function createAutoDims({ window = 300, minFrames = 45, every = 15 } = {}) {
  const buf = []; let n = 0, cur = null;
  return {
    add(frame) {
      buf.push(measureFrame(frame)); if (buf.length > window) buf.shift();
      if (++n % every === 0 && buf.length >= minFrames) cur = dimsFromMeasurements(buf);
      return cur;
    },
    get ready() { return !!cur; },
    get current() { return cur; },
    reset() { buf.length = 0; n = 0; cur = null; },
  };
}
