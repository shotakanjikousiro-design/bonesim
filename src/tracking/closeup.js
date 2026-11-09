// =====================================================================
//  closeup.js — support for close-up framings (face / head-and-shoulders /
//  hand filling the view), where the full-body assumptions break:
//   * hips/legs are outside the image but MediaPipe still extrapolates them
//     (often with visibility 0.3–0.9) → they must not drive the overlay fit;
//   * strong perspective (face much closer than the shoulders) → a single
//     weak-perspective scale cannot fit both;
//   * the head must be fitted to the FACE (eyes / ears / nose / mouth), not
//     hung at the end of an assumed neck.
//
//  Everything here is pure (no MediaPipe, no three.js) and works in the
//  "BoneSim camera frame" (+X image right, +Y up, +Z toward the camera,
//  metres) of angles.js / alignment.js.
// =====================================================================
import { fitPerspective } from './alignment.js';
import { apply, quatToMat, matToQuat } from './math3.js';

/** a landmark counts as "seen" if visible enough AND inside the image (small margin) */
export function landmarkOk(l, minVis = 0.6, margin = 0.01) {
  if (!l) return false;
  const v = Math.min(l.visibility ?? 1, l.presence ?? 1);
  return v >= minVis && l.x >= -margin && l.x <= 1 + margin && l.y >= -margin && l.y <= 1 + margin;
}

export const FRAMINGS = {
  full: { label: '全身' }, upper: { label: '上半身' }, headShoulders: { label: '顔・肩' },
  face: { label: '顔' }, hand: { label: '手' }, none: { label: '—' },
};

/**
 * Classify the framing of one frame.
 * @param lm2d   33 normalized pose landmarks (un-mirrored) or null
 * @param hands  [{landmarks2D: 21 normalized}] (any side) or []
 * @returns one of FRAMINGS' keys
 */
export function classifyFraming(lm2d, hands = []) {
  const ok = (i) => landmarkOk(lm2d?.[i]);
  const shoulders = ok(11) && ok(12);
  const hips = ok(23) && ok(24);
  const legs = (ok(27) || ok(28)) && (ok(25) || ok(26));
  const face = ok(0) && ((ok(2) && ok(5)) || (ok(7) && ok(8)) || (ok(2) && ok(7)) || (ok(5) && ok(8)));
  // a hand that fills a large part of the view dominates the framing
  let handSize = 0;
  for (const h of hands || []) {
    const L = h.landmarks2D; if (!L) continue;
    const inside = L.filter((p) => p.x >= 0 && p.x <= 1 && p.y >= 0 && p.y <= 1);
    if (inside.length < 15) continue; // hand mostly outside the image
    let x0 = 1, x1 = 0, y0 = 1, y1 = 0;
    for (const p of inside) { x0 = Math.min(x0, p.x); x1 = Math.max(x1, p.x); y0 = Math.min(y0, p.y); y1 = Math.max(y1, p.y); }
    handSize = Math.max(handSize, Math.hypot(x1 - x0, y1 - y0));
  }
  if (handSize > 0.45 || (handSize > 0 && !shoulders && !face)) return 'hand';
  if (shoulders && hips && legs) return 'full';
  if (shoulders && hips) return 'upper';
  if (shoulders) return 'headShoulders';
  if (face) return 'face';
  return 'none';
}

/** hysteresis: a new framing must persist `frames` consecutive frames before it is adopted */
export function createFramingTracker({ frames = 5 } = {}) {
  let cur = null, cand = null, n = 0;
  return {
    update(f) {
      if (cur === null) { cur = f; return cur; }
      if (f === cur) { cand = null; n = 0; return cur; }
      if (f === cand) { if (++n >= frames) { cur = f; cand = null; n = 0; } } else { cand = f; n = 1; }
      return cur;
    },
    get current() { return cur; },
    reset() { cur = cand = null; n = 0; },
  };
}

/**
 * Average adult face anchors relative to the ATLANTO-OCCIPITAL joint centre (the
 * rig's skull pivot), in the neutral head frame (metres, BoneSim axes). Derived
 * from the BodyParts3D skull in assets/rig.json: eyes ≈ 1 cm below / 0.8 cm behind
 * nasion, ±3.2 cm; ears (tragus) at the Frankfort level, skull half-width + 1 cm
 * soft tissue (bitragion ≈ 14 cm, cf. adult anthropometric mean ≈ 14.5 cm);
 * mouth corners ±2.5 cm, 3.5 cm below the nose tip.
 * Keys = MediaPipe pose landmark indices.
 */
export const HEAD_MODEL = {
  0: [0.0068, 0.0067, 0.1294],    // nose
  2: [0.0390, 0.0404, 0.0915],    // left eye
  5: [-0.0250, 0.0404, 0.0915],   // right eye
  7: [0.0765, 0.0250, 0.0100],    // left ear
  8: [-0.0645, 0.0250, 0.0100],   // right ear
  9: [0.0320, -0.0283, 0.1093],   // mouth left
  10: [-0.0180, -0.0283, 0.1093], // mouth right
};
const HEAD_W = { 0: 0.6, 2: 1, 5: 1, 7: 1, 8: 1, 9: 0.5, 10: 0.5 };

/**
 * Fit the skull to the face: known head orientation R (3×3 row-major, camera
 * frame), unknown position of the AO joint T — linear perspective LSQ on the
 * visible face landmarks. Depth therefore comes from the face's apparent size
 * (average adult head), independent of the body fit.
 * @returns {ao:[x,y,z] camera frame, rmsPx, n} or null
 */
export function fitHead(lm2d, R, width, height, f, { minVis = 0.5, model = HEAD_MODEL, refine = true } = {}) {
  const anchors = [], img = [], w = [];
  for (const [k, a] of Object.entries(model)) {
    const l = lm2d?.[k]; if (!landmarkOk(l, minVis, 0.02)) continue;
    anchors.push(a); img.push([l.x * width, l.y * height]); w.push(HEAD_W[k] ?? 0.5);
  }
  if (anchors.length < 3) return null;
  const solve = (Rm) => {
    const pts = anchors.map((a) => apply(Rm, a));
    const p = fitPerspective(pts, img, w, f, width / 2, height / 2);
    if (!p || !(p.T[2] < -0.08)) return null;
    let e = 0, sw = 0;
    for (let i = 0; i < pts.length; i++) {
      const X = pts[i][0] + p.T[0], Y = pts[i][1] + p.T[1], D = -(pts[i][2] + p.T[2]);
      e += w[i] * ((width / 2 + f * X / D - img[i][0]) ** 2 + (height / 2 - f * Y / D - img[i][1]) ** 2); sw += w[i];
    }
    return { ao: p.T, rmsPx: Math.sqrt(e / sw), R: Rm };
  };
  let best = solve(R); if (!best) return null;
  // The head orientation from MediaPipe's 3-D (world) face points is noisy in depth (yaw/pitch);
  // the 2-D layout of eyes / nose / mouth / ears pins it down well. Refine yaw & pitch (camera
  // frame, ±30° / ±20°) by the re-projection error, with a weak prior toward R.
  if (refine && anchors.length >= 5) {
    const ref = best.rmsPx;
    const cost = (r) => r.rmsPx * r.rmsPx + 0.02 * ref * ref * (r.dy * r.dy + r.dp * r.dp) / 100;
    let bc = cost({ ...best, dy: 0, dp: 0 }), by = 0, bp = 0;
    const tryAt = (dy, dp) => {
      const Rm = mulRot(rotYm(dy), mulRot(rotXm(dp), R)); const r = solve(Rm); if (!r) return;
      const c = cost({ ...r, dy, dp }); if (c < bc) { bc = c; best = r; by = dy; bp = dp; }
    };
    for (let dy = -30; dy <= 30; dy += 5) for (let dp = -20; dp <= 20; dp += 5) tryAt(dy, dp);
    const [cy, cp] = [by, bp];
    for (let dy = cy - 4; dy <= cy + 4; dy += 1) for (let dp = cp - 4; dp <= cp + 4; dp += 1) tryAt(dy, dp);
  }
  return { ao: best.ao, rmsPx: best.rmsPx, n: anchors.length, R: best.R, quaternion: matToQuat(best.R) };
}
const rotYm = (d) => { const c = Math.cos(d * Math.PI / 180), s = Math.sin(d * Math.PI / 180); return [c, 0, s, 0, 1, 0, -s, 0, c]; };
const rotXm = (d) => { const c = Math.cos(d * Math.PI / 180), s = Math.sin(d * Math.PI / 180); return [1, 0, 0, 0, c, -s, 0, s, c]; };
function mulRot(a, b) { const r = []; for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) r.push(a[3 * i] * b[j] + a[3 * i + 1] * b[3 + j] + a[3 * i + 2] * b[6 + j]); return r; }

/**
 * Place the 21 hand world landmarks (metric, hand-centred, BoneSim axes) in the
 * camera frame: perspective translation fitted to the 2D hand landmarks, then
 * X/Y of every point re-projected exactly onto its 2D landmark at its depth.
 * @returns 21 × [x,y,z] (camera frame) or null
 */
export function handToCamera(worldPts, lm2d, width, height, f) {
  if (!worldPts || !lm2d) return null;
  const img = lm2d.map((l) => [l.x * width, l.y * height]);
  const p = fitPerspective(worldPts, img, worldPts.map(() => 1), f, width / 2, height / 2);
  if (!p || !(p.T[2] < -0.05)) return null;
  return worldPts.map((q, i) => {
    const D = -(q[2] + p.T[2]); const d = Math.max(0.03, D);
    return [(img[i][0] - width / 2) * d / f, -(img[i][1] - height / 2) * d / f, q[2] + p.T[2]];
  });
}

/** One Euro-free simple quaternion smoother (sign-aligned exponential) for the head frame */
export function createQuatSmoother(alpha = 0.5) {
  let q = null;
  return (m) => {
    let n = matToQuat(m);
    if (q && n[0] * q[0] + n[1] * q[1] + n[2] * q[2] + n[3] * q[3] < 0) n = n.map((v) => -v);
    q = q ? q.map((v, i) => v + (n[i] - v) * alpha) : n;
    const l = Math.hypot(...q); q = q.map((v) => v / l);
    return { q: q.slice(), R: quatToMat(q) };
  };
}

/** default vertical FOV for a frame: the LONG image side is assumed to span ≈ 70° (webcams / phones) */
export function defaultFovY(width, height, longSideFov = 70) {
  const f = (Math.max(width, height) / 2) / Math.tan((longSideFov * Math.PI / 180) / 2);
  return 2 * Math.atan((height / 2) / f) * 180 / Math.PI;
}
