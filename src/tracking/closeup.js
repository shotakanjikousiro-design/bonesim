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
import { apply, quatToMat, matToQuat, mul, transpose, axisAngle, normalize, cross, sub } from './math3.js';
import { FACE_ANCHORS, FACE_TO_SKULL, FACE_DEPTH_SCALE } from './faceModel.js';

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
 * @param hasFace the face tracker (FaceLandmarker) has a face
 * @returns one of FRAMINGS' keys
 */
export function classifyFraming(lm2d, hands = [], hasFace = false) {
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
  // a hand alone → 'hand'. With a face / shoulders in view the framing describes the body and the
  // hand bones are simply drawn on top (face and hand are shown together).
  if (handSize > 0 && !shoulders && !face && !hasFace) return 'hand';
  if (shoulders && hips && legs) return 'full';
  if (shoulders && hips) return 'upper';
  if (shoulders) return 'headShoulders';
  if (face || hasFace) return 'face';
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

// ---------------------------------------------------------------------
//  Head from the FACE tracker (MediaPipe FaceLandmarker, 478 landmarks):
//  far better constrained than the 7 face points of the pose model.
// ---------------------------------------------------------------------
const FACE_IDS = Object.keys(FACE_ANCHORS).map(Number);
const FACE_PTS = FACE_IDS.map((i) => FACE_ANCHORS[i]);
const SKULL_FROM_FACE_T = transpose(FACE_TO_SKULL);

/** rotation (row-major 3×3) of the canonical face from FaceLandmarker's 4×4 facial transformation matrix (column-major) */
export function faceMatrixRotation(m) {
  if (!m || m.length < 16) return null;
  const c = [0, 1, 2].map((j) => normalize([m[4 * j], m[4 * j + 1], m[4 * j + 2]]));
  if (c.some((v) => !Number.isFinite(v[0]) || (v[0] === 0 && v[1] === 0 && v[2] === 0))) return null;
  return [c[0][0], c[1][0], c[2][0], c[0][1], c[1][1], c[2][1], c[0][2], c[1][2], c[2][2]];
}
/** fallback orientation of the canonical face from the landmarks' own 3-D (x, y image; z ≈ image-width units) */
function faceRotationFromLandmarks(L, width, height) {
  const P = (i) => [L[i].x * width, -L[i].y * height, -(L[i].z || 0) * width];
  const x = normalize(sub(P(263), P(33))); const up = sub(P(10), P(152));
  const z = normalize(cross(x, up)); const y = cross(z, x);
  if (!(Math.hypot(...z) > 0.5)) return null;
  return [x[0], y[0], z[0], x[1], y[1], z[1], x[2], y[2], z[2]];
}

/**
 * Fit the skull to FaceLandmarker landmarks: orientation from the facial transformation matrix
 * (or the landmarks' 3-D), refined together with the position by the 2-D re-projection error of
 * FACE_ANCHORS (expression-stable points: forehead, eye corners, nose, cheekbones).
 * @param face {landmarks: 478 normalized (un-mirrored), matrix?: 16 numbers column-major}
 * @returns {ao, R, quaternion, rmsPx, n, depthScale} (camera frame) or null
 */
export function fitHeadFromFace(face, width, height, f) {
  const L = face?.landmarks; if (!L || L.length < 468) return null;
  const Rf = faceMatrixRotation(face.matrix) || faceRotationFromLandmarks(L, width, height); if (!Rf) return null;
  const img = [], pts = [], w = [];
  for (let k = 0; k < FACE_IDS.length; k++) {
    const l = L[FACE_IDS[k]]; if (!l || !Number.isFinite(l.x) || !Number.isFinite(l.y)) continue;
    img.push([l.x * width, l.y * height]); pts.push(FACE_PTS[k]); w.push(1);
  }
  if (pts.length < 12) return null;
  const solve = (R) => {
    const m = pts.map((a) => apply(R, a));
    const p = fitPerspective(m, img, w, f, width / 2, height / 2);
    if (!p || !(p.T[2] < -0.05)) return null;
    let e = 0;
    for (let i = 0; i < m.length; i++) {
      const D = -(m[i][2] + p.T[2]); if (!(D > 0.01)) return null;
      e += (width / 2 + f * (m[i][0] + p.T[0]) / D - img[i][0]) ** 2 + (height / 2 - f * (m[i][1] + p.T[1]) / D - img[i][1]) ** 2;
    }
    return { T: p.T, e: e / m.length, R };
  };
  let best = solve(mul(Rf, SKULL_FROM_FACE_T)); if (!best) return null;
  const AX = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
  for (const step of [4, 2, 1, 0.5]) {
    for (let pass = 0; pass < 2; pass++) {
      let improved = false;
      for (const ax of AX) for (const sg of [1, -1]) {
        const r = solve(mul(axisAngle(ax, sg * step), best.R));
        if (r && r.e < best.e * 0.995) { best = r; improved = true; }
      }
      if (!improved) break;
    }
  }
  return { ao: best.T, R: best.R, quaternion: matToQuat(best.R), rmsPx: Math.sqrt(best.e), n: pts.length, depthScale: FACE_DEPTH_SCALE };
}

/** mouth opening (mandible rotation, degrees) from the FaceLandmarker "jawOpen" blendshape (0..1) */
export function jawAngleFromBlendshape(jawOpen, maxDeg = 30) {
  if (!Number.isFinite(jawOpen)) return 0;
  return maxDeg * Math.min(1, Math.max(0, (jawOpen - 0.04) / 0.66));
}
