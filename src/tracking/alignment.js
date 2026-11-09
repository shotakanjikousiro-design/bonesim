// =====================================================================
//  alignment.js — fit a camera so the 3D skeleton overlays the video.
//
//  The skeleton that the renderer draws is stickFK(pose, dims) — i.e. the rig
//  posed by the Pose INCLUDING pose.root.quaternion, pelvis centre at the
//  origin, in metres, in the "BoneSim camera frame" (+X image right,
//  +Y image up, +Z toward the camera; see angles.js). We fit, by weighted
//  least squares over visible shoulders/elbows/wrists/hips/knees/ankles,
//
//   ORTHOGRAPHIC (weak perspective, default, robust):
//      u = s*X + tx,   v = -s*Y + ty         (u,v = pixels of the DECODED video
//                                              frame, v downward)
//   PERSPECTIVE (pin-hole, assumed vertical FOV, camera at origin looking -Z):
//      u = cx + f*(X+Tx)/D, v = cy - f*(Y+Ty)/D, D = -(Z+Tz)
//      (linear least squares in Tx,Ty,Tz for fixed f)
//
//  RENDERER RECIPE (three.js):
//   Orthographic:
//     const cam = new THREE.OrthographicCamera(0, W, 0, -H, -10000, 10000); // W,H = alignment.width/height
//     cam.position.set(0, 0, 0);            // default orientation (looks down -Z)
//     skeletonRoot.matrixAutoUpdate = false;
//     skeletonRoot.matrix.fromArray(alignment.matrix);   // column-major 4x4
//     // skeletonRoot's children = rig posed with the Pose INCLUDING pose.root
//     // rotation (pelvis at local origin). If your rig ignores pose.root, use
//     // alignment.matrixWithRoot instead (it also contains the root rotation).
//   Perspective:
//     const cam = new THREE.PerspectiveCamera(alignment.perspective.fovY, W / H, 0.05, 50);
//     skeletonRoot.matrix.fromArray(alignment.perspective.matrix);  // (or matrixWithRoot)
//   Canvas: same pixel aspect and CSS box as the <video>. If the video is shown
//   with object-fit:cover (cropped), set the ortho camera to the visible crop:
//   left=cropX, right=cropX+cropW, top=-cropY, bottom=-(cropY+cropH) (source px).
//   Selfie/mirror display: if you mirror the <video> with CSS (scaleX(-1)),
//   mirror the overlay canvas the same way — the matrices are always expressed
//   in the decoded (un-CSS'd) frame. When options.inputMirrored is set
//   (frames arrive already mirrored), alignment.flipX = true and the matrices
//   contain the x-reflection (negative determinant; three.js handles winding).
// =====================================================================
import { stickFK, SIDES } from '../shared/conventions.js';
import { quatToMat } from './math3.js';

export const ALIGN_JOINTS = [
  ['shoulder', 11, 12, 1.0], ['elbow', 13, 14, 0.8], ['wrist', 15, 16, 0.6],
  ['hip', 23, 24, 1.0], ['knee', 25, 26, 0.8], ['ankle', 27, 28, 0.6],
];

/** model points (BoneSim camera frame, metres) for the ALIGN_JOINTS from a posed stick figure */
export function modelPointsFromFK(pose, dims) {
  const { positions } = stickFK(pose, dims);
  const out = [];
  for (const [name, li, ri, w] of ALIGN_JOINTS) for (const side of SIDES) {
    const p = positions[`${name}_${side}`];
    out.push({ p: [p.x, p.y, p.z], lm: side === 'left' ? li : ri, w });
  }
  return out;
}

/** weighted LSQ for u = s X + tx, v = -s Y + ty */
export function fitOrthographic(model, image, weights) {
  let sw = 0, mx = 0, my = 0, mu = 0, mv = 0;
  for (let i = 0; i < model.length; i++) {
    const w = weights[i]; if (!(w > 0)) continue;
    sw += w; mx += w * model[i][0]; my += w * model[i][1]; mu += w * image[i][0]; mv += w * image[i][1];
  }
  if (sw <= 0) return null;
  mx /= sw; my /= sw; mu /= sw; mv /= sw;
  let num = 0, den = 0;
  for (let i = 0; i < model.length; i++) {
    const w = weights[i]; if (!(w > 0)) continue;
    const X = model[i][0] - mx, Y = model[i][1] - my, U = image[i][0] - mu, Vv = image[i][1] - mv;
    num += w * (X * U - Y * Vv); den += w * (X * X + Y * Y);
  }
  if (den < 1e-12) return null;
  const s = num / den;
  const tx = mu - s * mx, ty = mv + s * my;
  let err = 0;
  for (let i = 0; i < model.length; i++) {
    const w = weights[i]; if (!(w > 0)) continue;
    err += w * ((s * model[i][0] + tx - image[i][0]) ** 2 + (-s * model[i][1] + ty - image[i][1]) ** 2);
  }
  return { s, tx, ty, rms: Math.sqrt(err / sw) };
}

/** perspective translation for fixed focal length f (pixels), principal point (cx,cy) */
export function fitPerspective(model, image, weights, f, cx, cy) {
  // rows:  -(u-cx) Tz - f Tx = f X + (u-cx) Z ;   -(v-cy) Tz + f Ty = -f Y + (v-cy) Z
  const A = [[0, 0, 0], [0, 0, 0], [0, 0, 0]], b = [0, 0, 0];
  const addRow = (r, rhs, w) => { for (let i = 0; i < 3; i++) { b[i] += w * r[i] * rhs; for (let j = 0; j < 3; j++) A[i][j] += w * r[i] * r[j]; } };
  let n = 0;
  for (let i = 0; i < model.length; i++) {
    const w = weights[i]; if (!(w > 0)) continue; n++;
    const [X, Y, Z] = model[i]; const du = image[i][0] - cx, dv = image[i][1] - cy;
    addRow([-f, 0, -du], f * X + du * Z, w);
    addRow([0, f, -dv], -f * Y + dv * Z, w);
  }
  if (n < 3) return null;
  const T = solve3(A, b); if (!T) return null;
  return { T };
}
function solve3(A, b) {
  const [a, bb, c] = A[0], [d, e, f] = A[1], [g, h, i] = A[2];
  const det = a * (e * i - f * h) - bb * (d * i - f * g) + c * (d * h - e * g);
  if (Math.abs(det) < 1e-12) return null;
  const inv = [
    [(e * i - f * h), -(bb * i - c * h), (bb * f - c * e)],
    [-(d * i - f * g), (a * i - c * g), -(a * f - c * d)],
    [(d * h - e * g), -(a * h - bb * g), (a * e - bb * d)],
  ];
  return [0, 1, 2].map((r) => (inv[r][0] * b[0] + inv[r][1] * b[1] + inv[r][2] * b[2]) / det);
}

/** column-major 4x4 from 3x3 linear part (row-major) + translation */
function mat4(L, t) {
  return [L[0], L[3], L[6], 0, L[1], L[4], L[7], 0, L[2], L[5], L[8], 0, t[0], t[1], t[2], 1];
}
const mul3 = (a, b) => { const r = []; for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) r.push(a[i * 3] * b[j] + a[i * 3 + 1] * b[3 + j] + a[i * 3 + 2] * b[6 + j]); return r; };

/**
 * @param pose        stabilised Pose (with root)
 * @param dims        dims used for the rig / stickFK
 * @param landmarks2D 33 normalized landmarks of the (un-mirrored-corrected) frame {x,y,visibility}
 * @param width,height decoded video size in pixels
 * @param inputMirrored true if frames arrived mirrored (landmarks2D were un-mirrored)
 * @param fovY        assumed vertical field of view for the perspective variant
 */
export function computeAlignment({ pose, dims, landmarks2D, width, height, inputMirrored = false, fovY = 50, minVisibility = 0.5, margin = 0.02 }) {
  if (!pose || !landmarks2D) return null;
  const mp = modelPointsFromFK(pose, dims);
  const model = [], image = [], w = [];
  for (const { p, lm, w: wj } of mp) {
    const l = landmarks2D[lm]; if (!l) continue;
    const vis = Math.min(l.visibility ?? 1, l.presence ?? 1);
    // only joints that are visible AND inside the image: MediaPipe extrapolates joints
    // outside the frame (hips/legs in a close-up) with deceptively high visibility
    const inside = l.x >= -margin && l.x <= 1 + margin && l.y >= -margin && l.y <= 1 + margin;
    model.push(p); image.push([l.x * width, l.y * height]);
    w.push(vis >= minVisibility && inside ? wj * vis : 0);
  }
  const nFit = w.filter((x) => x > 0).length;
  if (nFit < 2) return null;
  const o = fitOrthographic(model, image, w);
  if (!o) return null;
  const flip = inputMirrored ? -1 : 1;
  const F = [flip, 0, 0, 0, 1, 0, 0, 0, 1];
  const S = [o.s, 0, 0, 0, o.s, 0, 0, 0, o.s];
  const txd = inputMirrored ? width - o.tx : o.tx;
  const R = pose.root?.quaternion ? quatToMat(pose.root.quaternion) : [1, 0, 0, 0, 1, 0, 0, 0, 1];
  const ortho = { scale: o.s, tx: txd, ty: o.ty, rmsPx: o.rms };
  const matrix = mat4(mul3(F, S), [txd, -o.ty, 0]);
  const matrixWithRoot = mat4(mul3(mul3(F, S), R), [txd, -o.ty, 0]);

  // perspective variant
  const f = (height / 2) / Math.tan((fovY * Math.PI / 180) / 2);
  let persp = nFit >= 3 ? fitPerspective(model, image, w, f, width / 2, height / 2) : null;
  if (!persp || !(persp.T[2] < -0.1)) {
    // fall back to the weak-perspective equivalent: depth = f/s
    const D = f / o.s;
    persp = { T: [(o.tx - width / 2) / o.s, -(o.ty - height / 2) / o.s, -D] };
  }
  const T = persp.T.slice(); if (inputMirrored) T[0] = -T[0];
  const perspective = {
    fovY, focalPx: f, position: T,
    matrix: mat4(F, T), matrixWithRoot: mat4(mul3(F, R), T),
  };
  return {
    width, height, flipX: inputMirrored,
    ortho, matrix, matrixWithRoot, perspective,
    rootQuaternion: pose.root?.quaternion?.slice() ?? [0, 0, 0, 1],
    nFit,
    valid: Number.isFinite(o.s) && o.s > 0,
  };
}

/** project a point given in skeleton-local coordinates (pose incl. root) to decoded-video pixels (ortho) */
export function projectOrtho(alignment, p) {
  const m = alignment.matrix;
  const x = m[0] * p[0] + m[4] * p[1] + m[8] * p[2] + m[12];
  const y = m[1] * p[0] + m[5] * p[1] + m[9] * p[2] + m[13];
  return [x, -y];
}
/** perspective projection to decoded-video pixels */
export function projectPerspective(alignment, p) {
  const m = alignment.perspective.matrix, f = alignment.perspective.focalPx;
  const x = m[0] * p[0] + m[4] * p[1] + m[8] * p[2] + m[12];
  const y = m[1] * p[0] + m[5] * p[1] + m[9] * p[2] + m[13];
  const z = m[2] * p[0] + m[6] * p[1] + m[10] * p[2] + m[14];
  return [alignment.width / 2 + f * x / -z, alignment.height / 2 - f * y / -z];
}

// ---------------------------------------------------------------------
//  Image-plane refinement of the 3D landmarks.
//  MediaPipe's 2D (image) landmarks are much better aligned with the video
//  than its 3D world landmarks (which come from a separate regression head and
//  can disagree with the image by several cm, especially for truncated bodies).
//  We fit a pin-hole camera translation T to the world landmarks (fixed fovY),
//  then, for each visible landmark, keep the world DEPTH but take X/Y from the
//  2D landmark back-projected at that depth:
//     D = -(Z + Tz),  X = (u - cx) D / f - Tx,  Y = -(v - cy) D / f - Ty
//  Result: bone directions as seen by the camera match the video (better
//  overlay + in-plane angles), depth-related angles still come from MediaPipe z.
// ---------------------------------------------------------------------
export const REFINE_LANDMARKS = [0, 2, 5, 7, 8, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32];
const FIT_LANDMARKS = [11, 12, 13, 14, 15, 16, 23, 24, 25, 26, 27, 28];

/** pin-hole translation that best maps body points (BoneSim camera frame) onto the 2D landmarks */
export function fitWorldToImage(points, vis, lm2d, width, height, fovY = 50, minVis = 0.5) {
  const f = (height / 2) / Math.tan((fovY * Math.PI / 180) / 2);
  const model = [], image = [], w = [];
  for (const i of FIT_LANDMARKS) {
    const l = lm2d[i]; if (!l || !points[i]) continue;
    const v = Math.min(vis?.[i] ?? 1, l.visibility ?? 1);
    model.push(points[i]); image.push([l.x * width, l.y * height]); w.push(v >= minVis ? v : 0);
  }
  if (w.filter((x) => x > 0).length < 4) return null;
  const p = fitPerspective(model, image, w, f, width / 2, height / 2);
  if (!p || !(p.T[2] < -0.3)) return null;
  return { T: p.T, f };
}

/** @returns new points array (input not modified); weight 0..1 blends world -> image-refined X/Y */
export function refineWithImage(points, vis, lm2d, width, height, { T, f }, weight = 1, minVis = 0.3) {
  const cx = width / 2, cy = height / 2;
  const out = points.map((p) => p.slice());
  for (const i of REFINE_LANDMARKS) {
    const l = lm2d[i], p = points[i]; if (!l || !p) continue;
    const v = Math.min(vis?.[i] ?? 1, l.visibility ?? 1);
    if (v < minVis) continue;
    const a = weight * Math.min(1, (v - minVis) / (0.7 - minVis));
    const D = -(p[2] + T[2]); if (!(D > 0.1)) continue;
    const X = (l.x * width - cx) * D / f - T[0];
    const Y = -(l.y * height - cy) * D / f - T[1];
    out[i][0] = p[0] + (X - p[0]) * a;
    out[i][1] = p[1] + (Y - p[1]) * a;
  }
  // keep the hip midpoint at the origin (MediaPipe world convention)
  const o = [(out[23][0] + out[24][0]) / 2, (out[23][1] + out[24][1]) / 2, (out[23][2] + out[24][2]) / 2];
  for (const q of out) { q[0] -= o[0]; q[1] -= o[1]; q[2] -= o[2]; }
  return out;
}

/**
 * Estimate the camera's vertical FOV from one frame: the perspective model
 * u = cx + f (X+Tx)/D is fitted for candidate f, and the f with the lowest
 * reprojection residual wins (observable through the depth variation of the
 * body). Noisy per frame -> the pipeline keeps a running median.
 * @returns {fovY, informative} or null
 */
export function estimateFovY(points, vis, lm2d, width, height, { min = 28, max = 100, minVis = 0.6 } = {}) {
  const model = [], image = [], w = [];
  for (const i of FIT_LANDMARKS) {
    const l = lm2d[i]; if (!l || !points[i]) continue;
    const v = Math.min(vis?.[i] ?? 1, l.visibility ?? 1);
    if (v < minVis) continue;
    model.push(points[i]); image.push([l.x * width, l.y * height]); w.push(v);
  }
  if (model.length < 6) return null;
  const cost = (fov) => {
    const f = (height / 2) / Math.tan((fov * Math.PI / 180) / 2);
    const p = fitPerspective(model, image, w, f, width / 2, height / 2);
    if (!p || !(p.T[2] < -0.3)) return Infinity;
    let e = 0;
    for (let k = 0; k < model.length; k++) {
      const [X, Y, Z] = model[k], D = -(Z + p.T[2]);
      e += w[k] * ((width / 2 + f * (X + p.T[0]) / D - image[k][0]) ** 2 + (height / 2 - f * (Y + p.T[1]) / D - image[k][1]) ** 2);
    }
    return e;
  };
  let best = min, bestE = Infinity;
  for (let fov = min; fov <= max; fov += 3) { const e = cost(fov); if (e < bestE) { bestE = e; best = fov; } }
  for (let fov = Math.max(min, best - 3); fov <= Math.min(max, best + 3); fov += 0.5) { const e = cost(fov); if (e < bestE) { bestE = e; best = fov; } }
  if (!Number.isFinite(bestE)) return null;
  // informative if the residual rises clearly away from the optimum (enough depth variation)
  const eLo = cost(Math.max(min, best * 0.75)), eHi = cost(Math.min(max, best * 1.3));
  const informative = Math.min(eLo, eHi) > bestE * 1.15 + 1e-6 && best > min && best < max;
  return { fovY: best, informative };
}
