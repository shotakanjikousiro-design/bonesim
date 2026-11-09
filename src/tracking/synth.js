// =====================================================================
//  synth.js — synthesise MediaPipe-format results from a BoneSim Pose via
//  conventions.stickFK. Used by tests/tracking.test.mjs (round trip) and by
//  tracking-debug.html's "synthetic" source (works without camera/models).
//  Output format mirrors PoseLandmarker/HandLandmarker results:
//    { landmarks: 33 normalized {x,y,z,visibility,presence},
//      worldLandmarks: 33 {x,y,z,visibility,presence} (MediaPipe axes: x right, y down, z away; hip-mid origin),
//      hands: [{landmarks:21 normalized, worldLandmarks:21, handedness:[{categoryName,score}]}] }
// =====================================================================
import { stickFK, SIDES, sideSign, DEFAULT_DIMS } from '../shared/conventions.js';
import { bodyToMp, synthHand, mirrorPoseLandmarks, mirrorHandLandmarks, DEFAULT_ANGLE_OPTIONS } from './angles.js';
import { quatToMat, apply, add, mid, D2R } from './math3.js';

const v3 = (p) => [p.x, p.y, p.z];

/** gaussian random with a seeded generator */
export function makeRng(seed = 1) {
  let s = seed >>> 0 || 1;
  const u = () => { s ^= s << 13; s >>>= 0; s ^= s >> 17; s ^= s << 5; s >>>= 0; return (s + 0.5) / 4294967296; };
  return { uniform: u, gauss: () => Math.sqrt(-2 * Math.log(u())) * Math.cos(2 * Math.PI * u()) };
}

/**
 * @param pose  BoneSim Pose (root optional)
 * @param opts  {dims, neckPitchOffset, inputMirrored, noise (m, world), handNoise (m), pixelNoise (px), rng,
 *               width, height, camDistance (m), fovY, center:[u,v] normalized, withHands}
 */
export function synthesizeFrame(pose, opts = {}) {
  const {
    dims = DEFAULT_DIMS, neckPitchOffset = DEFAULT_ANGLE_OPTIONS.neckPitchOffset, inputMirrored = false,
    noise = 0, handNoise = 0, pixelNoise = 0, rng = makeRng(7), width = 640, height = 480,
    camDistance = 2.8, fovY = 50, center = [0.5, 0.55], withHands = true,
  } = opts;
  const { positions: Pp, quaternions: Q } = stickFK(pose, dims);
  const P = {}; for (const k in Pp) P[k] = v3(Pp[k]);
  const qa = (q) => quatToMat([q.x, q.y, q.z, q.w]);
  const Hh = qa(Q.head);
  const hy = dims.neck + dims.head * 0.45;
  const headPt = (x, y, z) => add(P.neckBase, apply(Hh, [x, y, z]));
  const eyeUp = 0.085 * Math.tan(neckPitchOffset * D2R);
  const pts = new Array(33);
  pts[0] = P.nose;
  for (const side of SIDES) {
    const s = sideSign(side), k = (n) => P[`${n}_${side}`];
    const L = side === 'left';
    pts[L ? 1 : 4] = headPt(s * 0.018, hy + eyeUp, 0.085);
    pts[L ? 2 : 5] = headPt(s * 0.032, hy + eyeUp, 0.085);
    pts[L ? 3 : 6] = headPt(s * 0.046, hy + eyeUp, 0.08);
    pts[L ? 7 : 8] = headPt(s * 0.075, hy, 0);
    pts[L ? 9 : 10] = headPt(s * 0.025, hy - 0.05, 0.09);
    pts[L ? 11 : 12] = k('shoulder'); pts[L ? 13 : 14] = k('elbow'); pts[L ? 15 : 16] = k('wrist');
    const Hd = qa(Q[`hand_${side}`]); const W = k('wrist');
    pts[L ? 17 : 18] = add(W, apply(Hd, [-s * 0.03, -0.09, 0]));   // pinky knuckle
    pts[L ? 19 : 20] = add(W, apply(Hd, [s * 0.03, -0.09, 0]));    // index knuckle
    pts[L ? 21 : 22] = add(W, apply(Hd, [s * 0.05, -0.05, 0.02])); // thumb
    pts[L ? 23 : 24] = k('hip'); pts[L ? 25 : 26] = k('knee'); pts[L ? 27 : 28] = k('ankle');
    pts[L ? 29 : 30] = k('heel'); pts[L ? 31 : 32] = k('toe');
  }
  const origin = mid(pts[23], pts[24]);
  const n = () => (noise ? rng.gauss() * noise : 0);
  // pin-hole camera at the origin of the BoneSim camera frame (principal point = image centre);
  // pelvis placed camDistance in front, shifted so it projects to `center` (normalized)
  const f = (height / 2) / Math.tan(fovY * D2R / 2);
  const off = [(center[0] - 0.5) * width * camDistance / f, -(center[1] - 0.5) * height * camDistance / f, -camDistance];
  const proj = (p) => {
    const X = p[0] - origin[0] + off[0], Y = p[1] - origin[1] + off[1], Z = p[2] - origin[2] + off[2];
    return { x: 0.5 + f * X / -Z / width + (pixelNoise ? rng.gauss() * pixelNoise / width : 0),
             y: 0.5 - f * Y / -Z / height + (pixelNoise ? rng.gauss() * pixelNoise / height : 0) };
  };
  let world = pts.map((p) => ({ ...bodyToMp([p[0] - origin[0] + n(), p[1] - origin[1] + n(), p[2] - origin[2] + n()]), visibility: 1, presence: 1 }));
  let landmarks = pts.map((p) => ({ ...proj(p), z: 0, visibility: 1, presence: 1 }));
  let hands = [];
  if (withHands) {
    for (const side of SIDES) {
      const Hd = qa(Q[`hand_${side}`]);
      const hp = synthHand(Hd, P[`wrist_${side}`], side, pose[side]?.fingers || {});
      const c = hp.reduce((a, p) => add(a, p), [0, 0, 0]).map((v) => v / hp.length);
      const hn = () => (handNoise ? rng.gauss() * handNoise : 0);
      let hw = hp.map((p) => ({ ...bodyToMp([p[0] - c[0] + hn(), p[1] - c[1] + hn(), p[2] - c[2] + hn()]) }));
      let hl = hp.map((p) => ({ ...proj(p), z: 0 }));
      // tasks-vision 1.x: label = subject's hand for an un-mirrored image (inverted by mirroring).
      // The pipeline never trusts the label; hands are assigned by wrist proximity.
      let label = side === 'left' ? 'Left' : 'Right';
      if (inputMirrored) { hw = mirrorHandLandmarks(hw, 'world'); hl = mirrorHandLandmarks(hl, 'normalized'); label = side === 'left' ? 'Right' : 'Left'; }
      hands.push({ landmarks: hl, worldLandmarks: hw, handedness: [{ categoryName: label, score: 0.98 }] });
    }
  }
  if (inputMirrored) { world = mirrorPoseLandmarks(world, 'world'); landmarks = mirrorPoseLandmarks(landmarks, 'normalized'); }
  return { landmarks, worldLandmarks: world, hands, width, height };
}
