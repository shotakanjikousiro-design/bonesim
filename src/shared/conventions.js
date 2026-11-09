// =====================================================================
//  BoneSim shared conventions — THE single source of truth for
//  coordinate frames and joint-angle definitions.
//
//  Both the tracking module (camera -> Pose) and the anatomical rig
//  (Pose -> bone meshes) MUST agree with `stickFK()` below.
//  Tests: tests/conventions.test.mjs
// =====================================================================
//
//  WORLD / BODY FRAME (right-handed, metres)
//    +X = subject's LEFT,  +Y = UP,  +Z = FORWARD (anterior)
//    Rest pose = anatomical position: standing, arms at sides,
//    palms facing forward (forearm fully SUPINATED), feet forward.
//
//  SIDE SIGN  s = +1 for 'left', -1 for 'right'.  lateral = (s,0,0)
//
//  ALL ANGLES IN DEGREES. Composition is intrinsic: q = qA * qB means
//  "rotate by qA, then by qB about the already-rotated local axes".
//  Local segment frames equal the world axes in the rest pose.
//
//  Pose = {
//    root?:  { position:[x,y,z], quaternion:[x,y,z,w] }   // pelvis in world/camera
//    spine:  { flexion, lateralBend, axialRotation }       // thorax rel. pelvis
//    neck:   { flexion, lateralBend, axialRotation }       // head rel. thorax
//    left|right: {
//      shoulder: { elevation, plane, axialRotation }       // HUMERO-THORACIC
//      elbow:    { flexion }
//      forearm:  { pronation }        // 0 = neutral, -90 = anatomical (supinated)
//      wrist:    { flexion, ulnarDeviation }
//      fingers?: { thumb:{cmcFlexion,cmcAbduction,mcp,ip},
//                  index|middle|ring|little:{mcp,mcpAbduction,pip,dip} }
//      hip:      { flexion, abduction, axialRotation }
//      knee:     { flexion }
//      ankle:    { dorsiflexion, inversion }
//    }
//    confidence?: { [jointPath]: 0..1 }
//  }
//
//  Joint definitions (q = local rotation of the child segment):
//   spine/neck : q = Rx(+flex) * Rz(+bend) * Ry(+rot)
//                flexion + = bend forward; lateralBend + = toward subject's RIGHT;
//                axialRotation + = turn toward subject's LEFT.
//   shoulder   : swing–twist.  d(θ,φ) = -cosθ·Y + sinθ·(cosφ·lateral + sinφ·Z)
//                θ = elevation (0 = arm hanging), φ = plane of elevation
//                (0 = abduction in coronal plane, 90 = forward flexion,
//                 -90 = extension), ψ = axialRotation (+ = INTERNAL).
//                q = swing(-Y → d) * Ry(-s·ψ)
//   elbow      : q = Rx(-flex)          (flexion brings forearm anterior)
//   forearm    : q = Ry(-s·(pronation + 90))   applied to the hand/radius frame
//                (rest/anatomical = -90 → identity). Neutral (0) with arm hanging:
//                thumb points forward, palm faces the thigh.
//   wrist      : q = Rx(-flex) * Rz(-s·ulnarDev)   (palmar flexion +)
//   hip        : q = Rx(-flex) * Rz(+s·abd) * Ry(-s·intRot)
//   knee       : q = Rx(+flex)          (flexion brings shank posterior)
//   ankle      : q = Rx(-dorsi) * Rz(-s·inv)  (foot long axis = +Z)
//   fingers    : mcp/pip/dip/ip flexion = Rx(-flex) in the hand frame
//                (palm faces +Z at rest, so flexion curls toward +Z);
//                mcpAbduction + = away from middle finger (radial for index).
//
//  Range-of-motion limits follow the Japanese Orthopaedic Association /
//  Japanese Association of Rehabilitation Medicine standard (関節可動域表示
//  ならびに測定法) — see ROM below.
// =====================================================================
import * as THREE from 'three';

export const SIDES = ['left', 'right'];
export const sideSign = (side) => (side === 'left' ? 1 : -1);
const D2R = Math.PI / 180;

const qAxis = (x, y, z, deg) =>
  new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(x, y, z), deg * D2R);
export const Rx = (deg) => qAxis(1, 0, 0, deg);
export const Ry = (deg) => qAxis(0, 1, 0, deg);
export const Rz = (deg) => qAxis(0, 0, 1, deg);
const mul = (...qs) => qs.reduce((a, b) => a.multiply(b), new THREE.Quaternion());

// ---------------- Range of motion (degrees) [min, max] ----------------
export const ROM = {
  spine:    { flexion: [-30, 45], lateralBend: [-50, 50], axialRotation: [-40, 40] }, // thoracolumbar
  neck:     { flexion: [-50, 60], lateralBend: [-50, 50], axialRotation: [-60, 60] },
  shoulder: { elevation: [0, 180], plane: [-90, 135], axialRotation: [-60, 80] },  // ext rot 60 / int rot 80
  elbow:    { flexion: [-5, 145] },
  forearm:  { pronation: [-90, 90] },
  wrist:    { flexion: [-70, 90], ulnarDeviation: [-25, 55] },  // dorsiflex 70, palmar 90, radial 25, ulnar 55
  hip:      { flexion: [-15, 125], abduction: [-20, 45], axialRotation: [-45, 45] },
  knee:     { flexion: [0, 130] },
  ankle:    { dorsiflexion: [-45, 20], inversion: [-20, 30] },
  finger:   { mcp: [-45, 90], pip: [0, 100], dip: [0, 80], mcpAbduction: [-20, 20] },
  thumb:    { cmcFlexion: [-10, 50], cmcAbduction: [0, 70], mcp: [-10, 60], ip: [-10, 80] },
};

// ---------------- Joint rotation builders ----------------
export function spineQ({ flexion = 0, lateralBend = 0, axialRotation = 0 } = {}) {
  return mul(Rx(flexion), Rz(lateralBend), Ry(axialRotation));
}
export function humerusDirection(side, elevation, plane) {
  const s = sideSign(side), t = elevation * D2R, p = plane * D2R;
  return new THREE.Vector3(
    Math.sin(t) * Math.cos(p) * s,
    -Math.cos(t),
    Math.sin(t) * Math.sin(p),
  ).normalize();
}
export function shoulderQ(side, { elevation = 0, plane = 0, axialRotation = 0 } = {}) {
  const s = sideSign(side);
  const el = Math.min(elevation, 179.5);
  const d = humerusDirection(side, el, plane);
  const swing = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, -1, 0), d);
  return mul(swing, Ry(-s * axialRotation));
}
export const elbowQ = (side, { flexion = 0 } = {}) => Rx(-flexion);
export const forearmQ = (side, { pronation = -90 } = {}) => Ry(-sideSign(side) * (pronation + 90));
export const wristQ = (side, { flexion = 0, ulnarDeviation = 0 } = {}) =>
  mul(Rx(-flexion), Rz(-sideSign(side) * ulnarDeviation));
export const hipQ = (side, { flexion = 0, abduction = 0, axialRotation = 0 } = {}) => {
  const s = sideSign(side);
  return mul(Rx(-flexion), Rz(s * abduction), Ry(-s * axialRotation));
};
export const kneeQ = (side, { flexion = 0 } = {}) => Rx(flexion);
export const ankleQ = (side, { dorsiflexion = 0, inversion = 0 } = {}) =>
  mul(Rx(-dorsiflexion), Rz(-sideSign(side) * inversion));

// ---------------- Neutral / anatomical poses ----------------
export function anatomicalPose() {
  const limb = () => ({
    shoulder: { elevation: 0, plane: 0, axialRotation: 0 },
    elbow: { flexion: 0 },
    forearm: { pronation: -90 },
    wrist: { flexion: 0, ulnarDeviation: 0 },
    hip: { flexion: 0, abduction: 0, axialRotation: 0 },
    knee: { flexion: 0 },
    ankle: { dorsiflexion: 0, inversion: 0 },
  });
  return {
    spine: { flexion: 0, lateralBend: 0, axialRotation: 0 },
    neck: { flexion: 0, lateralBend: 0, axialRotation: 0 },
    left: limb(), right: limb(),
  };
}

// ---------------- Default stick-figure dimensions (m) ----------------
export const DEFAULT_DIMS = {
  hipHalfWidth: 0.09,        // pelvis centre -> hip joint centre (x)
  thoraxHeight: 0.50,        // pelvis centre -> shoulder line (y), in thorax frame
  shoulderHalfWidth: 0.18,   // thorax midline -> glenohumeral centre (x)
  neck: 0.10, head: 0.20,
  upperArm: 0.30, forearm: 0.26, hand: 0.19,
  thigh: 0.43, shank: 0.42, footLength: 0.20, ankleHeight: 0.07,
};

// ---------------- Reference stick-figure forward kinematics ----------------
// Returns world-space joint positions + segment frames (quaternions) for a Pose.
// Positions are relative to the pelvis centre (root) unless pose.root given.
export function stickFK(pose, dims = DEFAULT_DIMS) {
  const P = {}; const Q = {};
  const V = (x, y, z) => new THREE.Vector3(x, y, z);
  const rootQ = pose.root?.quaternion ? new THREE.Quaternion(...pose.root.quaternion) : new THREE.Quaternion();
  const rootP = pose.root?.position ? V(...pose.root.position) : V(0, 0, 0);
  const at = (parentP, parentQ, local) => local.clone().applyQuaternion(parentQ).add(parentP);

  Q.pelvis = rootQ.clone(); P.pelvis = rootP.clone();
  Q.thorax = Q.pelvis.clone().multiply(spineQ(pose.spine));
  P.neckBase = at(P.pelvis, Q.thorax, V(0, dims.thoraxHeight, 0));
  Q.head = Q.thorax.clone().multiply(spineQ(pose.neck));
  P.headTop = at(P.neckBase, Q.head, V(0, dims.neck + dims.head, 0));
  P.nose = at(P.neckBase, Q.head, V(0, dims.neck + dims.head * 0.45, 0.10));

  for (const side of SIDES) {
    const s = sideSign(side); const L = pose[side] || {}; const k = (n) => `${n}_${side}`;
    // upper limb
    P[k('shoulder')] = at(P.pelvis, Q.thorax, V(s * dims.shoulderHalfWidth, dims.thoraxHeight, 0));
    Q[k('humerus')] = Q.thorax.clone().multiply(shoulderQ(side, L.shoulder));
    P[k('elbow')] = at(P[k('shoulder')], Q[k('humerus')], V(0, -dims.upperArm, 0));
    Q[k('ulna')] = Q[k('humerus')].clone().multiply(elbowQ(side, L.elbow));
    P[k('wrist')] = at(P[k('elbow')], Q[k('ulna')], V(0, -dims.forearm, 0));
    Q[k('radius')] = Q[k('ulna')].clone().multiply(forearmQ(side, L.forearm));
    Q[k('hand')] = Q[k('radius')].clone().multiply(wristQ(side, L.wrist));
    P[k('handTip')] = at(P[k('wrist')], Q[k('hand')], V(0, -dims.hand, 0));
    // thumb side & palm normal (world) — useful for tracking tests
    P[k('thumbSide')] = at(P[k('wrist')], Q[k('hand')], V(s * 0.04, -0.08, 0));
    P[k('pinkySide')] = at(P[k('wrist')], Q[k('hand')], V(-s * 0.04, -0.08, 0));
    // lower limb
    P[k('hip')] = at(P.pelvis, Q.pelvis, V(s * dims.hipHalfWidth, 0, 0));
    Q[k('femur')] = Q.pelvis.clone().multiply(hipQ(side, L.hip));
    P[k('knee')] = at(P[k('hip')], Q[k('femur')], V(0, -dims.thigh, 0));
    Q[k('tibia')] = Q[k('femur')].clone().multiply(kneeQ(side, L.knee));
    P[k('ankle')] = at(P[k('knee')], Q[k('tibia')], V(0, -dims.shank, 0));
    Q[k('foot')] = Q[k('tibia')].clone().multiply(ankleQ(side, L.ankle));
    P[k('heel')] = at(P[k('ankle')], Q[k('foot')], V(0, -dims.ankleHeight, -0.05));
    P[k('toe')] = at(P[k('ankle')], Q[k('foot')], V(0, -dims.ankleHeight, dims.footLength - 0.05));
  }
  return { positions: P, quaternions: Q };
}

export function clampPose(pose) {
  const out = structuredClone(pose);
  const clampObj = (obj, lim) => { if (!obj) return; for (const k in lim) if (k in obj) obj[k] = Math.min(lim[k][1], Math.max(lim[k][0], obj[k])); };
  clampObj(out.spine, ROM.spine); clampObj(out.neck, ROM.neck);
  for (const side of SIDES) {
    const L = out[side]; if (!L) continue;
    for (const j of ['shoulder', 'elbow', 'forearm', 'wrist', 'hip', 'knee', 'ankle']) clampObj(L[j], ROM[j]);
  }
  return out;
}
