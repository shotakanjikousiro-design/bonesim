// =====================================================================
//  angles.js — 3D landmarks -> BoneSim Pose (pure functions, no MediaPipe
//  dependency; runs in node and the browser).
//
//  Every angle follows src/shared/conventions.js EXACTLY: the decompositions
//  below are the algebraic inverses of spineQ / shoulderQ / elbowQ /
//  forearmQ / wristQ / hipQ / kneeQ / ankleQ as composed in stickFK().
//  tests/tracking.test.mjs checks the round trip Pose -> stickFK -> synthetic
//  MediaPipe landmarks -> computePose -> Pose.
//
//  ---------------------------------------------------------------------
//  AXES.  MediaPipe world landmarks (pose and hand) are metres, origin at the
//  hip midpoint (pose) / hand centre (hand), axes aligned with the CAMERA:
//      x_mp = image right,  y_mp = image DOWN,  z_mp = AWAY from camera
//  (smaller z = closer to camera; this is the OpenCV camera frame).
//  BoneSim body frame: +X subject-left, +Y up, +Z anterior.
//  For a subject FACING the camera in an UN-mirrored image (raw camera
//  frame, which is what getUserMedia delivers for both front and back
//  cameras — mirroring of the selfie view is normally done only by CSS),
//  the subject's left hand appears on image-right and their front faces the
//  camera, so
//      X_bs =  x_mp,   Y_bs = -y_mp,   Z_bs = -z_mp        (mpToBody)
//  This is a 180° rotation about x (det +1, no handedness change). We call
//  this the "BoneSim camera frame": a three.js camera at the origin with
//  default orientation (looking down -Z, +Y up, +X image right) sees the
//  scene exactly as the webcam does. pose.root.quaternion is the pelvis
//  orientation in this frame (identity = subject squarely facing camera).
//
//  MIRRORED INPUT.  If the frames handed to MediaPipe are already mirrored
//  (some virtual cameras / phone front cameras deliver mirrored pixels), the
//  model sees a reflected person: the subject's anatomical left limb is
//  labelled "right" and the 3D geometry is reflected. prepareBody/prepareHand
//  with {inputMirrored:true} undo this: negate x and swap every left/right
//  landmark pair (and assign hands by geometry, never by the handedness
//  label). The display mirror (CSS selfie view) is irrelevant here.
// =====================================================================
import { ROM, sideSign, SIDES } from '../shared/conventions.js';
import * as M from './math3.js';

const { sub, add, scale, dot, cross, normalize, perp, mid, angleBetween, wrapDeg, smoothstep,
  angleBlend, cols, transpose, mul, mulN, apply, rotX, rotY, rotZ, swingFromNegY, eulerXZY,
  eulerYXZ, matToQuat, R2D, clamp, frameXY, frameYZ, frameZY } = M;

// ---------------- landmark indices ----------------
export const POSE = {
  nose: 0, leftEyeInner: 1, leftEye: 2, leftEyeOuter: 3, rightEyeInner: 4, rightEye: 5, rightEyeOuter: 6,
  leftEar: 7, rightEar: 8, mouthLeft: 9, mouthRight: 10,
  leftShoulder: 11, rightShoulder: 12, leftElbow: 13, rightElbow: 14, leftWrist: 15, rightWrist: 16,
  leftPinky: 17, rightPinky: 18, leftIndex: 19, rightIndex: 20, leftThumb: 21, rightThumb: 22,
  leftHip: 23, rightHip: 24, leftKnee: 25, rightKnee: 26, leftAnkle: 27, rightAnkle: 28,
  leftHeel: 29, rightHeel: 30, leftFootIndex: 31, rightFootIndex: 32,
};
/** per-side landmark indices (subject's anatomical side) */
export const SIDE_LM = {
  left: { eye: 2, ear: 7, shoulder: 11, elbow: 13, wrist: 15, pinky: 17, index: 19, thumb: 21, hip: 23, knee: 25, ankle: 27, heel: 29, toe: 31 },
  right: { eye: 5, ear: 8, shoulder: 12, elbow: 14, wrist: 16, pinky: 18, index: 20, thumb: 22, hip: 24, knee: 26, ankle: 28, heel: 30, toe: 32 },
};
export const POSE_MIRROR_PAIRS = [[1, 4], [2, 5], [3, 6], [7, 8], [9, 10], [11, 12], [13, 14], [15, 16],
  [17, 18], [19, 20], [21, 22], [23, 24], [25, 26], [27, 28], [29, 30], [31, 32]];
export const HAND = {
  wrist: 0, thumbCmc: 1, thumbMcp: 2, thumbIp: 3, thumbTip: 4,
  indexMcp: 5, indexPip: 6, indexDip: 7, indexTip: 8, middleMcp: 9, middlePip: 10, middleDip: 11, middleTip: 12,
  ringMcp: 13, ringPip: 14, ringDip: 15, ringTip: 16, littleMcp: 17, littlePip: 18, littleDip: 19, littleTip: 20,
};
export const FINGER_CHAINS = { index: [5, 6, 7, 8], middle: [9, 10, 11, 12], ring: [13, 14, 15, 16], little: [17, 18, 19, 20] };

// ---------------- axis conversion ----------------
/** MediaPipe world (x right, y down, z away) -> BoneSim camera frame */
export const mpToBody = (p) => [p.x, -p.y, -p.z];
/** BoneSim camera frame -> MediaPipe world */
export const bodyToMp = (v) => ({ x: v[0], y: -v[1], z: -v[2] });
const visOf = (l) => {
  if (!l) return 0;
  const v = l.visibility ?? 1, p = l.presence ?? 1;
  return Math.max(0, Math.min(v, p));
};

/**
 * Mirror a 33-point MediaPipe pose landmark list (world or normalized):
 * swaps left/right pairs and reflects x (world: -x, normalized: 1-x).
 */
export function mirrorPoseLandmarks(list, kind = 'world') {
  if (!list) return list;
  const out = list.map((l) => ({ ...l, x: kind === 'normalized' ? 1 - l.x : -l.x }));
  for (const [a, b] of POSE_MIRROR_PAIRS) { const t = out[a]; out[a] = out[b]; out[b] = t; }
  return out;
}
/** Mirror a 21-point hand landmark list (no index swap — a hand has no pairs). */
export function mirrorHandLandmarks(list, kind = 'world') {
  if (!list) return list;
  return list.map((l) => ({ ...l, x: kind === 'normalized' ? 1 - l.x : -l.x }));
}

/** 33 MediaPipe world landmarks -> { points:[[x,y,z]], vis:[0..1] } in BoneSim camera frame */
export function prepareBody(worldLandmarks, { inputMirrored = false } = {}) {
  const lms = inputMirrored ? mirrorPoseLandmarks(worldLandmarks, 'world') : worldLandmarks;
  return { points: lms.map(mpToBody), vis: lms.map(visOf) };
}
/** 21 MediaPipe hand world landmarks -> [[x,y,z]] in BoneSim camera frame */
export function prepareHand(handWorld, { inputMirrored = false } = {}) {
  const lms = inputMirrored ? mirrorHandLandmarks(handWorld, 'world') : handWorld;
  return lms.map(mpToBody);
}

// ---------------- segment frames from landmarks ----------------
/**
 * Hand frame (columns X,Y,Z in camera frame) per conventions: hand long axis
 * = -Y (wrist -> middle MCP points along -Y), +X = s * (toward thumb side),
 * palm normal = +Z (palm faces +Z in anatomical position).
 * The right-handed cross product with the KNOWN side fixes the palm normal
 * (no front/back ambiguity even if MediaPipe's handedness label is wrong).
 */
export function handFrameFromHandLandmarks(h, side) {
  const s = sideSign(side);
  const Y = normalize(sub(h[HAND.wrist], h[HAND.middleMcp]));
  const across = add(sub(h[HAND.indexMcp], h[HAND.littleMcp]), scale(sub(h[HAND.middleMcp], h[HAND.ringMcp]), 0.5));
  const X = normalize(perp(scale(across, s), Y));
  return cols(X, Y, cross(X, Y));
}
/** Fallback hand frame from pose landmarks wrist / index knuckle / pinky knuckle. */
export function handFrameFromPose(P, side) {
  const s = sideSign(side), L = SIDE_LM[side];
  const knuckles = mid(P[L.index], P[L.pinky]);
  const Y = normalize(sub(P[L.wrist], knuckles));
  const X = normalize(perp(scale(sub(P[L.index], P[L.pinky]), s), Y));
  return cols(X, Y, cross(X, Y));
}

// ---------------- defaults ----------------
export const DEFAULT_ANGLE_OPTIONS = {
  /** Lumbopelvic rhythm prior: fraction of forward trunk inclination given to
   * spine (lumbar) flexion. Pelvic tilt vs lumbar flexion is not separable from
   * 2 hip + 2 shoulder landmarks; 0 = attribute all to pelvis/hips. */
  lumbopelvicRatio: 0.45,
  lumbopelvicExtensionRatio: 0.5,
  /** gravity "up" in BoneSim camera frame (camera assumed upright; calibration refines) */
  gravityUp: [0, 1, 0],
  /** elevation (deg) of the ear-midpoint -> eye-midpoint line in a neutral head (calibrated) */
  neckPitchOffset: 8,
  /** elbow flexion range over which shoulder axial rotation becomes observable */
  axialObsElbow: [15, 35],
  /** knee flexion range over which hip axial rotation (from the shank) becomes observable */
  axialObsKnee: [15, 35],
  /** humerus elevation range over which plane of elevation becomes observable */
  planeObsElevation: [3, 15],
  /** weight of the foot (heel->toe) direction for hip axial rotation */
  footRotationWeight: 0.5,
  /** per-side angle (deg) subtracted from the foot-derived hip rotation (tibial torsion / toe-out) */
  footRotationOffset: { left: 0, right: 0 },
  thumbRestRadialAngle: 40,
};

// ---------------- main ----------------
/**
 * Compute a BoneSim Pose from body landmarks.
 * @param body  {points:[[x,y,z]]*33, vis:[...]} in BoneSim camera frame (see prepareBody)
 * @param opts  {hands?: {left?:{points:[[x,y,z]]*21, score}, right?:{...}},
 *               hold?: {left?:{shoulderAxial, shoulderPlane, hipAxial, wristUlnar}, right?:...},
 *               ...DEFAULT_ANGLE_OPTIONS}
 * @returns {pose, confidence, observability, frames}
 *   observability[path] in 0..1 = how much of the angle came from the current
 *   frame (vs. the hold value) — the stabiliser uses it to decay held values.
 */
export function computePose(body, opts = {}) {
  const o = { ...DEFAULT_ANGLE_OPTIONS, ...opts };
  const P = body.points, V = body.vis || P.map(() => 1);
  const hold = o.hold || {};
  const conf = {}, obs = {}, frames = {};
  const vmin = (...idx) => Math.min(...idx.map((i) => V[i] ?? 0));

  // ---------- pelvis & thorax ----------
  const hL = P[POSE.leftHip], hR = P[POSE.rightHip], sL = P[POSE.leftShoulder], sR = P[POSE.rightShoulder];
  const pm = mid(hL, hR), sm = mid(sL, sR);
  // trunk up-vector; when the hips are not visible (desk webcam, upper body only)
  // MediaPipe's hip guesses are unreliable -> fall back toward gravity "up"
  const hipVis = Math.min(V[POSE.leftHip] ?? 0, V[POSE.rightHip] ?? 0);
  const wTrunk = smoothstep(hipVis, 0.15, 0.5);
  let trunk = normalize(sub(sm, pm), [0, 1, 0]);
  if (wTrunk < 1) trunk = normalize(add(scale(trunk, wTrunk), scale(normalize(o.gravityUp), 1 - wTrunk)), trunk);
  const Rt = frameXY(sub(sL, sR), trunk);             // thorax: X = R->L shoulder, Y = trunk
  const Rp0 = frameXY(sub(hL, hR), trunk);            // pelvis with trunk as up-vector
  const Xp = M.col(Rp0, 0);

  // lumbopelvic rhythm prior (sagittal split between pelvic tilt and lumbar flexion)
  let spineFlexPrior = 0;
  if (o.lumbopelvicRatio > 0 || o.lumbopelvicExtensionRatio > 0) {
    const up = normalize(o.gravityUp);
    const fwd = normalize(cross(Xp, up));
    const incl = Math.atan2(dot(trunk, fwd), dot(trunk, up)) * R2D; // + = leaning forward
    if (incl > 0) {
      // trunk-thigh angle (hip flexion if spine flexion were 0)
      const thighFlex = SIDES.map((sd) => {
        const L = SIDE_LM[sd]; const d = apply(transpose(Rp0), normalize(sub(P[L.knee], P[L.hip])));
        return Math.atan2(d[2], -d[1]) * R2D;
      });
      const x = Math.max(0, Math.min(incl, (thighFlex[0] + thighFlex[1]) / 2));
      spineFlexPrior = Math.min(ROM.spine.flexion[1], o.lumbopelvicRatio * x);
    } else {
      spineFlexPrior = Math.max(ROM.spine.flexion[0], o.lumbopelvicExtensionRatio * incl);
    }
  }
  const Rp = mul(Rp0, rotX(-spineFlexPrior));
  frames.pelvis = Rp; frames.thorax = Rt;
  const RpT = transpose(Rp), RtT = transpose(Rt);
  const shVis = vmin(POSE.leftShoulder, POSE.rightShoulder);
  const torsoVis = Math.min(shVis, hipVis);   // spine needs both girdles

  const pose = { root: { position: [0, 0, 0], quaternion: matToQuat(Rp) } };
  conf.root = shVis * (0.3 + 0.7 * wTrunk);

  // spine: thorax rel. pelvis = Rx(flex) Rz(bend) Ry(rot)
  {
    const [f, b, r] = eulerXZY(mul(RpT, Rt));
    pose.spine = { flexion: f, lateralBend: b, axialRotation: r };
    conf['spine.flexion'] = torsoVis * 0.4; // mostly a prior
    conf['spine.lateralBend'] = torsoVis * wTrunk; conf['spine.axialRotation'] = torsoVis * wTrunk;
  }

  // neck: head rel. thorax, head frame from ears + eyes
  {
    const eL = P[POSE.leftEar], eR = P[POSE.rightEar];
    const eyes = mid(P[POSE.leftEye], P[POSE.rightEye]);
    const X = normalize(sub(eL, eR));
    const Z = normalize(perp(sub(eyes, mid(eL, eR)), X));
    const Hobs = cols(X, cross(Z, X), Z);
    const Hh = mul(Hobs, rotX(o.neckPitchOffset));
    frames.head = Hh;
    const [f, b, r] = eulerXZY(mul(RtT, Hh));
    pose.neck = { flexion: f, lateralBend: b, axialRotation: r };
    const c = vmin(POSE.leftEar, POSE.rightEar, POSE.leftEye, POSE.rightEye) * shVis;
    conf['neck.flexion'] = c * 0.8; conf['neck.lateralBend'] = c; conf['neck.axialRotation'] = c;
  }

  for (const side of SIDES) {
    const s = sideSign(side), L = SIDE_LM[side], hd = hold[side] || {};
    const out = (pose[side] = {});
    const fr = (frames[side] = {});
    const c = (k, v) => { conf[`${side}.${k}`] = clamp(v, 0, 1); };
    const ob = (k, v) => { obs[`${side}.${k}`] = clamp(v, 0, 1); };

    // ======== upper limb ========
    const S = P[L.shoulder], E = P[L.elbow], W = P[L.wrist];
    const ua = normalize(sub(E, S)), fa = normalize(sub(W, E));
    const d = apply(RtT, ua);
    const elevation = Math.acos(clamp(-d[1], -1, 1)) * R2D;
    const planeObs = Math.atan2(d[2], s * d[0]) * R2D;
    const wPlane = smoothstep(elevation, o.planeObsElevation[0], o.planeObsElevation[1]);
    const plane = angleBlend(planeObs, hd.shoulderPlane ?? 0, wPlane);
    const elbowFlex = angleBetween(ua, fa);
    const Sw = swingFromNegY(d);
    const fs = apply(transpose(Sw), apply(RtT, fa));
    const psiObs = Math.atan2(-s * fs[0], fs[2]) * R2D;
    const wPsi = smoothstep(elbowFlex, o.axialObsElbow[0], o.axialObsElbow[1]) * (1 - smoothstep(elevation, 165, 178));
    const psi = angleBlend(psiObs, hd.shoulderAxial ?? 0, wPsi);
    const Hm = mulN(Rt, Sw, rotY(-s * psi));
    const Um = mul(Hm, rotX(-elbowFlex));
    fr.humerus = Hm; fr.ulna = Um;
    out.shoulder = { elevation, plane, axialRotation: psi };
    out.elbow = { flexion: elbowFlex };
    const armVis = vmin(L.shoulder, L.elbow) * shVis;
    const foreVis = vmin(L.shoulder, L.elbow, L.wrist);
    c('shoulder.elevation', armVis); c('shoulder.plane', armVis * wPlane);
    c('shoulder.axialRotation', foreVis * wPsi); c('elbow.flexion', foreVis);
    ob('shoulder.plane', wPlane); ob('shoulder.axialRotation', wPsi);

    // ---- forearm pronation & wrist: hand frame relative to ulna frame ----
    // ulna^-1 * hand = Ry(-s(pron+90)) * Rx(-wflex) * Rz(-s*ulnarDev)   (YXZ)
    const hand = o.hands?.[side];
    let Hd, handConf, fromHand = false;
    if (hand?.points?.length === 21) {
      Hd = handFrameFromHandLandmarks(hand.points, side); fromHand = true;
      handConf = (hand.score ?? 1) * vmin(L.wrist);
    } else {
      Hd = handFrameFromPose(P, side);
      handConf = vmin(L.wrist, L.index, L.pinky) * 0.5;
    }
    fr.hand = Hd;
    {
      const [ay, bx, cz] = eulerYXZ(mul(transpose(Um), Hd));
      const wristFlex = -bx;
      // gimbal lock at +-90 deg wrist flexion: pronation & deviation couple -> hold deviation
      const wz = smoothstep(90 - Math.abs(bx), 4, 15);
      const ulnarObs = -s * cz;
      const ulnar = angleBlend(ulnarObs, hd.wristUlnar ?? 0, wz);
      const ay2 = ay + Math.sign(bx) * ((-s * ulnar) - cz);
      const pron = wrapDeg(-s * ay2 - 90);
      out.forearm = { pronation: pron };
      out.wrist = { flexion: wristFlex, ulnarDeviation: ulnar };
      fr.radius = mul(Um, rotY(-s * (pron + 90)));
      // when the elbow is straight, humeral rotation and pronation are not separable
      c('forearm.pronation', handConf * foreVis * (0.5 + 0.5 * wPsi));
      c('wrist.flexion', handConf * (fromHand ? 1 : 0.5));
      c('wrist.ulnarDeviation', handConf * (fromHand ? 1 : 0.4) * wz);
      ob('wrist.ulnarDeviation', wz);
    }
    if (fromHand) {
      out.fingers = computeFingers(hand.points, Hd, side, o);
      c('fingers', hand.score ?? 1);
    }

    // ======== lower limb ========
    const Hp = P[L.hip], K = P[L.knee], A = P[L.ankle], Heel = P[L.heel], Toe = P[L.toe];
    const th = normalize(sub(K, Hp)), sh = normalize(sub(A, K));
    const dl = apply(RpT, th);
    // femur dir = Rx(-f) Rz(s a) (0,-1,0) = (s sin a, -cos a cos f, cos a sin f)
    const abd = Math.asin(clamp(s * dl[0], -1, 1)) * R2D;
    const flex = Math.atan2(dl[2], -dl[1]) * R2D;
    const F0 = mulN(Rp, rotX(-flex), rotZ(s * abd));
    const F0T = transpose(F0);
    const knee = angleBetween(th, sh);
    // shank in F0 = Ry(-s r)(0,-cos k,-sin k) = (s sin r sin k, -cos k, -sin k cos r)
    const v = apply(F0T, sh);
    const rShank = Math.atan2(s * v[0], -v[2]) * R2D;
    const legVis = vmin(L.hip, L.knee, L.ankle);
    const wShank = smoothstep(knee, o.axialObsKnee[0], o.axialObsKnee[1]) * legVis;
    // foot frame: Z = heel->toe, Y = toward ankle (heel, ankle, toe span the foot sagittal plane)
    const Fo = frameZY(sub(Toe, Heel), sub(A, Heel));
    fr.foot = Fo;
    const footVis = vmin(L.ankle, L.heel, L.toe);
    // F0^-1 foot = Ry(-s r) Rx(k - dorsi) Rz(-s inv)  -> the foot observes r for any knee
    // angle, EXCEPT when the foot is parallel to the thigh (k - dorsi = 90°, e.g. sitting):
    // then hip rotation and inversion both spin the foot about the same axis (gimbal).
    let [fy, fx, fz] = eulerYXZ(mul(F0T, Fo));
    if (Math.abs(fz) > 90) { fy = wrapDeg(fy - 180); fx = 180 - fx; fz = wrapDeg(fz - 180); } // pick |inversion| < 90
    const gFoot = smoothstep(90 - Math.abs(wrapDeg(fx) > 90 ? 180 - Math.abs(fx) : Math.abs(fx)), 8, 25);
    const rFoot = -s * fy - (o.footRotationOffset?.[side] ?? 0);
    const wFoot = gFoot * footVis;
    const fused = M.angleWeightedMean([rShank, rFoot], [wShank, wFoot * o.footRotationWeight]);
    const wR = clamp(Math.max(wShank, wFoot), 0, 1);
    const hipRot = angleBlend(fused.angle, hd.hipAxial ?? 0, wR);
    const Fm = mul(F0, rotY(-s * hipRot));
    const Tm = mul(Fm, rotX(knee));
    fr.femur = Fm; fr.tibia = Tm;
    out.hip = { flexion: flex, abduction: abd, axialRotation: hipRot };
    out.knee = { flexion: knee };
    const [, ax, az] = eulerYXZ(mul(transpose(Tm), Fo));
    out.ankle = { dorsiflexion: -ax, inversion: -s * az };
    const hipVis = vmin(L.hip, L.knee) * vmin(POSE.leftHip, POSE.rightHip);
    c('hip.flexion', hipVis); c('hip.abduction', hipVis);
    c('hip.axialRotation', hipVis * wR); ob('hip.axialRotation', wR);
    c('knee.flexion', legVis);
    c('ankle.dorsiflexion', footVis * legVis);
    c('ankle.inversion', footVis * legVis * 0.5); // heel/toe/ankle only weakly observe inversion
  }

  // per-joint summary (min over its angles)
  for (const key of Object.keys(conf)) {
    const parts = key.split('.');
    if (parts.length === 3) {
      const j = `${parts[0]}.${parts[1]}`;
      conf[j] = Math.min(conf[j] ?? 1, conf[key]);
    } else if (parts.length === 2 && parts[0] !== 'left' && parts[0] !== 'right') {
      conf[parts[0]] = Math.min(conf[parts[0]] ?? 1, conf[key]);
    }
  }
  pose.confidence = conf;
  return { pose, confidence: conf, observability: obs, frames };
}

// ---------------- fingers ----------------
// Finger model (documented here because conventions.js only says "Rx(-flex) in
// the hand frame"): for each long finger a metacarpal frame Mf is built with
// -Y along wrist->MCP and X = hand X orthogonalised. Then
//   proximal phalanx frame = Mf * Rz(sigma*mcpAbduction) * Rx(-mcp)
//   middle  = proximal * Rx(-pip),  distal = middle * Rx(-dip)
// sigma = +s for index & middle (radial = +), -s for ring & little (ulnar = +),
// i.e. mcpAbduction + = away from the middle finger.
// Thumb: metacarpal frame = Hand * Rz(s*(restRadial - cmcFlexion)) * Rx(-cmcAbduction)
// (cmcAbduction = palmar abduction toward +Z, cmcFlexion = toward the ulnar side
// in the palm plane, rest radial angle 40 deg); mcp / ip = unsigned angles between
// consecutive thumb segments (approximate).
export function computeFingers(h, Hd, side, o = DEFAULT_ANGLE_OPTIONS) {
  const s = sideSign(side);
  const X = M.col(Hd, 0);
  const out = {};
  for (const [name, [mcp, pip, dip, tip]] of Object.entries(FINGER_CHAINS)) {
    const Ym = normalize(sub(h[HAND.wrist], h[mcp]));
    const Mf = frameYZ(Ym, cross(X, Ym));
    const pd = apply(transpose(Mf), normalize(sub(h[pip], h[mcp])));
    const m = Math.atan2(pd[2], Math.hypot(pd[0], pd[1])) * R2D;
    const phi = Math.atan2(pd[0], -pd[1]) * R2D;
    const sigma = (name === 'index' || name === 'middle') ? s : -s;
    const abd = sigma * phi * smoothstep(90 - Math.abs(m), 2, 8);
    const F1 = mulN(Mf, rotZ(sigma * abd), rotX(-m));
    const md = apply(transpose(F1), normalize(sub(h[dip], h[pip])));
    const pipA = Math.atan2(md[2], -md[1]) * R2D;
    const F2 = mul(F1, rotX(-pipA));
    const dd = apply(transpose(F2), normalize(sub(h[tip], h[dip])));
    const dipA = Math.atan2(dd[2], -dd[1]) * R2D;
    out[name] = { mcp: m, mcpAbduction: abd, pip: pipA, dip: dipA };
  }
  const td = apply(transpose(Hd), normalize(sub(h[HAND.thumbMcp], h[HAND.thumbCmc])));
  const cmcAbduction = Math.atan2(td[2], Math.hypot(td[0], td[1])) * R2D;
  const radial = s * Math.atan2(td[0], -td[1]) * R2D;
  out.thumb = {
    cmcFlexion: (o.thumbRestRadialAngle ?? 40) - radial,
    cmcAbduction,
    mcp: angleBetween(sub(h[HAND.thumbMcp], h[HAND.thumbCmc]), sub(h[HAND.thumbIp], h[HAND.thumbMcp])),
    ip: angleBetween(sub(h[HAND.thumbIp], h[HAND.thumbMcp]), sub(h[HAND.thumbTip], h[HAND.thumbIp])),
  };
  return out;
}

/** Synthesise 21 hand landmarks (BoneSim frame, metres) from a hand frame + finger angles.
 *  Inverse of computeFingers/handFrameFromHandLandmarks; used by tests and as documentation. */
export function synthHand(Hd, wristPos, side, fingers = {}, o = DEFAULT_ANGLE_OPTIONS) {
  const s = sideSign(side);
  const at = (local) => add(wristPos, apply(Hd, local));
  const pts = new Array(21);
  pts[0] = at([0, 0, 0]);
  const mcpLocal = { index: [s * 0.024, -0.085, 0], middle: [0, -0.09, 0], ring: [-s * 0.02, -0.083, 0], little: [-s * 0.038, -0.076, 0] };
  const lens = { index: [0.045, 0.026, 0.02], middle: [0.05, 0.03, 0.021], ring: [0.046, 0.028, 0.02], little: [0.035, 0.02, 0.018] };
  const X = M.col(Hd, 0);
  for (const [name, idx] of Object.entries(FINGER_CHAINS)) {
    const f = fingers[name] || {};
    const mcpP = at(mcpLocal[name]);
    const Ym = normalize(sub(pts[0], mcpP));
    const Mf = frameYZ(Ym, cross(X, Ym));
    const sigma = (name === 'index' || name === 'middle') ? s : -s;
    const F1 = mulN(Mf, rotZ(sigma * (f.mcpAbduction ?? 0)), rotX(-(f.mcp ?? 0)));
    const F2 = mul(F1, rotX(-(f.pip ?? 0)));
    const F3 = mul(F2, rotX(-(f.dip ?? 0)));
    const [l1, l2, l3] = lens[name];
    pts[idx[0]] = mcpP;
    pts[idx[1]] = add(mcpP, apply(F1, [0, -l1, 0]));
    pts[idx[2]] = add(pts[idx[1]], apply(F2, [0, -l2, 0]));
    pts[idx[3]] = add(pts[idx[2]], apply(F3, [0, -l3, 0]));
  }
  const t = fingers.thumb || {};
  const T1 = mulN(Hd, rotZ(s * ((o.thumbRestRadialAngle ?? 40) - (t.cmcFlexion ?? 0))), rotX(-(t.cmcAbduction ?? 0)));
  const T2 = mul(T1, rotX(-(t.mcp ?? 0))); const T3 = mul(T2, rotX(-(t.ip ?? 0)));
  pts[1] = at([s * 0.022, -0.025, 0.012]);
  pts[2] = add(pts[1], apply(T1, [0, -0.045, 0]));
  pts[3] = add(pts[2], apply(T2, [0, -0.032, 0]));
  pts[4] = add(pts[3], apply(T3, [0, -0.027, 0]));
  return pts;
}

