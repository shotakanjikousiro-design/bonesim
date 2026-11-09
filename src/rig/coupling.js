// =====================================================================
//  coupling.js — pure functions for COUPLED joint motion (連動運動).
//
//  Every function here is a small, documented, *approximate* model taken
//  from the classic kinematic literature. They are deliberately simple and
//  smooth so that they can be unit-tested and so that students can see the
//  qualitative pattern. They are NOT patient-specific.
//
//  All angles are in DEGREES. Nothing here touches three.js objects except
//  the quaternion helpers at the bottom (used by anatomicalRig.js).
// =====================================================================
import * as THREE from 'three';

const D2R = Math.PI / 180;
export const clamp = (x, a, b) => Math.min(b, Math.max(a, x));

/** C1-smooth step from 0 (x<=a) to 1 (x>=b). */
export function smoothstep(a, b, x) {
  const t = clamp((x - a) / (b - a), 0, 1);
  return t * t * (3 - 2 * t);
}

/**
 * Smooth ramp: ≈ max(0, x − x0) but with the corner rounded over ±w degrees
 * (quadratic blend), so velocities stay continuous when a phase starts.
 */
export function softRamp(x, x0, w = 10) {
  const d = x - x0;
  if (d <= -w) return 0;
  if (d >= w) return d;
  return (d + w) * (d + w) / (4 * w);
}

// ---------------------------------------------------------------------
//  a. SHOULDER GIRDLE / SCAPULOHUMERAL RHYTHM
// ---------------------------------------------------------------------
/**
 * Default parameters of the shoulder-girdle model.
 *
 *  Scapular upward rotation (thoracic, about the scapular-plane normal):
 *    ≈ 0 during the "setting phase" (first ~30° of humerothoracic elevation),
 *    then 0.4° per degree → 60° at 180° elevation, i.e. overall GH:ST = 120:60 = 2:1
 *    (Inman, Saunders & Abbott 1944; Poppen & Walker 1976 report ≈ 2:1 overall,
 *    closer to 4:1 early and 1:1 late).
 *  Scapular posterior tilt: up to ≈ 20° at full elevation, starting after ~30°
 *    (Ludewig et al. 2009, J Bone Joint Surg Am 91:378).
 *  Scapular external rotation: small, up to ≈ 10° late in elevation (ibid.).
 *  Clavicle elevation at the SC joint: Inman (1944) gives ≈ 4° per 10° of arm
 *    elevation up to 90° (36° total). With a 15 cm clavicle that lifts the
 *    acromion ≈ 9 cm, far more than the 2–4 cm shoulder rise actually seen, and
 *    modern 3-D bone-pin studies measure only ≈ 10–15° (Ludewig et al. 2009).
 *    Default here: 2° per 10° up to 90°, max 18°. The classic Inman numbers are
 *    available as GIRDLE_PARAMS_INMAN.
 *  Clavicle posterior (axial) rotation: ≈ 0 below 90°, then up to ≈ 35° at
 *    full elevation (Inman: 30–50°).
 *  Clavicle protraction: added with forward flexion (plane → 90°), up to ≈ 10°.
 */
export const GIRDLE_PARAMS = {
  settingPhase: 30,               // deg of HT elevation with ≈ no scapular rotation
  upwardRotationPerDeg: 0.4,      // after the setting phase → max 60° at 180°
  posteriorTiltMax: 20,
  externalRotationMax: 10,
  clavicleElevationPerDeg: 0.06,  // ≈ 0.6° per 10° up to 90°: with the scapular rotation the GH centre
  clavicleElevationMax: 6,        //   rises ≈ 2 cm at 90° (Ludewig 2009: clavicle elevation ≈ 6–11° at 120°)
  claviclePosteriorRotationMax: 35,
  retractionMax: 18,              // clavicle RETRACTS ≈ 15–30° during elevation in all planes (Ludewig 2009)
};

/** Inman 1944 classic clavicle numbers (4° per 10° of arm elevation, 36° max). */
export const GIRDLE_PARAMS_INMAN = { ...GIRDLE_PARAMS, clavicleElevationPerDeg: 0.4, clavicleElevationMax: 36 };

/**
 * Shoulder-girdle angles for a humerothoracic elevation θ and plane φ.
 * Returns the THORACIC orientation of the scapula (upwardRotation,
 * posteriorTilt, externalRotation) and the clavicle angles at the SC joint
 * (elevation, protraction (< 0 = retraction), posteriorRotation), in degrees.
 * Elevation in EXTENSION (plane → −90°) does not upwardly rotate the scapula
 * (it tilts anteriorly / rotates downward instead): the elevation-driven terms
 * fade out for planes between −15° and −75°, and extension adds a small
 * anterior tilt and downward rotation (≈ 0.15° per degree).
 */
export function shoulderGirdleAngles(elevation, plane = 0, P = GIRDLE_PARAMS) {
  const th = clamp(elevation, 0, 180);
  const wPlane = smoothstep(-75, -15, plane);     // 1 in abduction/flexion planes, 0 in pure extension
  const ext = (1 - wPlane) * th;                  // "extension amount"
  const upward = wPlane * Math.min(P.upwardRotationPerDeg * softRamp(th, P.settingPhase, 12),
    P.upwardRotationPerDeg * (180 - P.settingPhase)) - 0.15 * ext;
  const late = smoothstep(P.settingPhase, 180, th);
  const posteriorTilt = wPlane * P.posteriorTiltMax * late - 0.15 * ext;
  const externalRotation = wPlane * P.externalRotationMax * smoothstep(60, 180, th);
  // clavicle: linear-ish elevation up to 90°, smoothly saturating
  const clavElev = wPlane * Math.min(P.clavicleElevationMax,
    P.clavicleElevationPerDeg * (th - softRamp(th, 90, 15)));  // ≈ linear to 90°, then flat
  const clavPost = wPlane * P.claviclePosteriorRotationMax * smoothstep(90, 180, th);
  const protraction = -wPlane * P.retractionMax * smoothstep(0, 150, th);
  return {
    scapula: { upwardRotation: upward, posteriorTilt, externalRotation },
    clavicle: { elevation: clavElev, protraction, posteriorRotation: clavPost },
  };
}

/**
 * Optional "automatic humeral external rotation" in high abduction
 * (greater tuberosity clearing the acromion): adds external rotation up to
 * ≈ 35° between 90° and 180° of elevation in the coronal plane; fades out as
 * the plane approaches forward flexion. Returns degrees of INTERNAL rotation
 * to add (negative = external).
 */
export function autoHumeralExternalRotation(elevation, plane = 0) {
  const coronal = Math.max(0, Math.cos(plane * D2R));
  return -35 * smoothstep(90, 180, elevation) * coronal;
}

// ---------------------------------------------------------------------
//  d. WRIST — radiocarpal / midcarpal share
// ---------------------------------------------------------------------
/**
 * Share of wrist motion taken by the RADIOCARPAL joint (proximal carpal row on the
 * radius); the rest is midcarpal. Sarrafian, Melamed & Goshgarian 1977 (Clin Orthop
 * 126:153): palmar FLEXION ≈ 40 % radiocarpal : 60 % midcarpal, DORSIFLEXION
 * (extension) ≈ 66.5 % radiocarpal : 33.5 % midcarpal. Radial–ulnar deviation is
 * approximated as 60 : 40. The share is sign-dependent, so the split is computed on
 * absolute angles (from neutral) and returned as deltas from the rest angles.
 */
export const WRIST_SHARE = { palmarFlexion: 0.4, dorsiflexion: 0.665, ulnarDeviation: 0.6 };
const rcFlex = (f, sh) => (f >= 0 ? sh.palmarFlexion * f : sh.dorsiflexion * f);
export function wristSplit(flexion, ulnarDeviation, restFlexion = 0, restUlnarDeviation = 0, share = WRIST_SHARE) {
  const dF = flexion - restFlexion, dU = ulnarDeviation - restUlnarDeviation;
  const rcF = rcFlex(flexion, share) - rcFlex(restFlexion, share);
  const rcU = dU * share.ulnarDeviation;
  return {
    radiocarpal: { flexion: rcF, ulnarDeviation: rcU },
    midcarpal: { flexion: dF - rcF, ulnarDeviation: dU - rcU },
  };
}

// ---------------------------------------------------------------------
//  f. KNEE — screw-home mechanism
// ---------------------------------------------------------------------
/**
 * Tibial axial rotation relative to the femur (degrees, + = INTERNAL) as a
 * function of knee flexion. Screw-home: the tibia rotates EXTERNALLY by up
 * to ≈ 10° during the last ≈ 20° of extension (Hallén & Lindahl 1966;
 * Ishii 1997 measure ≈ 5–10°). Smooth (C1) curve: −10° at 0°, 0° from 20°
 * flexion onwards, held at −10° in hyper-extension.
 */
export const SCREW_HOME = { max: 10, range: 20 };
export function screwHomeRotation(kneeFlexion, P = SCREW_HOME) {
  const t = clamp(1 - kneeFlexion / P.range, 0, 1);
  return -P.max * t * t * (3 - 2 * t);
}

/**
 * Patella — 1-D root find. Given f(φ) = |apex(φ) − tuberosity| − L (a
 * continuous function of the patellar rotation φ about the patello-femoral
 * axis), return the root nearest to `guess` (degrees). Scans [lo, hi] in
 * `step` increments for sign changes, refines by bisection. Returns `guess`
 * if no root exists (degenerate geometry).
 */
export function solvePatellaAngle(f, guess, lo = -40, hi = 160, step = 2) {
  let best = null;
  let a = lo, fa = f(a);
  for (let b = lo + step; b <= hi + 1e-9; b += step) {
    const fb = f(b);
    if (fa === 0) { if (best === null || Math.abs(a - guess) < Math.abs(best - guess)) best = a; }
    else if (fa * fb < 0) {
      let x0 = a, x1 = b, f0 = fa;
      for (let i = 0; i < 40; i++) {
        const m = 0.5 * (x0 + x1), fm = f(m);
        if (f0 * fm <= 0) x1 = m; else { x0 = m; f0 = fm; }
      }
      const r = 0.5 * (x0 + x1);
      if (best === null || Math.abs(r - guess) < Math.abs(best - guess)) best = r;
    }
    a = b; fa = fb;
  }
  return best === null ? guess : best;
}

/**
 * Patella with a slack/taut patellar tendon (1-D, about the trochlear axis).
 * The patella rides the trochlear groove = rotates by φ about the groove
 * centre (circle fit). f(φ) = |apex(φ) − tuberosity| − L_taut.
 *  - f(0) ≤ 0: tendon slack → φ = 0 (patella stays where it is);
 *  - f(0) > 0: the patella is pulled distally → φ = first root of f in (0, hi].
 * φ is continuous in the knee angle (the root starts at 0 when the tendon
 * becomes taut). Returns {phi, taut}.
 */
export function patellaTautAngle(f, hi = 170, step = 1) {
  let fa = f(0);
  if (!(fa > 0)) return { phi: 0, taut: false };
  for (let a = 0, b = step; b <= hi + 1e-9; a = b, b += step) {
    const fb = f(b);
    if (fb <= 0) {
      let x0 = a, x1 = b, f0 = fa;
      for (let i = 0; i < 40; i++) { const m = 0.5 * (x0 + x1), fm = f(m); if (fm > 0) { x0 = m; f0 = fm; } else x1 = m; }
      void f0; return { phi: 0.5 * (x0 + x1), taut: true };
    }
    fa = fb;
  }
  return { phi: hi, taut: true };
}

// ---------------------------------------------------------------------
//  h. SPINE / NECK — distribution over motion segments
// ---------------------------------------------------------------------
/**
 * Weights distributing thorax-relative-to-pelvis motion over the lumbar
 * motion segments and the thoracolumbar junction. In this model the whole
 * thoracic spine + ribcage is ONE rigid segment, so the T12/L1 weight stands
 * for all thoracic levels.
 *  - flexion: lumbar-heavy (≈ 70 % L1–S1, largest at L4/5 & L5/S1; ≈ 30 % for the
 *    whole thoracic spine; White & Panjabi 1990: lumbar flex/ext ≈ 12–20° per level).
 *  - lateral bend: ≈ half thoracic (White & Panjabi 1990).
 *  - axial rotation: thoracic-heavy (lumbar facets allow only ≈ 1–2° per
 *    level; ≈ 75 % at the thoracic "segment").
 * Weights per joint (order lumbosacral … T12L1) each sum to 1.
 */
export const SPINE_JOINTS = ['lumbosacral', 'L4L5', 'L3L4', 'L2L3', 'L1L2', 'T12L1'];
export const SPINE_WEIGHTS = {
  flexion:       [0.18, 0.17, 0.13, 0.11, 0.11, 0.30],
  lateralBend:   [0.08, 0.10, 0.11, 0.11, 0.10, 0.50],
  axialRotation: [0.04, 0.05, 0.05, 0.05, 0.06, 0.75],
};
/**
 * Neck (head relative to thorax) over C7/T1 … C1/C2 and the atlanto-
 * occipital joint (order below).
 *  - flexion: ≈ 25 % at O–C1 ("yes" joint), ≈ 10 % at C1–C2, rest over C2–T1
 *    with the mid-cervical levels largest.
 *  - lateral bend: mostly C2–C7, ≈ 8 % at O–C1, none at C1–C2.
 *  - axial rotation: ≈ 50 % at C1–C2 ("no" joint), ≈ 3 % O–C1, rest C2–T1.
 */
export const NECK_JOINTS = ['C7T1', 'C6C7', 'C5C6', 'C4C5', 'C3C4', 'C2C3', 'C1C2', 'atlantooccipital'];
export const NECK_WEIGHTS = {
  flexion:       [0.06, 0.11, 0.13, 0.12, 0.10, 0.08, 0.10, 0.30],
  lateralBend:   [0.12, 0.16, 0.17, 0.17, 0.16, 0.14, 0.00, 0.08],
  axialRotation: [0.06, 0.08, 0.09, 0.09, 0.08, 0.07, 0.50, 0.03],
};
/** "Naive" distributions for the 連動運動 OFF comparison: one joint takes all. */
export const SPINE_WEIGHTS_NAIVE = {
  flexion: [1, 0, 0, 0, 0, 0], lateralBend: [1, 0, 0, 0, 0, 0], axialRotation: [1, 0, 0, 0, 0, 0],
};
export const NECK_WEIGHTS_NAIVE = {
  flexion: [1, 0, 0, 0, 0, 0, 0, 0], lateralBend: [1, 0, 0, 0, 0, 0, 0, 0], axialRotation: [1, 0, 0, 0, 0, 0, 0, 0],
};

/** Split {flexion, lateralBend, axialRotation} into per-joint angle triples. */
export function distributeSpine(angles = {}, weights) {
  const n = weights.flexion.length; const out = [];
  for (let i = 0; i < n; i++) out.push({
    flexion: (angles.flexion || 0) * weights.flexion[i],
    lateralBend: (angles.lateralBend || 0) * weights.lateralBend[i],
    axialRotation: (angles.axialRotation || 0) * weights.axialRotation[i],
  });
  return out;
}

// ---------------------------------------------------------------------
//  e. FINGERS
// ---------------------------------------------------------------------
/** If DIP is not given, couple it to PIP (DIP ≈ 2/3 PIP; Rijpkema & Girard 1991). */
export const dipFromPip = (pip) => (2 / 3) * pip;

/** Finger poses for the グー (fist) / パー (open) preset, blended by t∈[0,1] (0 = パー). */
export function fistFingers(t) {
  const f = (mcp, pip, dip, ab = 0) => ({ mcp: mcp * t, pip: pip * t, dip: dip * t, mcpAbduction: ab * (1 - t) });
  return {
    thumb: { cmcFlexion: 35 * t, cmcAbduction: 25 * t, mcp: 45 * t, ip: 50 * t },
    index: f(85, 100, 60, 10), middle: f(88, 100, 62, 0), ring: f(90, 100, 62, 10), little: f(92, 100, 60, 15),
  };
}

// ---------------------------------------------------------------------
//  quaternion helpers
// ---------------------------------------------------------------------
const _v = new THREE.Vector3();
export function axisAngleQ(axis, deg, target = new THREE.Quaternion()) {
  _v.set(axis[0], axis[1], axis[2]).normalize();
  return target.setFromAxisAngle(_v, deg * D2R);
}
/** Twist angle (deg) of quaternion q about unit axis a (swing–twist decomposition). */
export function twistAngle(q, axis) {
  const a = new THREE.Vector3(...axis).normalize();
  const p = a.x * q.x + a.y * q.y + a.z * q.z;
  return 2 * Math.atan2(p, q.w) / D2R;
}
