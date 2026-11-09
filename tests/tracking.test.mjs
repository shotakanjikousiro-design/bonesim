// Round-trip & filter tests for src/tracking  (node tests/tracking.test.mjs)
//   Pose -> conventions.stickFK -> synthetic MediaPipe landmarks (pose 33 + hands 21,
//   MediaPipe axes, both sides, optionally mirrored input) -> angles.computePose -> Pose
import * as C from '../src/shared/conventions.js';
import { computePose, prepareBody, prepareHand, mirrorPoseLandmarks } from '../src/tracking/angles.js';
import { synthesizeFrame, makeRng } from '../src/tracking/synth.js';
import { createPipeline } from '../src/tracking/pipeline.js';
import { estimateDims } from '../src/tracking/calibration.js';
import { enforceSegmentLengths, OneEuroAngle, circularClamp } from '../src/tracking/filters.js';
import { computeAlignment, projectOrtho, projectPerspective } from '../src/tracking/alignment.js';
import { wrapDeg, quatToMat, rotationAngleBetween, dist } from '../src/tracking/math3.js';

let failures = 0;
const ok = (cond, msg) => { console.log(`${cond ? 'PASS' : 'FAIL'}  ${msg}`); if (!cond) failures++; };
const rng = makeRng(12345);
const U = (a, b) => a + (b - a) * rng.uniform();
const NOHOLD_OPTS = { lumbopelvicRatio: 0, lumbopelvicExtensionRatio: 0 };

function randomQuat(yaw, pitch, roll) {
  const q = C.Ry(yaw).multiply(C.Rx(pitch)).multiply(C.Rz(roll));
  return [q.x, q.y, q.z, q.w];
}
function randomPose({ elbowMin = 40, kneeMin = 0 } = {}) {
  const limb = () => ({
    shoulder: { elevation: U(20, 150), plane: U(-60, 120), axialRotation: U(-50, 70) },
    elbow: { flexion: U(elbowMin, 140) },
    forearm: { pronation: U(-85, 85) },
    wrist: { flexion: U(-60, 70), ulnarDeviation: U(-20, 40) },
    hip: { flexion: U(-10, 100), abduction: U(-15, 40), axialRotation: U(-40, 40) },
    knee: { flexion: U(kneeMin, 120) },
    ankle: { dorsiflexion: U(-40, 15), inversion: U(-15, 25) },
    fingers: {
      thumb: { cmcFlexion: U(0, 40), cmcAbduction: U(5, 60), mcp: U(5, 50), ip: U(5, 60) },
      ...Object.fromEntries(['index', 'middle', 'ring', 'little'].map((f) => [f, { mcp: U(0, 80), mcpAbduction: U(-15, 15), pip: U(0, 90), dip: U(0, 60) }])),
    },
  });
  return {
    root: { position: [0, 0, 0], quaternion: randomQuat(U(-40, 40), U(-15, 15), U(-10, 10)) },
    spine: { flexion: 0, lateralBend: U(-25, 25), axialRotation: U(-30, 30) },
    neck: { flexion: U(-40, 40), lateralBend: U(-30, 30), axialRotation: U(-50, 50) },
    left: limb(), right: limb(),
  };
}
const ANGLES = [
  'spine.flexion', 'spine.lateralBend', 'spine.axialRotation', 'neck.flexion', 'neck.lateralBend', 'neck.axialRotation',
  ...['shoulder.elevation', 'shoulder.plane', 'shoulder.axialRotation', 'elbow.flexion', 'forearm.pronation',
    'wrist.flexion', 'wrist.ulnarDeviation', 'hip.flexion', 'hip.abduction', 'hip.axialRotation', 'knee.flexion',
    'ankle.dorsiflexion', 'ankle.inversion'].flatMap((a) => [`left.${a}`, `right.${a}`]),
];
const FINGER_ANGLES = ['index.mcp', 'index.mcpAbduction', 'index.pip', 'index.dip', 'middle.mcp', 'ring.pip', 'little.mcp', 'little.mcpAbduction', 'little.dip',
  'thumb.cmcFlexion', 'thumb.cmcAbduction', 'thumb.mcp', 'thumb.ip'];
const get = (o, path) => path.split('.').reduce((a, k) => a?.[k], o);

function runOnce(pose, { inputMirrored = false, withHands = true, hold } = {}) {
  const fr = synthesizeFrame(pose, { inputMirrored, withHands });
  const body = prepareBody(fr.worldLandmarks, { inputMirrored });
  // hand assignment as the pipeline does it (geometry, not label): synth emits [left, right]
  const hands = {};
  if (withHands) {
    const lm2d = inputMirrored ? mirrorPoseLandmarks(fr.landmarks, 'normalized') : fr.landmarks;
    for (const h of fr.hands) {
      const wx = inputMirrored ? 1 - h.landmarks[0].x : h.landmarks[0].x, wy = h.landmarks[0].y;
      const dl = Math.hypot(wx - lm2d[15].x, wy - lm2d[15].y), dr = Math.hypot(wx - lm2d[16].x, wy - lm2d[16].y);
      hands[dl < dr ? 'left' : 'right'] = { points: prepareHand(h.worldLandmarks, { inputMirrored }), score: 1 };
    }
  }
  return computePose(body, { ...NOHOLD_OPTS, hands, hold });
}

function errorTable(label, poses, opts, angleList = ANGLES, extra = {}) {
  const stats = {};
  let rootMax = 0;
  for (const p of poses) {
    const r = runOnce(p, opts);
    for (const a of angleList) {
      const e = Math.abs(wrapDeg(get(r.pose, a) - get(p, a)));
      (stats[a] ||= []).push(e);
    }
    if (extra.fingers) for (const side of C.SIDES) for (const a of FINGER_ANGLES) {
      const e = Math.abs(wrapDeg(get(r.pose[side].fingers, a) - get(p[side].fingers, a)));
      (stats[`${side}.fingers.${a}`] ||= []).push(e);
    }
    rootMax = Math.max(rootMax, rotationAngleBetween(quatToMat(r.pose.root.quaternion), quatToMat(p.root.quaternion)));
  }
  console.log(`\n--- ${label} (${poses.length} poses) ---`);
  console.log('angle'.padEnd(36) + 'max°'.padStart(8) + 'mean°'.padStart(8));
  let worst = 0;
  for (const [a, es] of Object.entries(stats)) {
    const mx = Math.max(...es), mean = es.reduce((x, y) => x + y, 0) / es.length;
    worst = Math.max(worst, mx);
    if (!extra.quiet) console.log(a.padEnd(36) + mx.toFixed(3).padStart(8) + mean.toFixed(3).padStart(8));
  }
  console.log('root orientation'.padEnd(36) + rootMax.toFixed(3).padStart(8));
  return { worst: Math.max(worst, rootMax), stats };
}

// ---------------- 1. random poses, both sides ----------------
const poses = Array.from({ length: 400 }, () => randomPose());
let r = errorTable('Random poses, raw camera frame (hands on)', poses, {}, ANGLES, { fingers: true });
ok(r.worst < 2, `random poses: all observable angles < 2° (worst ${r.worst.toFixed(4)}°)`);
r = errorTable('Random poses, MIRRORED input (inputMirrored=true)', poses, { inputMirrored: true }, ANGLES, { fingers: true, quiet: true });
ok(r.worst < 2, `mirrored input: all angles < 2° (worst ${r.worst.toFixed(4)}°)`);
r = errorTable('Random poses, pose landmarks only (no HandLandmarker)', poses, { withHands: false }, ANGLES, { quiet: true });
ok(r.worst < 2, `pose-only hand fallback: all angles < 2° (worst ${r.worst.toFixed(4)}°)`);

// mirrored input WITHOUT correction must give a different (wrong) pose — proves the flag matters
{
  const p = poses[0]; const fr = synthesizeFrame(p, { inputMirrored: true });
  const wrong = computePose(prepareBody(fr.worldLandmarks, { inputMirrored: false }), NOHOLD_OPTS).pose;
  const e = Math.abs(wrapDeg(wrong.left.shoulder.elevation - p.left.shoulder.elevation)) + Math.abs(wrapDeg(wrong.right.shoulder.elevation - p.right.shoulder.elevation));
  ok(e > 1, `un-corrected mirrored input swaps sides (sanity, Δ=${e.toFixed(1)}°)`);
}

// ---------------- 2. special sweeps ----------------
function sweep(label, mk, check, opts = {}) {
  let worst = 0;
  for (const side of C.SIDES) for (let v = -90; v <= 90; v += 5) {
    const p = mk(side, v); const res = runOnce(p, opts);
    worst = Math.max(worst, check(res, p, side, v));
  }
  console.log(`${label.padEnd(64)} max err ${worst.toFixed(4)}°`);
  return worst;
}
const base = () => { const p = C.anatomicalPose(); p.root = { position: [0, 0, 0], quaternion: [0, 0, 0, 1] }; return p; };
console.log('\n--- Special cases ---');
let w = sweep('pronation sweep -90..90, elbow 90°, shoulder 0', (side, v) => { const p = base(); p[side].elbow.flexion = 90; p[side].forearm.pronation = v; return p; },
  (res, p, side, v) => Math.abs(wrapDeg(res.pose[side].forearm.pronation - v)));
ok(w < 2, 'pronation sweep with elbow 90°');
w = sweep('pronation sweep, elbow 90°, shoulder flex 60/int.rot 30', (side, v) => { const p = base(); Object.assign(p[side].shoulder, { elevation: 60, plane: 90, axialRotation: 30 }); p[side].elbow.flexion = 90; p[side].forearm.pronation = v; return p; },
  (res, p, side, v) => Math.abs(wrapDeg(res.pose[side].forearm.pronation - v)));
ok(w < 2, 'pronation sweep with elbow 90° and shoulder flexed/rotated');
w = sweep('pronation sweep, elbow 90°, pose-only (no hand model)', (side, v) => { const p = base(); p[side].elbow.flexion = 90; p[side].forearm.pronation = v; return p; },
  (res, p, side, v) => Math.abs(wrapDeg(res.pose[side].forearm.pronation - v)), { withHands: false });
ok(w < 2, 'pronation sweep, pose-landmark fallback');
w = sweep('pronation sweep, elbow 90°, mirrored input', (side, v) => { const p = base(); p[side].elbow.flexion = 90; p[side].forearm.pronation = v; return p; },
  (res, p, side, v) => Math.abs(wrapDeg(res.pose[side].forearm.pronation - v)), { inputMirrored: true });
ok(w < 2, 'pronation sweep, mirrored input');
w = sweep('pronation sweep, elbow STRAIGHT (ψ=0, hold 0)', (side, v) => { const p = base(); p[side].elbow.flexion = 0; p[side].forearm.pronation = v; return p; },
  (res, p, side, v) => Math.abs(wrapDeg(res.pose[side].forearm.pronation - v)));
ok(w < 2, 'pronation sweep with straight elbow (ψ held at true value)');
// straight elbow + unknown ψ: only ψ+pronation observable -> hand orientation must still match
w = sweep('elbow straight, true ψ=25, hold ψ=0: hand-orientation err', (side, v) => { const p = base(); p[side].shoulder.axialRotation = 25; p[side].elbow.flexion = 0; p[side].forearm.pronation = v * 0.6; return p; },
  (res, p, side) => {
    const a = C.stickFK(p).quaternions[`hand_${side}`]; const b = C.stickFK(res.pose).quaternions[`hand_${side}`];
    return rotationAngleBetween(quatToMat([a.x, a.y, a.z, a.w]), quatToMat([b.x, b.y, b.z, b.w]));
  });
ok(w < 2, 'straight elbow: hand orientation preserved (ψ+pronation sum)');
w = 0;
for (const side of C.SIDES) for (let v = -60; v <= 80; v += 5) {
  const p = base(); p[side].elbow.flexion = 90; p[side].shoulder.axialRotation = v;
  const res = runOnce(p); w = Math.max(w, Math.abs(wrapDeg(res.pose[side].shoulder.axialRotation - v)));
  const p2 = base(); Object.assign(p2[side].shoulder, { elevation: 90, plane: 0, axialRotation: v }); p2[side].elbow.flexion = 90;
  w = Math.max(w, Math.abs(wrapDeg(runOnce(p2).pose[side].shoulder.axialRotation - v)));
}
console.log(`${'shoulder axial sweep -60..80, elbow 90° (arm down & abducted 90)'.padEnd(64)} max err ${w.toFixed(4)}°`);
ok(w < 2, 'shoulder axial rotation sweep with elbow 90°');
w = 0;
for (const side of C.SIDES) for (let v = -45; v <= 45; v += 5) for (const knee of [0, 90]) {
  const p = base(); Object.assign(p[side].hip, { flexion: knee ? 90 : 0, axialRotation: v }); p[side].knee.flexion = knee;
  w = Math.max(w, Math.abs(wrapDeg(runOnce(p).pose[side].hip.axialRotation - v)));
}
console.log(`${'hip axial sweep -45..45 (knee 0 via foot, knee 90 via shank+foot)'.padEnd(64)} max err ${w.toFixed(4)}°`);
ok(w < 2, 'hip axial rotation sweep');

// ---------------- 3. spine flexion ambiguity: positions must round-trip ----------------
{
  let maxPos = 0;
  for (let i = 0; i < 100; i++) {
    const p = randomPose(); p.spine.flexion = U(-25, 40);
    for (const ratio of [0, 0.45]) {
      const res = computePose(prepareBody(synthesizeFrame(p).worldLandmarks), { lumbopelvicRatio: ratio, hands: {} });
      const A = C.stickFK(p).positions, B = C.stickFK(res.pose).positions;
      for (const k of ['neckBase', 'shoulder_left', 'elbow_right', 'wrist_left', 'knee_left', 'ankle_right', 'toe_left']) maxPos = Math.max(maxPos, A[k].distanceTo(B[k]));
    }
  }
  console.log(`\nspine flexion≠0 (pelvic tilt vs lumbar flexion prior): max FK position error ${(maxPos * 1000).toFixed(3)} mm`);
  ok(maxPos < 0.002, 'FK joint positions round-trip with any lumbopelvic split');
}

// ---------------- 4. calibration & bone lengths ----------------
{
  const dimsTrue = Object.fromEntries(Object.entries(C.DEFAULT_DIMS).map(([k, v]) => [k, v * 1.08]));
  const frames = [];
  const nr = makeRng(3);
  for (let i = 0; i < 60; i++) {
    const p = base(); p.left.shoulder.elevation = 20; p.right.shoulder.elevation = 20; p.left.forearm.pronation = 0; p.right.forearm.pronation = 0;
    const fr = synthesizeFrame(p, { dims: dimsTrue, noise: 0.005, rng: nr });
    frames.push(prepareBody(fr.worldLandmarks));
  }
  const cal = estimateDims(frames);
  let maxRel = 0;
  for (const k of ['hipHalfWidth', 'shoulderHalfWidth', 'thoraxHeight', 'upperArm', 'forearm', 'thigh', 'shank', 'footLength']) maxRel = Math.max(maxRel, Math.abs(cal.dims[k] / dimsTrue[k] - 1));
  console.log(`calibration: height ${cal.height.toFixed(3)} m (true ${(1.72 * 1.08).toFixed(3)}), max segment rel. err ${(maxRel * 100).toFixed(2)}%, neckPitchOffset ${cal.neckPitchOffset.toFixed(2)}°`);
  ok(maxRel < 0.03, 'calibration recovers segment lengths (5 mm noise) within 3%');
  ok(Math.abs(cal.neckPitchOffset - 8) < 1.5, 'calibration recovers neutral head pitch offset');
  // full pipeline: calibrate() then angles stay exact with bone-length enforcement active
  const pl = createPipeline({ angles: NOHOLD_OPTS });
  const prom = pl.calibrate({ durationMs: 1000 });
  let resolved = null; prom.then((c) => { resolved = c; });
  for (let i = 0; i < 40; i++) pl.process({ t: i * 33.3, ...synthesizeFrame(base(), { dims: dimsTrue }) });
  await Promise.resolve();
  ok(resolved && Math.abs(pl.dims.thigh / dimsTrue.thigh - 1) < 0.01, `pipeline.calibrate() resolves and updates dims (thigh ${pl.dims.thigh.toFixed(3)} m)`);
  const pt = randomPose(); let o2;
  for (let i = 40; i < 100; i++) o2 = pl.process({ t: i * 33.3, ...synthesizeFrame(pt, { dims: dimsTrue }) });
  const eMax = Math.max(...ANGLES.map((a) => Math.abs(wrapDeg(get(C.clampPose(pt), a) - get(o2.pose, a)))));
  ok(eMax < 2, `after calibration (bone lengths enforced) static pose converges within 2° (worst ${eMax.toFixed(3)}°)`);
  // implicit calibration (autoDims) while moving, no calibrate() call
  const pa = createPipeline({ angles: NOHOLD_OPTS }); let oa;
  for (let i = 0; i < 120; i++) { const p = base(); p.left.elbow.flexion = 60 + 40 * Math.sin(i / 10); p.right.shoulder.elevation = 30 + 20 * Math.sin(i / 13); oa = pa.process({ t: i * 33.3, ...synthesizeFrame(p, { dims: dimsTrue, noise: 0.005, rng: nr }) }); }
  const relA = Math.max(...['upperArm', 'forearm', 'thigh', 'shank', 'shoulderHalfWidth'].map((k) => Math.abs(oa.dims[k] / dimsTrue[k] - 1)));
  ok(oa.dimsSource === 'auto' && relA < 0.03, `autoDims adapts the rig without calibrate() (max rel. err ${(relA * 100).toFixed(2)}%)`);
  // desk webcam: upper body only (hips/legs not visible) -> arms still tracked with good confidence
  const pu = createPipeline({ angles: NOHOLD_OPTS }); let ou; const pt2 = randomPose(); pt2.root.quaternion = [0, 0, 0, 1]; pt2.spine = { flexion: 0, lateralBend: 0, axialRotation: 0 };
  for (let i = 0; i < 40; i++) { const fr = synthesizeFrame(pt2); for (let j = 23; j <= 32; j++) { fr.worldLandmarks[j].visibility = 0.05; fr.landmarks[j].visibility = 0.05; } ou = pu.process({ t: i * 33.3, ...fr }); }
  const eu = Math.max(...['left.shoulder.elevation', 'left.elbow.flexion', 'left.forearm.pronation', 'right.shoulder.plane', 'right.shoulder.axialRotation'].map((a) => Math.abs(wrapDeg(get(ou.pose, a) - get(C.clampPose(pt2), a)))));
  ok(ou.pose.confidence['left.elbow.flexion'] > 0.5 && eu < 3, `upper-body-only framing: arms tracked (conf ${ou.pose.confidence['left.elbow.flexion'].toFixed(2)}, worst err ${eu.toFixed(2)}°)`);
  const body = prepareBody(synthesizeFrame(randomPose(), { dims: C.DEFAULT_DIMS }).worldLandmarks);
  const fixed = enforceSegmentLengths(body.points, dimsTrue);
  ok(Math.abs(dist(fixed[13], fixed[15]) - dimsTrue.forearm) < 1e-9 && Math.abs(dist(fixed[24], fixed[26]) - dimsTrue.thigh) < 1e-9, 'enforceSegmentLengths sets exact segment lengths');
}

// ---------------- 5. alignment ----------------
{
  let worstPx = 0, worstPersp = 0;
  for (let i = 0; i < 50; i++) {
    const p = randomPose(); p.spine.flexion = 0;
    const fr = synthesizeFrame(p, { width: 1280, height: 720, camDistance: 3.0, fovY: 50 });
    const res = computePose(prepareBody(fr.worldLandmarks), NOHOLD_OPTS);
    const al = computeAlignment({ pose: res.pose, dims: C.DEFAULT_DIMS, landmarks2D: fr.landmarks, width: 1280, height: 720, fovY: 50 });
    const P = C.stickFK(res.pose).positions;
    for (const [k, li] of [['shoulder_left', 11], ['wrist_right', 16], ['ankle_left', 27]]) {
      const pp = [P[k].x, P[k].y, P[k].z];
      const [u, v] = projectOrtho(al, pp); worstPx = Math.max(worstPx, Math.hypot(u - fr.landmarks[li].x * 1280, v - fr.landmarks[li].y * 720));
      const [u2, v2] = projectPerspective(al, pp); worstPersp = Math.max(worstPersp, Math.hypot(u2 - fr.landmarks[li].x * 1280, v2 - fr.landmarks[li].y * 720));
    }
  }
  console.log(`alignment (1280x720, subject 3 m, true perspective): ortho max reproj ${worstPx.toFixed(1)} px, perspective max reproj ${worstPersp.toFixed(2)} px`);
  ok(worstPersp < 2, 'perspective alignment reprojects joints within 2 px');
  ok(worstPx < 40, 'orthographic alignment reprojects joints within 40 px (weak-perspective error)');
  // mirrored input: alignment must map into the DECODED (mirrored) frame
  const p = randomPose();
  const fr = synthesizeFrame(p, { inputMirrored: true, width: 1280, height: 720 });
  const pl = createPipeline({ inputMirrored: true, angles: NOHOLD_OPTS });
  const out = pl.process({ t: 0, ...fr });
  const P = C.stickFK(out.pose, out.dims).positions;
  // subject's LEFT wrist appears in the mirrored frame where MediaPipe labels "right wrist" (16)
  const [u, v] = projectPerspective(out.alignment, [P.wrist_left.x, P.wrist_left.y, P.wrist_left.z]);
  const e = Math.hypot(u - fr.landmarks[16].x * 1280, v - fr.landmarks[16].y * 720);
  ok(e < 3, `mirrored-input alignment lands on the mirrored video (${e.toFixed(2)} px)`);
}

// ---------------- 5b. camera FOV auto-estimate + image-plane refinement ----------------
{
  const poseAt = (t) => { const p = base();
    for (const s of C.SIDES) { p[s].shoulder = { elevation: 60 + 40 * Math.sin(t), plane: 50 + 50 * Math.sin(0.7 * t), axialRotation: 20 * Math.sin(0.5 * t) }; p[s].elbow.flexion = 80 + 40 * Math.sin(1.3 * t); p[s].forearm.pronation = 60 * Math.sin(t); p[s].hip.flexion = 20 + 20 * Math.sin(0.5 * t); p[s].knee.flexion = 30 + 30 * Math.sin(0.5 * t); }
    return p; };
  const keys = ['left.shoulder.elevation', 'left.shoulder.axialRotation', 'left.elbow.flexion', 'left.forearm.pronation', 'right.knee.flexion'];
  for (const [fovCam, distM] of [[70, 1.6], [45, 3.0]]) {
    const rms = {};
    for (const refine of [0, 1]) {
      const nr = makeRng(8); const pl = createPipeline({ imageRefine: refine, angles: NOHOLD_OPTS }); let e = 0, n = 0;
      for (let i = 0; i < 150; i++) {
        const p = poseAt(i / 30);
        const o = pl.process({ t: i * 33.3, ...synthesizeFrame(p, { fovY: fovCam, camDistance: distM, width: 1280, height: 720, noise: 0.01, pixelNoise: 1.5, rng: nr }) });
        if (i > 60) for (const k of keys) { e += wrapDeg(get(o.debug.rawPose, k) - get(p, k)) ** 2; n++; }
      }
      rms[refine] = Math.sqrt(e / n);
      if (refine) { const f = pl.fovY; ok(f.sure && Math.abs(f.fov - fovCam) < 7, `FOV auto-estimate: camera ${fovCam}° at ${distM} m -> ${f.fov.toFixed(1)}°`); }
    }
    ok(rms[1] <= rms[0], `image-plane refinement (auto FOV) does not hurt: RMS ${rms[0].toFixed(2)}° -> ${rms[1].toFixed(2)}°`);
  }
}

// ---------------- 6. temporal filtering: jitter vs lag ----------------
{
  const fps = 30, T = 8, N = fps * T;
  const truthAt = (t) => {
    const p = base();
    p.root.quaternion = randomQuat(10 * Math.sin(0.3 * t), 0, 0);
    for (const side of C.SIDES) {
      const ph = side === 'left' ? 0 : 1.3;
      p[side].shoulder = { elevation: 50 + 30 * Math.sin(0.7 * t + ph), plane: 40 + 30 * Math.sin(0.4 * t), axialRotation: 20 * Math.sin(0.5 * t + ph) };
      p[side].elbow.flexion = 75 + 45 * Math.sin(2 * Math.PI * 0.5 * t + ph);          // 0.5 Hz bending
      p[side].forearm.pronation = 70 * Math.sin(2 * Math.PI * 0.4 * t + ph);           // pron/sup while bending
      p[side].wrist = { flexion: 20 * Math.sin(1.1 * t), ulnarDeviation: 10 };
      p[side].hip = { flexion: 30 + 25 * Math.sin(0.8 * t + ph), abduction: 10, axialRotation: 10 * Math.sin(0.6 * t) };
      p[side].knee.flexion = 40 + 30 * Math.sin(0.8 * t + ph);
      p[side].ankle = { dorsiflexion: 5, inversion: 0 };
    }
    p.spine = { flexion: 0, lateralBend: 8 * Math.sin(0.5 * t), axialRotation: 12 * Math.sin(0.35 * t) };
    p.neck = { flexion: 10 * Math.sin(0.9 * t), lateralBend: 0, axialRotation: 25 * Math.sin(0.6 * t) };
    return p;
  };
  const keys = ['left.shoulder.elevation', 'left.shoulder.axialRotation', 'left.elbow.flexion', 'left.forearm.pronation', 'right.forearm.pronation', 'left.hip.flexion', 'left.knee.flexion', 'neck.axialRotation', 'left.wrist.flexion'];
  const nrng = makeRng(99);
  const pl = createPipeline({ angles: NOHOLD_OPTS });
  const raw = {}, filt = {}, truth = {};
  for (let i = 0; i < N; i++) {
    const t = i / fps; const p = truthAt(t);
    const fr = synthesizeFrame(p, { noise: 0.01, handNoise: 0.003, pixelNoise: 2, rng: nrng });
    const out = pl.process({ t: t * 1000, ...fr });
    for (const k of keys) { (raw[k] ||= []).push(get(out.debug.rawPose, k)); (filt[k] ||= []).push(get(out.pose, k)); (truth[k] ||= []).push(get(p, k)); }
  }
  // unfiltered estimate from the same noisy data (no landmark filter, no stabiliser)
  const unf = {}; const nrng2 = makeRng(99);
  for (let i = 0; i < N; i++) {
    const t = i / fps; const p = truthAt(t);
    const fr = synthesizeFrame(p, { noise: 0.01, handNoise: 0.003, pixelNoise: 2, rng: nrng2 });
    const hands = { left: { points: prepareHand(fr.hands[0].worldLandmarks) }, right: { points: prepareHand(fr.hands[1].worldLandmarks) } };
    const res = computePose(prepareBody(fr.worldLandmarks), { ...NOHOLD_OPTS, hands });
    for (const k of keys) (unf[k] ||= []).push(get(res.pose, k));
  }
  const skip = fps; // ignore 1 s warm-up
  const rms = (a, b) => Math.sqrt(a.slice(skip).reduce((s, v, i) => s + wrapDeg(v - b[i + skip]) ** 2, 0) / (a.length - skip));
  const jit = (a) => Math.sqrt(a.slice(skip + 2).reduce((s, v, i) => s + (v - 2 * a[i + skip + 1] + a[i + skip]) ** 2, 0) / (a.length - skip - 2));
  const lag = (a, b) => { // best integer-frame shift of filtered vs truth (frames)
    let best = 0, bestE = Infinity;
    for (let L = 0; L <= 10; L++) { let e = 0; for (let i = skip; i < a.length; i++) e += wrapDeg(a[i] - b[i - L]) ** 2; if (e < bestE) { bestE = e; best = L; } }
    return best;
  };
  console.log('\n--- Temporal filtering (1 cm world noise, 3 mm hand noise, 2 px, 30 fps, 8 s) ---');
  console.log('angle'.padEnd(30) + 'RMS raw°'.padStart(10) + 'RMS filt°'.padStart(11) + 'jitter raw'.padStart(12) + 'jitter filt'.padStart(13) + 'lag ms'.padStart(8));
  let allBetter = true, maxLag = 0, jitRatio = 0;
  for (const k of keys) {
    const rr = rms(unf[k], truth[k]), rf = rms(filt[k], truth[k]), jr = jit(unf[k]), jf = jit(filt[k]), L = lag(filt[k], truth[k]) * 1000 / fps;
    console.log(k.padEnd(30) + rr.toFixed(2).padStart(10) + rf.toFixed(2).padStart(11) + jr.toFixed(2).padStart(12) + jf.toFixed(2).padStart(13) + L.toFixed(0).padStart(8));
    if (!(rf < rr)) allBetter = false;
    // One Euro deliberately smooths slow motion more; judge lag on fast-moving angles
    const peakVel = Math.max(...truth[k].slice(1).map((v, i) => Math.abs(v - truth[k][i]) * fps));
    if (peakVel > 60) maxLag = Math.max(maxLag, L);
    jitRatio = Math.max(jitRatio, jf / jr);
  }
  ok(allBetter, 'filtered RMS error < unfiltered RMS error for every tracked angle');
  ok(jitRatio < 0.35, `jitter (2nd difference) reduced ≥ ~3x (worst ratio ${jitRatio.toFixed(2)})`);
  ok(maxLag <= 70, `lag ≤ 70 ms for fast angles (peak > 60°/s) (worst ${maxLag.toFixed(0)} ms)`);

  // velocity limit: a one-frame 180° flip is rejected
  const pl2 = createPipeline({ angles: NOHOLD_OPTS });
  const p0 = base(); p0.left.elbow.flexion = 90; p0.left.forearm.pronation = -60;
  let outp;
  for (let i = 0; i < 30; i++) outp = pl2.process({ t: i * 33.3, ...synthesizeFrame(p0) });
  const flipped = structuredClone(p0); flipped.left.forearm.pronation = 80;
  outp = pl2.process({ t: 30 * 33.3, ...synthesizeFrame(flipped) });
  const jump = Math.abs(outp.pose.left.forearm.pronation - (-60));
  ok(jump <= 35, `single-frame 140° pronation jump limited to ${jump.toFixed(1)}° (max-velocity + One Euro)`);
  // angle filter wraps correctly
  const fa = new OneEuroAngle({ minCutoff: 1, beta: 0 }); fa.filter(179, 0); const v = fa.filter(-179, 0.033);
  ok(Math.abs(wrapDeg(v - 179)) < 2, 'OneEuroAngle handles ±180 wrap');
  ok(circularClamp(170, [-90, 135]) === 135 && circularClamp(-170, [-90, 135]) === 135 && circularClamp(-100, [-90, 135]) === -90, 'circular ROM clamp of shoulder plane is stable across ±180°');
  // hold & decay when a limb disappears (low visibility)
  const pl3 = createPipeline({ angles: NOHOLD_OPTS });
  const ph = base(); ph.left.elbow.flexion = 100;
  for (let i = 0; i < 30; i++) pl3.process({ t: i * 33.3, ...synthesizeFrame(ph) });
  let o3;
  for (let i = 30; i < 45; i++) { const fr = synthesizeFrame(base()); for (const j of [13, 15, 17, 19, 21]) fr.worldLandmarks[j].visibility = 0.05; o3 = pl3.process({ t: i * 33.3, ...fr }); }
  ok(o3.pose.left.elbow.flexion > 80, `occluded elbow holds last value (${o3.pose.left.elbow.flexion.toFixed(1)}° after 0.5 s)`);
}

console.log(failures ? `\n${failures} FAILED` : '\nALL PASS');
process.exit(failures ? 1 : 0);
