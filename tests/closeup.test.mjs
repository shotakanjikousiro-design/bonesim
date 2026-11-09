// node tests/closeup.test.mjs — close-up framings (head & shoulders, hand near the camera)
// Synthetic MediaPipe-format frames (src/tracking/synth.js) → pipeline → rig with the
// close-up overrides → perspective re-projection compared with the 2D landmarks.
import fs from 'fs';
import path from 'path';
import { fileURLToPath } from 'url';
import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { createRig } from '../src/rig/anatomicalRig.js';
import { anatomicalPose, stickFK } from '../src/shared/conventions.js';
import { synthesizeFrame } from '../src/tracking/synth.js';
import { createPipeline } from '../src/tracking/pipeline.js';
import { classifyFraming, defaultFovY, HEAD_MODEL } from '../src/tracking/closeup.js';
import { computeAlignment } from '../src/tracking/alignment.js';

const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const rigData = JSON.parse(fs.readFileSync(path.join(ROOT, 'assets/rig.json'), 'utf8'));
const buf = fs.readFileSync(path.join(ROOT, 'assets/skeleton.glb'));
const gltf = await new Promise((res, rej) => new GLTFLoader().parse(buf.buffer.slice(buf.byteOffset, buf.byteOffset + buf.byteLength), '', res, rej));
const rig = await createRig({ gltf, rigData, skin: false });

let fails = 0;
const check = (name, ok, detail = '') => { console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}${detail ? '  — ' + detail : ''}`); if (!ok) fails++; };
const W = 720, H = 1280; const FOV = defaultFovY(W, H); const f = (H / 2) / Math.tan(FOV * Math.PI / 360);
const proj = (p) => [W / 2 + f * p.x / -p.z, H / 2 - f * p.y / -p.z];
const px = (l) => [l.x * W, l.y * H];
const d2 = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1]);

/** run the pipeline on a static synthetic pose for n frames (≈30 fps) */
function run(pose, synthOpts, n = 40, anatomicalFace = false) {
  const pl = createPipeline({});
  let res, fr;
  for (let i = 0; i < n; i++) {
    fr = synthesizeFrame(pose, { width: W, height: H, fovY: FOV, ...synthOpts });
    if (anatomicalFace) faceFromHeadModel(fr, pose, synthOpts);
    res = pl.process({ t: i * 33, width: W, height: H, landmarks: fr.landmarks, worldLandmarks: fr.worldLandmarks, hands: fr.hands });
  }
  return { res, fr };
}
/**
 * synth.js draws a schematic face (nose level with the ears, ears 15 cm apart). For the
 * head-fit test, replace the face landmarks with an anatomical face: HEAD_MODEL anchors
 * around an atlanto-occipital point AO = neckBase + R_head·(0, 0.163, 0.009), projected with
 * synth.js' camera model (pelvis at camDistance, shifted to `center`).
 */
function faceFromHeadModel(fr, pose, { camDistance = 2.8, center = [0.5, 0.55] }) {
  const { positions: P, quaternions: Q } = stickFK(pose);
  // AO ≈ 16 cm above / 1 cm behind the shoulder line in the rig's (anatomical) skeleton
  const ao = P.neckBase.clone().add(new THREE.Vector3(0, 0.163, 0.009).applyQuaternion(Q.head));
  const origin = P.hip_left.clone().add(P.hip_right).multiplyScalar(0.5);
  const off = new THREE.Vector3((center[0] - 0.5) * W * camDistance / f, -(center[1] - 0.5) * H * camDistance / f, -camDistance);
  const at = (a) => { const p = new THREE.Vector3(...a).applyQuaternion(Q.head).add(ao).sub(origin).add(off); return { x: 0.5 + f * p.x / -p.z / W, y: 0.5 - f * p.y / -p.z / H }; };
  const M = HEAD_MODEL; const eye = (k, dx) => [M[k][0] + dx, M[k][1], M[k][2]];
  const set = { 0: M[0], 2: M[2], 5: M[5], 7: M[7], 8: M[8], 9: M[9], 10: M[10], 1: eye(2, -0.014), 3: eye(2, 0.014), 4: eye(5, 0.014), 6: eye(5, -0.014) };
  for (const [k, a] of Object.entries(set)) Object.assign(fr.landmarks[k], at(a));
}

/** put the rig in the camera frame the way cameraMode does for perspective framings */
function placeRig(res, { useHead = true, useHands = false } = {}) {
  rig.scaleToDims(res.dims);
  rig.setPose(res.pose, { useRoot: true });
  let T = new THREE.Vector3();
  if (res.framing !== 'hand') {
    const al = res.alignment;
    if (al?.valid && al.nFit >= 2) T.fromArray(al.perspective.position);
    else if (res.head) T.fromArray(res.head.ao).sub(rig.rootLocalPosition('skull'));
  }
  rig.root.matrixAutoUpdate = false; rig.root.matrix.makeTranslation(T.x, T.y, T.z); rig.root.updateMatrixWorld(true);
  const cu = {};
  if (useHead && res.head) cu.head = { ao: new THREE.Vector3().fromArray(res.head.ao).sub(T).toArray(), quaternion: res.head.quaternion };
  if (useHands) { cu.hands = {}; for (const h of res.hands) if (h.cameraPoints) cu.hands[h.side] = h.cameraPoints.map((p) => [p[0] - T.x, p[1] - T.y, p[2] - T.z]); }
  rig.setCloseup(Object.keys(cu).length ? cu : null); rig.root.updateMatrixWorld(true);
  return T;
}
const world = (segId) => new THREE.Vector3().setFromMatrixPosition(rig.segments[segId].group.matrixWorld);
/** projected horizontal width (px) of the skull at the ear (Frankfort) level */
function skullWidthPx() {
  const sk = rig.meshes.skull; const pos = sk.geometry.attributes.position; const ao = rigData.joints.atlantooccipital.center;
  let x0 = Infinity, x1 = -Infinity; const v = new THREE.Vector3();
  for (let i = 0; i < pos.count; i++) {
    v.fromBufferAttribute(pos, i); if (Math.abs(v.y - (ao[1] + 0.025)) > 0.006) continue;
    const p = proj(v.applyMatrix4(sk.matrixWorld)); x0 = Math.min(x0, p[0]); x1 = Math.max(x1, p[0]);
  }
  return x1 - x0;
}
/** projected rig face anchors (eyes/ears/nose/mouth) vs the 2D landmarks */
function faceError(lm) {
  const M = rig.segments.skull.group.matrixWorld; const e = [];
  for (const [k, a] of Object.entries(HEAD_MODEL)) {
    // HEAD_MODEL offsets are in the neutral head frame relative to the AO joint; the skull group
    // origin is the AO joint and its holder holds the rest geometry → offset is in group-local frame
    const p = new THREE.Vector3(...a).applyMatrix4(M);
    e.push(d2(proj(p), px(lm[k])));
  }
  return e;
}

// ---------------- 1. head & shoulders (selfie, hips out of the image) ----------------
{
  const pose = anatomicalPose(); pose.left.elbow.flexion = 20; pose.right.elbow.flexion = 20; pose.neck.flexion = 5;
  const D = 0.55; // pelvis/face plane distance from the camera (selfie at arm's length)
  const fk = stickFK(pose).positions; const nose = fk.nose;
  const center = [0.5 - nose.x * f / (W * D), 0.5 + (nose.y + 0.06) * f / (H * D)];
  const { res, fr } = run(pose, { camDistance: D, center, noise: 0.004, pixelNoise: 0.8 }, 40, true);
  const L = fr.landmarks;
  check('synthetic selfie: hips outside the image', L[23].y > 1.05 && L[24].y > 1.05, `hip y = ${L[23].y.toFixed(2)}`);
  check('framing = 顔・肩 (headShoulders)', res.framing === 'headShoulders' && classifyFraming(L, []) === 'headShoulders', res.framing);
  check('auto dims not learned from the close-up', res.dimsSource === 'default', res.dimsSource);
  check('alignment fitted only to visible in-image joints', res.alignment?.nFit >= 2 && res.alignment.nFit <= 6, `nFit ${res.alignment?.nFit}`);
  const T = placeRig(res);
  const earPx = d2(px(L[7]), px(L[8]));
  const ratio = skullWidthPx() / earPx;
  check('skull bone width at ear level ≈ ear-to-ear distance (0.78–0.95 ×; tragus has ≈1 cm soft tissue)', ratio > 0.78 && ratio < 0.95, `skull ${skullWidthPx().toFixed(0)} px / ears ${earPx.toFixed(0)} px = ${ratio.toFixed(2)}`);
  const fe = faceError(L); const feMax = Math.max(...fe);
  check('face anchors re-project onto eyes/ears/nose/mouth (max < 4 % of ear distance)', feMax < 0.04 * earPx, `max ${feMax.toFixed(1)} px, mean ${(fe.reduce((a, b) => a + b) / fe.length).toFixed(1)} px`);
  const sh = [d2(proj(world('humerus_L')), px(L[11])), d2(proj(world('humerus_R')), px(L[12]))];
  check('shoulders re-project onto the shoulder landmarks (< 10 % of ear distance)', Math.max(...sh) < 0.10 * earPx, `${sh.map((x) => x.toFixed(1)).join(' / ')} px`);
  // the cervical chain connects thorax and skull (no gap) and stays plausible
  const ao = world('skull'), c7 = world('C7');
  check('neck connects thorax to skull with a plausible length (8–20 cm)', ao.distanceTo(c7) > 0.08 && ao.distanceTo(c7) < 0.20, `${(ao.distanceTo(c7) * 100).toFixed(1)} cm`);
  // what the old fit did: hallucinated (off-image) hips/legs included, single ortho scale
  const old = computeAlignment({ pose: res.pose, dims: res.dims, landmarks2D: res.landmarks2D.normalized, width: W, height: H, fovY: FOV, margin: 100 });
  console.log(`info  legacy ortho fit incl. off-image joints: scale ${old?.ortho.scale.toFixed(0)} px/m vs perspective head depth ${(-res.head.ao[2]).toFixed(2)} m`);
  rig.setCloseup(null);
}

// ---------------- 2. hand close to the camera ----------------
{
  const pose = anatomicalPose();
  pose.right.shoulder = { elevation: 90, plane: 90, axialRotation: 0 }; pose.right.forearm.pronation = -60;
  pose.right.fingers = { thumb: { cmcFlexion: 5, cmcAbduction: 20, mcp: 10, ip: 10 }, index: { mcp: 20, pip: 25, dip: 15 }, middle: { mcp: 25, pip: 30, dip: 20 }, ring: { mcp: 30, pip: 35, dip: 20 }, little: { mcp: 35, pip: 40, dip: 25 } };
  const fk = stickFK(pose).positions; const hc = fk.wrist_right.clone().lerp(fk.handTip_right, 0.45);
  const D = hc.z + 0.24; // hand ≈ 24 cm from the camera
  const center = [0.5 - hc.x * f / (W * D), 0.5 + hc.y * f / (H * D)];
  const { res } = run(pose, { camDistance: D, center, handNoise: 0.002, pixelNoise: 0.5 });
  const h = res.hands.find((x) => x.side === 'right');
  check('framing = 手 (hand) for a hand near the camera', res.framing === 'hand', res.framing);
  check('hand placed in the camera frame', !!h?.cameraPoints, h ? `wrist depth ${(-h.cameraPoints[0][2]).toFixed(2)} m` : 'none');
  if (h?.cameraPoints) {
    placeRig(res, { useHead: false, useHands: true });
    const L = h.landmarks2D.map(px), C3 = h.cameraPoints.map((p) => new THREE.Vector3(...p));
    const joints = [['indexProx_R', 5], ['indexMid_R', 6], ['indexDist_R', 7], ['middleProx_R', 9], ['middleMid_R', 10], ['middleDist_R', 11],
      ['ringProx_R', 13], ['ringMid_R', 14], ['ringDist_R', 15], ['littleProx_R', 17], ['littleMid_R', 18], ['littleDist_R', 19],
      ['thumbMeta_R', 1], ['thumbProx_R', 2], ['thumbDist_R', 3]];
    const errs = joints.map(([seg, i]) => d2(proj(world(seg)), L[i]));
    const tips = [['indexTip_R', 8], ['middleTip_R', 12], ['ringTip_R', 16], ['littleTip_R', 20], ['thumbTip_R', 4]].map(([n, i]) => d2(proj(rig.landmarkWorld(n)), L[i]));
    const wristErr = d2(proj(world('carpalsProximal_R')), L[0]);
    const handLenPx = d2(L[0], L[12]);
    check('finger joints re-project onto the hand landmarks (max < 3 % of hand length)', Math.max(...errs) < 0.03 * handLenPx, `max ${Math.max(...errs).toFixed(1)} px, hand ${handLenPx.toFixed(0)} px`);
    check('fingertips re-project onto the tip landmarks (max < 3 % of hand length)', Math.max(...tips) < 0.03 * handLenPx, `max ${Math.max(...tips).toFixed(1)} px`);
    check('carpals at the wrist landmark (< 6 % of hand length)', wristErr < 0.06 * handLenPx, `${wristErr.toFixed(1)} px`);
    // bone length vs landmark segment length (3-D)
    const pairs = [['indexProx_R', 'indexMid_R', 5, 6], ['middleMid_R', 'middleDist_R', 10, 11], ['thumbProx_R', 'thumbDist_R', 2, 3]];
    const ratios = pairs.map(([a, b, i, j]) => world(a).distanceTo(world(b)) / C3[i].distanceTo(C3[j]));
    check('hand bone lengths = landmark segment lengths (±3 %)', ratios.every((r) => Math.abs(r - 1) < 0.03), ratios.map((r) => r.toFixed(3)).join(', '));
    const met = world('middleProx_R').distanceTo(world('carpalsProximal_R')) / C3[9].distanceTo(C3[0]);
    check('metacarpal/carpal block scaled to the wrist→MCP distance (±12 %)', Math.abs(met - 1) < 0.12, met.toFixed(3));
  }
  rig.setCloseup(null);
}

// ---------------- 3. full body unchanged ----------------
{
  const pose = anatomicalPose(); pose.left.shoulder.elevation = 40;
  const { res } = run(pose, { width: 640, height: 480, camDistance: 3.0, center: [0.5, 0.52] });
  check('full body: framing = 全身 and dims learning allowed', res.framing === 'full', `${res.framing}, dims ${res.dimsSource}`);
}

console.log(fails ? `\n${fails} FAILED` : '\nALL PASS');
process.exit(fails ? 1 : 0);
