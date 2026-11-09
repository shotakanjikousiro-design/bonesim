// node tests/rig.test.mjs — anatomical rig checks (loads the real GLB in node)
import fs from 'fs';
import path from 'path';
import { fileURLToPath } from 'url';
import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { createRig, completePose, shaftAxis } from '../src/rig/anatomicalRig.js';
import * as C from '../src/rig/coupling.js';
import { stickFK, anatomicalPose, humerusDirection, SIDES } from '../src/shared/conventions.js';

const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const rigData = JSON.parse(fs.readFileSync(path.join(ROOT, 'assets/rig.json'), 'utf8'));
const buf = fs.readFileSync(path.join(ROOT, 'assets/skeleton.glb'));
const gltf = await new Promise((res, rej) => new GLTFLoader().parse(buf.buffer.slice(buf.byteOffset, buf.byteOffset + buf.byteLength), '', res, rej));

let fails = 0;
const check = (name, ok, detail = '') => { console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}${detail ? '  — ' + detail : ''}`); if (!ok) fails++; };
const deg = (a, b) => THREE.MathUtils.radToDeg(a.angleTo(b));
const SFX = { left: 'L', right: 'R' };
const sgn = (side) => (side === 'left' ? 1 : -1);

const t0 = Date.now();
const rig = await createRig({ gltf, rigData, skin: true });
console.log(`rig built in ${Date.now() - t0} ms (incl. skin binding)`);
const update = () => rig.root.updateMatrixWorld(true);

// ---------- pure coupling functions ----------
{
  const g = (e, p = 0) => C.shoulderGirdleAngles(e, p).scapula.upwardRotation;
  check('girdle: setting phase (≤20°) → upward rotation < 1°', g(20) < 1, g(20).toFixed(2));
  check('girdle: 180° → 60° upward rotation (GH:ST 2:1)', Math.abs(g(180) - 60) < 0.5, g(180).toFixed(1));
  check('girdle: monotonic', [0, 30, 60, 90, 120, 150, 180].every((e, i, a) => i === 0 || g(e) >= g(a[i - 1])));
  const ce = C.shoulderGirdleAngles(90).clavicle.elevation, ceI = C.shoulderGirdleAngles(90, 0, C.GIRDLE_PARAMS_INMAN).clavicle.elevation;
  check('clavicle elevation at 90°: default ≈ 0.6°/10° (Ludewig magnitude), Inman option ≈ 4°/10°', ce > 4 && ce <= 6 && ceI > 30 && ceI <= 36, `${ce.toFixed(1)}° / Inman ${ceI.toFixed(1)}°`);
  const gf = C.shoulderGirdleAngles(120, 90);
  check('clavicle retracts (not protracts) with elevation, also in flexion', gf.clavicle.protraction < -10, `${gf.clavicle.protraction.toFixed(1)}°`);
  const gx = C.shoulderGirdleAngles(50, -90);
  check('extension (plane −90°): no upward rotation, slight downward rotation / anterior tilt', gx.scapula.upwardRotation <= 0 && gx.scapula.posteriorTilt <= 0 && gx.clavicle.elevation === 0,
    `upward ${gx.scapula.upwardRotation.toFixed(1)}°, tilt ${gx.scapula.posteriorTilt.toFixed(1)}°`);
  check('screw-home: −10° at 0°, 0 at ≥25°', Math.abs(C.screwHomeRotation(0) + 10) < 1e-9 && C.screwHomeRotation(30) === 0);
  const w = C.wristSplit(50, 20), we = C.wristSplit(-60, 0);
  check('wrist split: palmar flexion 40:60, dorsiflexion 66.5:33.5, deviation 60:40 (Sarrafian 1977)',
    Math.abs(w.radiocarpal.flexion - 20) < 1e-9 && Math.abs(w.midcarpal.ulnarDeviation - 8) < 1e-9 && Math.abs(we.radiocarpal.flexion + 39.9) < 1e-9);
  for (const k of ['flexion', 'lateralBend', 'axialRotation']) {
    const s1 = C.SPINE_WEIGHTS[k].reduce((a, b) => a + b, 0), s2 = C.NECK_WEIGHTS[k].reduce((a, b) => a + b, 0);
    check(`spine/neck weights sum to 1 (${k})`, Math.abs(s1 - 1) < 1e-9 && Math.abs(s2 - 1) < 1e-9);
  }
  const root = C.solvePatellaAngle((x) => x - 37.3, 30);
  check('patella root finder', Math.abs(root - 37.3) < 1e-6);
}

// ---------- rest pose leaves vertices unchanged ----------
const restPoseObj = () => {
  const p = anatomicalPose();
  for (const side of SIDES) {
    const r = rig.restPose[side]; // rig.json restPose + the humeral rotation the rig derives
    for (const j in r) Object.assign(p[side][j], r[j]);
  }
  return p;
};
{
  rig.setPose(restPoseObj(), { coupling: true }); update();
  // the forearm chain (ulna and all its descendants) carries the deliberate carrying-angle
  // correction (see anatomicalRig.js "elbow"); everything else must be untouched
  const forearmChain = new Set(); for (const seg of rig.order) if (seg.id.startsWith('ulna_') || (seg.parent && forearmChain.has(seg.parent))) forearmChain.add(seg.id);
  let maxErr = 0; const v = new THREE.Vector3(), w = new THREE.Vector3();
  for (const m of Object.values(rig.meshes)) {
    if (forearmChain.has(m.userData.segment)) continue;
    const pos = m.geometry.attributes.position;
    for (let i = 0; i < pos.count; i += 37) { v.fromBufferAttribute(pos, i); w.copy(v).applyMatrix4(m.matrixWorld); maxErr = Math.max(maxErr, v.distanceTo(w)); }
  }
  check('rest pose leaves every vertex unchanged (except the corrected forearm chain)', maxErr < 1e-6, `max ${(maxErr * 1000).toExponential(2)} mm`);
  for (const side of SIDES) {
    const e = rig.elbowInfo(side);
    check(`${side} elbow model: mesh rest carrying angle measured`, Number.isFinite(e.restCarrying), `mesh ${e.restCarrying.toFixed(1)}° → model ${e.carrying}° (shafts); axis tilt ${e.tilt.toFixed(1)}° vs mechanical axis / ${e.tiltVsShaft.toFixed(1)}° vs shaft; rest humeral rotation ${e.restHumeralRotation}°`);
  }
}

// ---------- anatomical pose vs stickFK ----------
function forearmChainIds() { const f = new Set(); for (const seg of rig.order) if (seg.id.startsWith('ulna_') || (seg.parent && f.has(seg.parent))) f.add(seg.id); return f; }
const J = (seg) => rig.jointWorld(seg);
const LM = (n) => rig.landmarkWorld(n);
{
  const pose = anatomicalPose();
  rig.setPose(pose); update();
  const fk = stickFK(pose).positions;
  for (const side of SIDES) {
    const x = SFX[side]; const k = (n) => fk[`${n}_${side}`];
    const pairs = [
      ['upper arm', J(`humerus_${x}`), J(`ulna_${x}`), k('shoulder'), k('elbow')],
      ['forearm', J(`ulna_${x}`), J(`carpalsProximal_${x}`), k('elbow'), k('wrist')],
      ['hand (wrist→3rd MCP)', J(`carpalsProximal_${x}`), J(`middleProx_${x}`), k('wrist'), k('handTip')],
      ['thigh', J(`femur_${x}`), J(`tibia_${x}`), k('hip'), k('knee')],
      ['shank', J(`tibia_${x}`), J(`talus_${x}`), k('knee'), k('ankle')],
      ['foot', LM(`heel_${x}`), LM(`toeTip_${x}`), k('heel'), k('toe')],
    ];
    for (const [n, a, b, c, d] of pairs) {
      const e = deg(b.clone().sub(a), d.clone().sub(c));
      check(`anatomical ${side} ${n} direction vs stickFK`, e < 15, `${e.toFixed(1)}°`);
    }
    const rs = LM(`radialStyloid_${x}`), us = LM(`ulnarStyloid_${x}`);
    check(`anatomical ${side}: radial styloid lateral (palms forward)`, sgn(side) * (rs.x - us.x) > 0.01);
  }
}

// ---------- elbow flexion 90 ----------
for (const side of SIDES) {
  const x = SFX[side]; const p = anatomicalPose(); p[side].elbow.flexion = 90;
  rig.setPose(p); update();
  const fa = J(`carpalsProximal_${x}`).sub(J(`ulna_${x}`)).normalize();
  check(`${side} elbow 90° → forearm anterior`, fa.z > 0.9, `dir ${fa.toArray().map((v) => v.toFixed(2))}`);
}

// ---------- carrying angle (cubitus valgus) ----------
for (const side of SIDES) {
  const x = SFX[side], s = sgn(side);
  const shaft = (segs, a, b) => { const A = J(a), B = J(b); const d = B.clone().sub(A); const len = d.length();
    return shaftAxis(segs.flatMap((id) => rig.segments[id].meshes), A, d.normalize(), len, { matrixOf: (m) => m.matrixWorld }); };
  const coronal = (h, f) => { // signed angle humerus→forearm in the coronal plane, + = forearm lateral (valgus)
    const H = new THREE.Vector2(h.x * s, -h.y).normalize(), F = new THREE.Vector2(f.x * s, -f.y).normalize();
    return THREE.MathUtils.radToDeg(Math.atan2(H.x * F.y - H.y * F.x, H.dot(F))) * -1; };
  const p = anatomicalPose(); rig.setPose(p); update();
  const hum = shaft([`humerus_${x}`], `humerus_${x}`, `ulna_${x}`);
  const uln = shaft([`ulna_${x}`], `ulna_${x}`, `carpalsProximal_${x}`);
  const fa = shaft([`ulna_${x}`, `radius_${x}`], `ulna_${x}`, `carpalsProximal_${x}`);
  const cU = coronal(hum, uln), cF = coronal(hum, fa);
  const cJ = coronal(J(`ulna_${x}`).sub(J(`humerus_${x}`)), J(`carpalsProximal_${x}`).sub(J(`ulna_${x}`)));
  check(`${side} carrying angle (anatomical position) 8–15° valgus`, cU >= 8 && cU <= 15 && cF >= 8 && cF <= 15,
    `humerus–ulna shafts ${cU.toFixed(1)}°, humerus–forearm ${cF.toFixed(1)}°, joint centres ${cJ.toFixed(1)}°`);
  const p90 = anatomicalPose(); p90[side].elbow.flexion = 90; rig.setPose(p90); update();
  const f90 = J(`carpalsProximal_${x}`).sub(J(`ulna_${x}`)).normalize();
  const dev = THREE.MathUtils.radToDeg(Math.atan2(f90.x * s, f90.z));
  check(`${side} elbow 90°: forearm within ±5° of sagittal`, Math.abs(dev) <= 5, `${dev.toFixed(1)}° (+ = lateral)`);
  const p140 = anatomicalPose(); p140[side].elbow.flexion = 140; rig.setPose(p140); update();
  const med = s * (J(`carpalsProximal_${x}`).x - J(`ulna_${x}`).x);
  check(`${side} elbow 140°: wrist medial to the elbow (carrying angle disappears)`, med < 0, `${(med * 1000).toFixed(1)} mm (+ = lateral)`);
}

// ---------- pronation: radius crosses ulna, ulna fixed ----------
for (const side of SIDES) {
  const x = SFX[side];
  const p = anatomicalPose(); rig.setPose(p); update();
  const rs0 = LM(`radialStyloid_${x}`), us0 = LM(`ulnarStyloid_${x}`);
  const ulnaM0 = rig.segments[`ulna_${x}`].holder.matrixWorld.clone();
  p[side].forearm.pronation = 90; rig.setPose(p); update();
  const rs1 = LM(`radialStyloid_${x}`), us1 = LM(`ulnarStyloid_${x}`);
  const ulnaM1 = rig.segments[`ulna_${x}`].holder.matrixWorld;
  const ulnaSame = ulnaM0.elements.every((e, i) => Math.abs(e - ulnaM1.elements[i]) < 1e-9);
  check(`${side} pronation 90: radial styloid moves lateral → medial of ulnar styloid`,
    sgn(side) * (rs0.x - us0.x) > 0.01 && sgn(side) * (rs1.x - us1.x) < -0.01,
    `lat. offset ${(1000 * sgn(side) * (rs0.x - us0.x)).toFixed(0)} → ${(1000 * sgn(side) * (rs1.x - us1.x)).toFixed(0)} mm`);
  check(`${side} pronation: ulna unchanged`, ulnaSame);
  check(`${side} pronation: ulnar styloid unchanged`, us0.distanceTo(us1) < 1e-9);
  const hand = rig.segments[`hand_${x}`]; void hand;
}

// ---------- shoulder abduction 150 ----------
for (const side of SIDES) {
  const x = SFX[side];
  const p0 = anatomicalPose(); rig.setPose(p0); update();
  const scap0 = rig.segmentWorldQuaternion(`scapula_${x}`);
  const ia0 = LM(`inferiorAngleScapula_${x}`);
  const p = anatomicalPose(); p[side].shoulder = { elevation: 150, plane: 0, axialRotation: 0 };
  const d = rig.setPose(p); update();
  const scap1 = rig.segmentWorldQuaternion(`scapula_${x}`);
  const rel = scap1.clone().multiply(scap0.clone().invert());
  const up = Math.abs(C.twistAngle(rel, rigData.joints[`acromioclavicular_${x}`].axes.upwardRotation));
  check(`${side} abduction 150: scapular upward rotation 40–60°`, up >= 40 && up <= 60, `${up.toFixed(1)}° (model ${d[side].scapularUpwardRotation.toFixed(1)}°)`);
  const ia1 = LM(`inferiorAngleScapula_${x}`);
  check(`${side} abduction 150: inferior angle moves laterally`, sgn(side) * (ia1.x - ia0.x) > 0.03, `${(1000 * sgn(side) * (ia1.x - ia0.x)).toFixed(0)} mm`);
  const hum = J(`ulna_${x}`).sub(J(`humerus_${x}`));
  const e = deg(hum, humerusDirection(side, 150, 0));
  const pen = rig.scapulaRibPenetration(side);
  check(`${side} abduction 150: scapula stays on the rib cage`, pen.maxDepth < 0.004, `max depth ${(pen.maxDepth * 1000).toFixed(1)} mm, correction IR ${d[side].scapulaRibCorrection?.toFixed(1)}° / tilt ${d[side].scapulaRibTiltCorrection?.toFixed(1)}°`);
  check(`${side} abduction 150: humerus matches humerusDirection() within 3°`, e < 3, `${e.toFixed(2)}°`);
  for (const [el, pl] of [[90, 90], [120, 45], [60, -60], [170, 90]]) {
    const pp = anatomicalPose(); pp[side].shoulder = { elevation: el, plane: pl, axialRotation: 20 };
    rig.setPose(pp); update();
    const ee = deg(J(`ulna_${x}`).sub(J(`humerus_${x}`)), humerusDirection(side, el, pl));
    check(`${side} shoulder ${el}/${pl}: humerus direction within 3°`, ee < 3, `${ee.toFixed(2)}°`);
  }
  { // GH centre rise at 90° abduction
    rig.setPose(anatomicalPose()); update(); const g0 = J(`humerus_${x}`).y; const p90 = anatomicalPose(); p90[side].shoulder = { elevation: 90, plane: 0, axialRotation: 0 };
    rig.setPose(p90); update(); const rise = J(`humerus_${x}`).y - g0;
    const pen90 = rig.scapulaRibPenetration(side);
    check(`${side} abduction 90: GH centre rises 10–25 mm`, rise > 0.010 && rise < 0.025, `${(rise * 1000).toFixed(1)} mm`);
    check(`${side} abduction 90: scapula stays on the rib cage`, pen90.maxDepth < 0.004, `max depth ${(pen90.maxDepth * 1000).toFixed(1)} mm`);
    rig.setPose(p); update();
  }
  // coupling OFF → scapula stays
  rig.setPose(p, { coupling: false }); update();
  const off = rig.segmentWorldQuaternion(`scapula_${x}`).angleTo(scap0);
  check(`${side} coupling OFF: scapula does not rotate`, off < 1e-6);
  const e2 = deg(J(`ulna_${x}`).sub(J(`humerus_${x}`)), humerusDirection(side, 150, 0));
  check(`${side} coupling OFF: humerus still matches`, e2 < 3, `${e2.toFixed(2)}°`);
  rig.setPose(anatomicalPose(), { coupling: true });
}

// ---------- knee screw-home & patella ----------
for (const side of SIDES) {
  const x = SFX[side];
  const tw = (flex) => {
    const p = anatomicalPose(); p[side].knee.flexion = flex; rig.setPose(p); update();
    const qt = rig.segments[`tibia_${x}`].group.quaternion.clone();
    const r0 = rig.restPose[side].knee.flexion;
    const qf = C.axisAngleQ(rigData.joints[`knee_${x}`].axes.flexion, flex - r0);
    return C.twistAngle(qf.invert().multiply(qt), rigData.joints[`knee_${x}`].axes.axialRotation);
  };
  const dRot = tw(30) - tw(0);
  check(`${side} knee 0 vs 30: tibial rotation difference 5–12° (internal on flexion)`, dRot >= 5 && dRot <= 12, `${dRot.toFixed(1)}°`);
  const { restTendonLength: Lr, tautTendonLength: Lt } = rig.patellaInfo(side);
  let maxDev = 0, maxLen = 0, inside = 0, total = 0; const lens = [];
  const fem = rig.meshes[`femur_${x}`], pat = rig.meshes[`patella_${x}`];
  const verts = (m, step) => { const p = m.geometry.attributes.position, n = m.geometry.attributes.normal, o = [];
    for (let i = 0; i < p.count; i += step) o.push([new THREE.Vector3().fromBufferAttribute(p, i).applyMatrix4(m.matrixWorld), new THREE.Vector3().fromBufferAttribute(n, i).transformDirection(m.matrixWorld)]); return o; };
  for (let f = 0; f <= 120; f += 10) {
    const p = anatomicalPose(); p[side].knee.flexion = f; const dd = rig.setPose(p); update();
    const L = LM(`patellaApex_${x}`).distanceTo(LM(`tibialTuberosity_${x}`));
    lens.push((L * 1000).toFixed(1)); maxLen = Math.max(maxLen, L);
    if (dd[side].patellaTendonTaut) maxDev = Math.max(maxDev, Math.abs(L - Lt));
    if (f % 30 === 0) { // patella must not cut into the femur or the tibia
      for (const bone of [fem, rig.meshes[`tibia_${x}`]]) {
        const F = verts(bone, 1);
        for (const [pp] of verts(pat, 3)) { let best = Infinity, bn = null; for (const fv of F) { const d2 = fv[0].distanceToSquared(pp); if (d2 < best) { best = d2; bn = fv; } } total++; if (pp.clone().sub(bn[0]).dot(bn[1]) < -0.0015) inside++; }
      }
    }
  }
  check(`${side} patellar tendon: constant (±2 mm) once taut, never stretched`, maxDev < 0.002 && maxLen < Lt + 0.002,
    `rest ${(Lr * 1000).toFixed(1)} mm (slack), taut ${(Lt * 1000).toFixed(1)} mm; lengths ${lens.join(',')}`);
  check(`${side} patella does not penetrate the femur / tibia (0–120°)`, inside / total < 0.03, `${inside}/${total} vertices > 1.5 mm inside`);
  const p = anatomicalPose(); p[side].knee.flexion = 90; const d = rig.setPose(p);
  check(`${side} patellar flexion at 90° knee flexion ≈ 0.7× (50–75°)`, d[side].patellaRotation > 50 && d[side].patellaRotation < 75, `patellar flexion ${d[side].patellaRotation?.toFixed(1)}°, glide ${d[side].patellaGlide?.toFixed(1)}°`);
}

// ---------- joints remain connected ----------
function connectivity(label) {
  update(); let maxErr = 0; let worst = '';
  for (const seg of rig.order) {
    if (!seg.parent || seg.id.startsWith('patella')) continue; // patella glides (not a fixed pivot)
    const par = rig.segments[seg.parent];
    const expected = seg.center.clone().applyMatrix4(par.holder.matrixWorld);
    const actual = new THREE.Vector3().setFromMatrixPosition(seg.group.matrixWorld);
    const e = expected.distanceTo(actual); if (e > maxErr) { maxErr = e; worst = seg.id; }
  }
  check(`joints connected (${label})`, maxErr < 1e-6, `max ${(maxErr * 1000).toExponential(2)} mm at ${worst}`);
}
const bigPose = () => completePose({
  spine: { flexion: 30, lateralBend: -15, axialRotation: 25 }, neck: { flexion: 20, lateralBend: 10, axialRotation: 40 },
  left: { shoulder: { elevation: 120, plane: 60, axialRotation: 30 }, elbow: { flexion: 100 }, forearm: { pronation: 60 }, wrist: { flexion: 40, ulnarDeviation: 20 },
    hip: { flexion: 70, abduction: 15, axialRotation: 10 }, knee: { flexion: 80 }, ankle: { dorsiflexion: 10, inversion: 15 },
    fingers: C.fistFingers(1) },
  right: { shoulder: { elevation: 160, plane: 0, axialRotation: -30 }, elbow: { flexion: 30 }, knee: { flexion: 5 } },
});
rig.setPose(bigPose()); connectivity('big pose');
// spine: thorax orientation exactly spineQ
{
  const p = bigPose(); rig.setPose(p); update();
  const fk = stickFK(p).quaternions;
  const e = THREE.MathUtils.radToDeg(rig.segmentWorldQuaternion('thorax').angleTo(fk.thorax));
  const eh = THREE.MathUtils.radToDeg(rig.segmentWorldQuaternion('skull').angleTo(fk.head));
  check('spine distribution: thorax orientation equals spineQ', e < 1e-3, `${e.toExponential(2)}°`);
  check('neck distribution: head orientation equals stickFK head', eh < 1e-3, `${eh.toExponential(2)}°`);
  const c12 = rig.segments.C1.group.quaternion; // C1C2 joint share of rotation
  const tw = C.twistAngle(c12, [0, 1, 0]);
  check('neck rotation: C1–C2 carries ≈ 50%', Math.abs(tw - 20) < 1.5, `${tw.toFixed(1)}° of 40°`);
}
// scaling
rig.scaleToDims({ ...rigData.dims, upperArm: rigData.dims.upperArm * 1.15, forearm: rigData.dims.forearm * 0.9, thigh: rigData.dims.thigh * 1.1, shank: rigData.dims.shank * 1.1, hand: rigData.dims.hand * 1.1, shoulderHalfWidth: 0.19, thoraxHeight: 0.55 });
rig.setPose(bigPose()); connectivity('scaled, big pose');
{
  rig.setPose(anatomicalPose()); update();
  const ua = J('humerus_L').distanceTo(J('ulna_L'));
  check('scaleToDims: upper-arm length scaled', Math.abs(ua / rigData.dims.upperArm - 1.15) < 0.01, `${(ua * 1000).toFixed(1)} mm`);
  const gh = Math.abs(J('humerus_L').x - J('humerus_R').x) / 2;
  check('scaleToDims: shoulder half-width', Math.abs(gh - 0.19) < 0.004, `${(gh * 1000).toFixed(1)} mm`);
  let maxDev = 0;
  for (let f = 0; f <= 120; f += 20) {
    const p = anatomicalPose(); p.left.knee.flexion = f; rig.setPose(p); update();
    const L = LM('patellaApex_L').distanceTo(LM('tibialTuberosity_L'));
    if (f === 0) maxDev = 0; maxDev = Math.max(maxDev, Math.abs(L - LM('patellaApex_L').distanceTo(LM('tibialTuberosity_L'))));
  }
  check('scaled patella solve runs', Number.isFinite(maxDev));
}
rig.scaleToDims(rigData.dims);
rig.setPose(restPoseObj()); update();
{
  let maxErr = 0; const v = new THREE.Vector3(), w = new THREE.Vector3();
  for (const m of Object.values(rig.meshes)) { if (forearmChainIds().has(m.userData.segment)) continue; const pos = m.geometry.attributes.position; for (let i = 0; i < pos.count; i += 97) { v.fromBufferAttribute(pos, i); w.copy(v).applyMatrix4(m.matrixWorld); maxErr = Math.max(maxErr, v.distanceTo(w)); } }
  check('scaleToDims(rig.dims) restores rest geometry (< 1 mm)', maxErr < 0.001, `${(maxErr * 1000).toFixed(3)} mm`);
}

console.log(fails ? `\n${fails} FAILED` : '\nALL PASS');
process.exit(fails ? 1 : 0);
