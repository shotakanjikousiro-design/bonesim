// Semantic sanity checks for src/shared/conventions.js  (node tests/conventions.test.mjs)
import * as C from '../src/shared/conventions.js';
import * as THREE from 'three';
let fail = 0;
const ok = (cond, msg) => { console.log(`${cond ? 'PASS' : 'FAIL'}  ${msg}`); if (!cond) fail++; };
const dir = (a, b) => b.clone().sub(a).normalize();
const pose = () => C.anatomicalPose();
const fk = (p) => C.stickFK(p).positions;

for (const side of C.SIDES) {
  const s = C.sideSign(side); const k = (n) => `${n}_${side}`;
  let p = pose(); p[side].shoulder = { elevation: 90, plane: 0, axialRotation: 0 };
  let P = fk(p); ok(dir(P[k('shoulder')], P[k('elbow')]).x * s > 0.99, `${side}: abduction 90 → arm lateral`);
  p = pose(); p[side].shoulder = { elevation: 90, plane: 90, axialRotation: 0 };
  P = fk(p); ok(dir(P[k('shoulder')], P[k('elbow')]).z > 0.99, `${side}: flexion 90 → arm forward`);
  p = pose(); p[side].elbow = { flexion: 90 };
  P = fk(p); ok(dir(P[k('elbow')], P[k('wrist')]).z > 0.99, `${side}: elbow flex 90 → forearm forward`);
  p = pose(); p[side].elbow = { flexion: 90 }; p[side].shoulder.axialRotation = 90;
  P = fk(p); ok(dir(P[k('elbow')], P[k('wrist')]).x * -s > 0.99, `${side}: internal rotation 90 → forearm toward midline`);
  p = pose(); P = fk(p);
  ok(dir(P[k('pinkySide')], P[k('thumbSide')]).x * s > 0.99, `${side}: anatomical → thumb lateral`);
  p = pose(); p[side].forearm.pronation = 0; P = fk(p);
  ok(dir(P[k('pinkySide')], P[k('thumbSide')]).z > 0.99, `${side}: neutral forearm → thumb forward`);
  p = pose(); p[side].forearm.pronation = 90; P = fk(p);
  ok(dir(P[k('pinkySide')], P[k('thumbSide')]).x * -s > 0.99, `${side}: full pronation → thumb medial`);
  p = pose(); p[side].wrist.flexion = 90; P = fk(p);
  ok(dir(P[k('wrist')], P[k('handTip')]).z > 0.99, `${side}: palmar flexion (supinated) → hand toward palm side (+Z)`);
  p = pose(); p[side].wrist.ulnarDeviation = 45; P = fk(p);
  ok(dir(P[k('wrist')], P[k('handTip')]).x * -s > 0.6, `${side}: ulnar deviation → hand toward little-finger (medial) side`);
  p = pose(); p[side].hip.flexion = 90; P = fk(p);
  ok(dir(P[k('hip')], P[k('knee')]).z > 0.99, `${side}: hip flex 90 → thigh forward`);
  p = pose(); p[side].hip.abduction = 30; P = fk(p);
  ok(dir(P[k('hip')], P[k('knee')]).x * s > 0.49, `${side}: hip abduction → thigh lateral`);
  p = pose(); p[side].hip.flexion = 90; p[side].knee.flexion = 90; p[side].hip.axialRotation = 30; P = fk(p);
  ok(dir(P[k('knee')], P[k('ankle')]).x * s > 0.49, `${side}: hip internal rotation (hip&knee 90) → foot swings lateral`);
  p = pose(); p[side].knee.flexion = 90; P = fk(p);
  ok(dir(P[k('knee')], P[k('ankle')]).z < -0.99, `${side}: knee flex 90 → shank backward`);
  p = pose(); p[side].ankle.dorsiflexion = 20; P = fk(p);
  ok(dir(P[k('heel')], P[k('toe')]).y > 0.3, `${side}: dorsiflexion → toes up`);
}
let p = pose(); p.spine.flexion = 30; let P = fk(p);
ok(dir(P.pelvis, P.neckBase).z > 0.49, 'spine flexion → trunk forward');
p = pose(); p.spine.lateralBend = 30; P = fk(p);
ok(dir(P.pelvis, P.neckBase).x < -0.49, 'spine lateralBend + → toward right (-X)');
p = pose(); p.neck.axialRotation = 90; P = fk(p);
ok(P.nose.x > 0.05, 'neck axialRotation + → face turns left (+X)');
console.log(fail ? `\n${fail} FAILED` : '\nALL PASS'); process.exit(fail ? 1 : 0);
