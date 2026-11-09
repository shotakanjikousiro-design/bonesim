// =====================================================================
//  anatomicalRig.js — BodyParts3D skeleton driven by a BoneSim Pose.
//
//  Hierarchy: one THREE.Group per rig.json segment, its origin (pivot) at
//  the segment's joint centre, parented as in rig.json. Each group holds a
//  "holder" group (matrix = S·T(−centre)) that contains the meshes, whose
//  vertices are in WORLD REST coordinates — so at rest every vertex is exactly
//  where the GLB put it. Group quaternions are LOCAL rotations computed from
//  the Pose; because every group has identity rotation at rest, the rest-
//  pose (world) axes in rig.json are directly valid in the parent-local frame.
//
//  The mesh is not exactly in anatomical position, so each joint is driven by
//  (pose − rig.restPose). Coupled motions (連動運動) live in coupling.js.
// =====================================================================
import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { SIDES, shoulderQ, hipQ, spineQ, anatomicalPose } from '../shared/conventions.js';
import * as C from './coupling.js';
import { createMaterialSet } from './materials.js';
import { makeSkinnedSkin } from './skinBinding.js';

const SFX = { left: 'L', right: 'R' };
const FINGERS = { index: 2, middle: 3, ring: 4, little: 5 };
const V3 = (a) => new THREE.Vector3(a[0], a[1], a[2]);

/** landmarks whose nearest-surface bone would be ambiguous */
const LANDMARK_SEGMENT_HINT = {
  hipCenter: 'femur', kneeCenter: 'femur', ankleCenter: 'tibia', shoulderCenter: 'humerus',
  elbowCenter: 'humerus', wristCenter: 'radius', glenoidCenter: 'scapula', pelvisCenter: 'pelvis',
  radialStyloid: 'radius', ulnarStyloid: 'ulna', radialHead: 'radius', olecranon: 'ulna',
  patellaApex: 'patella', patellaBase: 'patella', tibialTuberosity: 'tibia',
  medialMalleolus: 'tibia', lateralMalleolus: 'tibia', medialEpicondyleHumerus: 'humerus',
  lateralEpicondyleHumerus: 'humerus', acromion: 'scapula', coracoid: 'scapula',
};

/** Deep-merge a (possibly partial) Pose onto the anatomical pose. */
export function completePose(pose = {}) {
  const out = anatomicalPose();
  const merge = (dst, src) => {
    for (const k in src) {
      const v = src[k];
      if (v && typeof v === 'object' && !Array.isArray(v)) { if (!dst[k] || typeof dst[k] !== 'object') dst[k] = {}; merge(dst[k], v); }
      else if (v !== undefined) dst[k] = v;
    }
  };
  merge(out, pose || {});
  return out;
}

/**
 * Principal (long) axis of the vertices of `meshes` whose projection on the
 * unit direction `dir` (from point `origin`) lies between lo·len and hi·len —
 * i.e. the diaphysis. Optional `matrix` maps vertices first. Returns a unit
 * vector oriented along `dir`.
 */
export function shaftAxis(meshes, origin, dir, len, { lo = 0.3, hi = 0.7, matrixOf = null } = {}) {
  const pts = []; const v = new THREE.Vector3();
  for (const m of meshes) {
    const pos = m.geometry.attributes.position; const M = matrixOf ? matrixOf(m) : null;
    for (let i = 0; i < pos.count; i++) {
      v.fromBufferAttribute(pos, i); if (M) v.applyMatrix4(M);
      const t = v.clone().sub(origin).dot(dir) / len; if (t >= lo && t <= hi) pts.push(v.clone());
    }
  }
  const c = new THREE.Vector3(); pts.forEach((p) => c.add(p)); c.multiplyScalar(1 / pts.length);
  const C = [0, 0, 0, 0, 0, 0, 0, 0, 0];
  for (const p of pts) { const d = [p.x - c.x, p.y - c.y, p.z - c.z]; for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) C[3 * i + j] += d[i] * d[j]; }
  let x = dir.clone().normalize();
  for (let it = 0; it < 50; it++) { // power iteration
    const y = new THREE.Vector3(C[0] * x.x + C[1] * x.y + C[2] * x.z, C[3] * x.x + C[4] * x.y + C[5] * x.z, C[6] * x.x + C[7] * x.y + C[8] * x.z);
    x = y.normalize();
  }
  return x.dot(dir) < 0 ? x.negate() : x;
}

export async function loadRigAssets({ glbUrl = './assets/skeleton.glb', rigUrl = './assets/rig.json' } = {}) {
  const [rigData, gltf] = await Promise.all([
    fetch(rigUrl).then((r) => { if (!r.ok) throw new Error(`rig.json: HTTP ${r.status}`); return r.json(); }),
    import('../shared/binAsset.js').then(({ loadBinary }) => loadBinary(glbUrl))
      .then((b) => new GLTFLoader().parseAsync(b.buffer.slice(b.byteOffset, b.byteOffset + b.byteLength), new URL('.', new URL(glbUrl, document.baseURI)).href)),
  ]);
  return { rigData, gltf };
}

/**
 * createRig({ scene, glbUrl, rigUrl, gltf?, rigData?, skin? })
 * gltf / rigData may be given directly (node tests).
 */
export async function createRig({ scene, glbUrl, rigUrl, gltf, rigData, skin = true } = {}) {
  if (!gltf || !rigData) ({ gltf, rigData } = await loadRigAssets({ glbUrl, rigUrl }));
  const R = rigData;
  const J = R.joints;
  const mats = createMaterialSet();

  // ---------------- hierarchy ----------------
  const root = new THREE.Group(); root.name = 'BoneSimRig';
  const pelvisCenter = V3(R.pelvisCenter);
  const segs = {};
  const order = [];
  const visit = (id) => {
    if (segs[id]) return;
    const s = R.segments[id];
    if (s.parent) visit(s.parent);
    const center = s.joint ? V3(J[s.joint].center) : pelvisCenter.clone();
    const group = new THREE.Group(); group.name = `seg:${id}`;
    const holder = new THREE.Group(); holder.name = `holder:${id}`;
    holder.matrixAutoUpdate = false;
    holder.matrix.makeTranslation(-center.x, -center.y, -center.z);
    group.add(holder);
    const seg = { id, parent: s.parent, joint: s.joint, center, group, holder, S: new THREE.Matrix3(), children: [], meshes: [], index: order.length };
    segs[id] = seg; order.push(seg);
    if (s.parent) { segs[s.parent].children.push(seg); segs[s.parent].group.add(group); }
    else root.add(group);
  };
  Object.keys(R.segments).forEach(visit);
  const restOffset = (seg) => seg.parent ? seg.center.clone().sub(segs[seg.parent].center) : new THREE.Vector3();
  for (const seg of order) { seg.basePos = restOffset(seg); seg.group.position.copy(seg.basePos); }
  root.position.copy(pelvisCenter);

  // ---------------- meshes ----------------
  const meshes = {}; let skinSrc = null;
  const glbMeshes = [];
  gltf.scene.traverse((o) => { if (o.isMesh) glbMeshes.push(o); });
  for (const m of glbMeshes) {
    const segId = m.userData?.segment ?? R.nodes[m.name]?.segment;
    if (m.name === 'skin' || !segId) { if (m.name === 'skin') skinSrc = m; continue; }
    const seg = segs[segId]; if (!seg) { console.warn('rig: no segment for', m.name); continue; }
    m.removeFromParent();
    m.position.set(0, 0, 0); m.quaternion.identity(); m.scale.set(1, 1, 1);
    m.material = mats.xray; m.frustumCulled = false;
    m.userData.nodeName = m.name; m.userData.segment = segId;
    seg.holder.add(m); seg.meshes.push(m); meshes[m.name] = m;
  }
  root.updateMatrixWorld(true);

  // skin → skinned mesh bound to the holders
  let skinMesh = null;
  if (skin && skinSrc) {
    skinSrc.removeFromParent();
    skinSrc.material = mats.skinXray;
    const adjacency = (a, b) => {
      if (a < 0 || b < 0) return false;
      const A = order[a], B = order[b];
      return A.parent === B.id || B.parent === A.id || (A.parent && A.parent === B.parent);
    };
    const depth = (s) => { let d = 0; for (let x = s; x.parent; x = segs[x.parent]) d++; return d; };
    const near = (a, b) => { // same segment or ≤ 2 joints apart along the tree
      if (a === b) return true; let A = order[a], B = order[b]; let n = 0;
      while (A !== B && n <= 2) { if (depth(A) >= depth(B)) A = segs[A.parent] ?? A; else B = segs[B.parent] ?? B; n++; }
      return A === B && n <= 2;
    };
    skinMesh = makeSkinnedSkin(skinSrc, order.map((s) => s.holder), order.map((s) => s.meshes), adjacency, segs.pelvis.holder, near);
    skinMesh.visible = false;
  }

  // ---------------- landmarks ----------------
  const landmarkObjs = {};
  const lmGroup = [];
  const sphereGeo = new THREE.SphereGeometry(0.0045, 10, 8);
  const lmMat = new THREE.MeshBasicMaterial({ color: 0x40e0ff, depthTest: false, transparent: true });
  for (const [name, p] of Object.entries(R.landmarks || {})) {
    const base = name.replace(/_[LR]$/, ''); const sfx = name.match(/_([LR])$/)?.[1];
    let segId = LANDMARK_SEGMENT_HINT[base] ? (sfx ? `${LANDMARK_SEGMENT_HINT[base]}_${sfx}` : LANDMARK_SEGMENT_HINT[base]) : null;
    if (!segs[segId]) segId = nearestSegment(V3(p));
    const o = new THREE.Mesh(sphereGeo, lmMat); o.name = `lm:${name}`; o.renderOrder = 10;
    o.position.set(p[0], p[1], p[2]); o.visible = false; o.userData.landmark = name;
    segs[segId].holder.add(o); landmarkObjs[name] = o; lmGroup.push(o);
  }
  function nearestSegment(p) {
    let best = null, bd = Infinity; const v = new THREE.Vector3();
    for (const seg of order) for (const m of seg.meshes) {
      const pos = m.geometry.attributes.position; const step = Math.max(1, Math.floor(pos.count / 1500));
      for (let i = 0; i < pos.count; i += step) { v.fromBufferAttribute(pos, i); const d = v.distanceToSquared(p); if (d < bd) { bd = d; best = seg.id; } }
    }
    return best;
  }

  // ---------------- joint-axis helpers ----------------
  const AXIS_COLORS = { flexion: 0xff5050, dorsiflexion: 0xff5050, pronation: 0x40d040, inversion: 0xb070ff, ulnarDeviation: 0xffd040, abduction: 0x4090ff, axialRotation: 0xff9a30, elevation: 0x4090ff, protraction: 0x30d0d0, posteriorRotation: 0xc0c040, upwardRotation: 0x4090ff, internalRotation: 0xff9a30, posteriorTilt: 0xc0c040, lateralBend: 0x4090ff };
  const axisObjs = [];
  for (const seg of order) {
    if (!seg.joint) continue; const j = J[seg.joint]; const par = segs[seg.parent];
    const holder = new THREE.Group(); holder.name = `axes:${seg.joint}`; holder.visible = false;
    holder.position.copy(seg.basePos);
    const len = /mcp|pip|dip|ip1|cmc|C\d|L\d|T12|lumbo|atlanto/.test(seg.joint) ? 0.03 : 0.06;
    for (const [k, a] of Object.entries(j.axes || {})) {
      if (/^(lumbosacral|L\dL\d|T12L1|C\dC\d|C7T1|C1C2|atlantooccipital)$/.test(seg.joint) && k !== 'flexion') continue;
      const arrow = new THREE.ArrowHelper(V3(a).normalize(), new THREE.Vector3(), len, AXIS_COLORS[k] ?? 0xffffff, len * 0.25, len * 0.12);
      arrow.traverse((o) => { if (o.material) { o.material.depthTest = false; o.material.transparent = true; } o.renderOrder = 11; });
      holder.add(arrow);
    }
    par.group.add(holder); axisObjs.push({ holder, seg });
  }

  // ---------------- pose → local rotations ----------------
  const rest = (side) => {
    const r = R.restPose?.[SFX[side]] || {};
    return {
      shoulder: { elevation: 0, plane: 0, axialRotation: 0, ...r.shoulder },
      elbow: { flexion: 0, ...r.elbow }, forearm: { pronation: -90, ...r.forearm },
      wrist: { flexion: 0, ulnarDeviation: 0, ...r.wrist },
      hip: { flexion: 0, abduction: 0, axialRotation: 0, ...r.hip },
      knee: { flexion: 0, ...r.knee }, ankle: { dorsiflexion: 0, inversion: 0, ...r.ankle },
    };
  };
  const REST = { left: rest('left'), right: rest('right') };

  // ---------------- elbow: carrying angle (rest correction + oblique axis) ----------------
  // The BodyParts3D forearm sits in ≈3–6° VARUS relative to the humerus and the fitted
  // flexion axis is exactly ⟂ to the humeral shaft. A normal elbow shows ≈10–15° VALGUS
  // (carrying angle) in extension that disappears in flexion because the flexion axis
  // is oblique (medial end more distal by ≈5–8° relative to the plane ⟂ the shaft;
  // London 1981, Deland 1987). We therefore
  //  1. undo the mesh's rest flexion about the fitted axis (E),
  //  2. add a fixed valgus rotation V about the AP axis through the elbow centre so the
  //     humeral-shaft / forearm-shaft angle in extension = CARRYING_ANGLE,
  //  3. flex about the tilted axis a' (medial end distal by AXIS_TILT): the forearm then
  //     ends up over / slightly medial to the humerus in full flexion.
  const CARRYING_ANGLE = 10;       // humeral shaft ↔ ulnar shaft, extension (clinical carrying angle)
  const RESIDUAL_AT_90 = 1.5;      // forearm deviation from the sagittal plane at 90° flexion (≈ 0)
  const elbowModel = {};
  for (const side of SIDES) {
    const x = SFX[side]; const s = side === 'left' ? 1 : -1;
    const gh = V3(J[`glenohumeral_${x}`].center), hu = V3(J[`humeroulnar_${x}`].center), rc = V3(J[`radiocarpal_${x}`].center);
    const a0 = V3(J[`humeroulnar_${x}`].axes.flexion).normalize();
    const umech = hu.clone().sub(gh).normalize();                                 // humeral mechanical axis
    const uh = shaftAxis(segs[`humerus_${x}`].meshes, gh, umech, gh.distanceTo(hu)); // humeral shaft
    const E = new THREE.Quaternion().setFromAxisAngle(a0, -REST[side].elbow.flexion * Math.PI / 180);
    const fdir = rc.clone().sub(hu).normalize();
    const uRest = shaftAxis(segs[`ulna_${x}`].meshes, hu, fdir, hu.distanceTo(rc));  // ulnar shaft
    const lat = new THREE.Vector3(s, 0, 0);
    const frame = (u) => { // AP normal n and lateral in-plane direction m for reference axis u (a0 ⟂ part sets the plane)
      const aPerp = a0.clone().sub(u.clone().multiplyScalar(a0.dot(u))).normalize();
      const n = new THREE.Vector3().crossVectors(u, aPerp).normalize();
      const m = new THREE.Vector3().crossVectors(n, u).normalize(); if (m.dot(lat) < 0) { m.negate(); n.negate(); }
      return { aPerp, n, m };
    };
    const ang = (f, u, m) => Math.atan2(f.dot(m), f.dot(u)) * 180 / Math.PI;     // + = valgus (lateral)
    const Fs = frame(uh);
    const c0 = ang(uRest.clone().applyQuaternion(E), uh, Fs.m);
    const V = new THREE.Quaternion().setFromAxisAngle(Fs.n, (CARRYING_ANGLE - c0) * Math.PI / 180);
    const pre = V.clone().multiply(E);
    // oblique flexion axis: tilt (medial end distal) relative to the mechanical axis so that the
    // forearm (elbow centre → wrist centre) flexes into ≈ the sagittal plane and lies over the humerus
    const Fm = frame(umech);
    const fjc = fdir.clone().applyQuaternion(pre);
    const cjc = ang(fjc, umech, Fm.m);
    const tilt = cjc - RESIDUAL_AT_90; const b = tilt * Math.PI / 180;
    const aMed = Fm.aPerp.dot(Fm.m) < 0 ? Fm.aPerp.clone() : Fm.aPerp.clone().negate();
    const aT = aMed.multiplyScalar(Math.cos(b)).add(umech.clone().multiplyScalar(Math.sin(b))).normalize();
    if (aT.dot(a0) < 0) aT.negate();                                             // keep "+ = flexion"
    const tiltShaft = Math.asin(Math.min(1, Math.abs(aT.dot(uh)))) * 180 / Math.PI;
    elbowModel[side] = { pre, axis: aT.toArray(), restCarrying: c0, carrying: CARRYING_ANGLE, tilt, tiltVsShaft: tiltShaft, carryingJointCentres: cjc };
    // Humeral axial rotation of the mesh is not in rig.restPose: choose it so that, in the
    // anatomical position (axialRotation 0), the elbow flexion axis lies in the coronal plane
    // (conventions: elbowQ = Rx(−flex) in the humerus frame → forearm flexes in the sagittal plane).
    const r = REST[side].shoulder; let best = 0, bestErr = Infinity;
    for (let psi = -60; psi <= 60; psi += 0.25) {
      const D0 = shoulderQ(side, { elevation: 0, plane: 0, axialRotation: 0 }).multiply(shoulderQ(side, { ...r, axialRotation: psi }).invert());
      const err = Math.abs(aT.clone().applyQuaternion(D0).z);
      if (err < bestErr - 1e-9 || (Math.abs(err - bestErr) < 1e-9 && Math.abs(psi) < Math.abs(best))) { bestErr = err; best = psi; }
    }
    r.axialRotation = best; elbowModel[side].restHumeralRotation = best;
  }
  const ax = (jointId, key) => J[jointId]?.axes?.[key];
  const Rq = (jointId, key, deg) => C.axisAngleQ(ax(jointId, key), deg);
  const qmul = (...qs) => qs.reduce((a, b) => a.multiply(b), new THREE.Quaternion());

  // ---------------- rib cage (scapulothoracic contact) ----------------
  // Star-shaped outline of the thorax segment per 1-cm horizontal slice (max radius in 5°
  // bins around the slice centre), in the thorax-group frame; scapula sample points (AC frame).
  const thoraxHull = { slices: [] }; const scapPts = {};
  function updateThoraxHull() {
    const th = segs.thorax; const pts = []; const v = new THREE.Vector3();
    for (const m of th.meshes) { const pos = m.geometry.attributes.position; for (let i = 0; i < pos.count; i += 2) pts.push(v.fromBufferAttribute(pos, i).clone().sub(th.center).applyMatrix3(th.S)); }
    let y0 = Infinity, y1 = -Infinity; for (const p of pts) { y0 = Math.min(y0, p.y); y1 = Math.max(y1, p.y); }
    const NB = 72, dy = 0.01; const slices = [];
    for (let y = y0; y <= y1; y += dy) {
      const sl = pts.filter((p) => Math.abs(p.y - y) < dy); if (sl.length < 20) { slices.push(null); continue; }
      let x0 = Infinity, x1 = -Infinity, z0 = Infinity, z1 = -Infinity; for (const p of sl) { x0 = Math.min(x0, p.x); x1 = Math.max(x1, p.x); z0 = Math.min(z0, p.z); z1 = Math.max(z1, p.z); }
      const cx = (x0 + x1) / 2, cz = (z0 + z1) / 2; const r = new Float64Array(NB).fill(-1);
      for (const p of sl) { const b = Math.floor(((Math.atan2(p.z - cz, p.x - cx) + Math.PI) / (2 * Math.PI)) * NB) % NB; r[b] = Math.max(r[b], Math.hypot(p.x - cx, p.z - cz)); }
      for (let k = 0; k < 2; k++) for (let b = 0; b < NB; b++) if (r[b] < 0) r[b] = Math.max(r[(b + NB - 1) % NB], r[(b + 1) % NB]);
      slices.push({ cx, cz, r });
    }
    Object.assign(thoraxHull, { y0, dy, NB, slices });
    for (const side of SIDES) {
      const x = SFX[side]; const sc = segs[`scapula_${x}`]; const list = [];
      for (const m of sc.meshes) { const pos = m.geometry.attributes.position; const step = Math.max(1, Math.floor(pos.count / 300)); for (let i = 0; i < pos.count; i += step) list.push(v.fromBufferAttribute(pos, i).clone().sub(sc.center).applyMatrix3(sc.S)); }
      const o = { local: list, allow: null }; scapPts[side] = o;
      o.allow = list.map((p) => Math.max(0, hullDepth(scapToThorax(side, p, new THREE.Quaternion(), new THREE.Quaternion()))) + 0.001);
    }
  }
  /** depth (m) of a thorax-frame point inside the rib-cage outline (< 0 = outside) */
  function hullDepth(p) {
    const H = thoraxHull; const fi = (p.y - H.y0) / H.dy; const i = Math.floor(fi);
    const a = H.slices[i], b = H.slices[i + 1]; if (!a || !b) return -1;
    const f = fi - i; const d = (sl) => {
      const ang = Math.atan2(p.z - sl.cz, p.x - sl.cx); const fb = ((ang + Math.PI) / (2 * Math.PI)) * H.NB - 0.5;
      const b0 = ((Math.floor(fb) % H.NB) + H.NB) % H.NB, b1 = (b0 + 1) % H.NB, t = fb - Math.floor(fb);
      return sl.r[b0] * (1 - t) + sl.r[b1] * t - Math.hypot(p.x - sl.cx, p.z - sl.cz);
    };
    return d(a) * (1 - f) + d(b) * f;
  }
  function scapToThorax(side, pLocal, Cq, T) {
    const x = SFX[side];
    return pLocal.clone().applyQuaternion(T).add(segs[`scapula_${x}`].basePos.clone().applyQuaternion(Cq)).add(segs[`clavicle_${x}`].basePos);
  }
  function scapPenetration(side, Cq, T) {
    const o = scapPts[side]; if (!o) return 0; let m = 0;
    o.local.forEach((p, i) => { m = Math.max(m, hullDepth(scapToThorax(side, p, Cq, T)) - o.allow[i]); });
    return m;
  }
  /**
   * Smallest correction (coordinate descent over scapular internal rotation and posterior
   * tilt, 2° → 0.5° steps, |each| ≤ 25°) that brings the scapula back onto the rib cage.
   * Upward rotation (the teaching variable) is never changed. Returns {q, ir, tilt} or null.
   */
  function ribCorrection(side, Cq, T) {
    let pen0 = scapPenetration(side, Cq, T); if (!(pen0 > 0)) return null;
    const x = SFX[side]; const aIR = ax(`acromioclavicular_${x}`, 'internalRotation'), aT = ax(`acromioclavicular_${x}`, 'posteriorTilt');
    let ir = 0, tilt = 0; const make = (i, t) => C.axisAngleQ(aIR, i).multiply(C.axisAngleQ(aT, t));
    const cost = (i, t) => Math.max(0, scapPenetration(side, Cq, make(i, t).multiply(T))) * 1000 + 0.002 * (i * i + t * t);
    let best = cost(0, 0);
    for (const step of [4, 2, 1, 0.5]) {
      for (let it = 0; it < 12; it++) {
        let moved = false;
        for (const [di, dt] of [[step, 0], [-step, 0], [0, step], [0, -step]]) {
          const ni = ir + di, nt = tilt + dt; if (Math.abs(ni) > 25 || Math.abs(nt) > 25) continue;
          const c = cost(ni, nt); if (c < best - 1e-9) { best = c; ir = ni; tilt = nt; moved = true; }
        }
        if (!moved) break;
      }
    }
    return { q: make(ir, tilt), ir, tilt };
  }

  // patella geometry (per side, femur-group frame; recomputed by scaleToDims).
  // Patella model (2 DOF in the sagittal plane of the trochlea):
  //  - its ORIENTATION (patellar flexion) is φ = κ·Δknee-flexion, κ = 0.7 (in-vivo patellar
  //    flexion lags knee flexion at ≈ 0.6–0.75×; e.g. Nha 2008, Suzuki 2012);
  //  - its CENTROID glides on the circle fitted to the trochlear groove (angle θ about the
  //    groove centre): θ = φ while the patellar tendon is slack, and once it is taut θ is the
  //    first root of |apex − tibial tuberosity| = L_taut (tendon length held constant).
  // The BodyParts3D patella sits low (quadriceps relaxed), so the rest apex–tuberosity
  // distance (≈ 25 mm) is treated as slack; L_taut is the smallest length that keeps the
  // constraint feasible over 0–140° (≈ 35–38 mm).
  const PATELLA_KAPPA = 0.7;
  const patellaGeom = {};
  function updatePatellaGeom() {
    for (const side of SIDES) {
      const x = SFX[side]; const pf = J[`patellofemoral_${x}`]; if (!pf || !segs[`patella_${x}`]) continue;
      const tib = segs[`tibia_${x}`], pat = segs[`patella_${x}`];
      const u = V3(pf.axes.flexion).normalize();
      const tubLocal = V3(pf.tibialTuberosity).sub(tib.center).applyMatrix3(tib.S);
      const apexRel = V3(pf.patellaApex).sub(pat.center).applyMatrix3(pat.S);          // rel. groove centre
      const cenRel = V3(pf.patellaRestCentroid).sub(pat.center).applyMatrix3(pat.S);
      const K = tib.basePos.clone(), P = pat.basePos.clone();
      const Lrest = P.clone().add(apexRel).distanceTo(K.clone().add(tubLocal));
      const g = { u, axis: u.toArray(), tubLocal, apexRel, cenRel, a0: apexRel.clone().sub(cenRel), K, P, Lrest, Ltaut: Lrest, push: null };
      patellaGeom[side] = g;
      const r0 = REST[side].knee.flexion; let need = Lrest;
      for (let kf = 0; kf <= 140; kf += 5) {
        const tub = tubAt(g, x, kf, r0, true); const phi = PATELLA_KAPPA * (kf - r0);
        let m = Infinity; for (let th = phi; th <= phi + 170; th += 1) m = Math.min(m, apexAt(g, th, phi, 0).distanceTo(tub));
        need = Math.max(need, m + 0.0008);
      }
      g.Ltaut = need;
      // contact: because the orientation lags the glide, the patella's articular surface can dip
      // into the femur; tabulate (per 5° of knee flexion) the outward radial lift that keeps it
      // on the femoral surface (signed distance to the femur mesh ≥ 0.3 mm).
      const fem = segs[`femur_${x}`]; const sdf = makeSdf(fem.meshes, fem.center, fem.S, P);
      const pv = []; const v = new THREE.Vector3();
      for (const mm of pat.meshes) { const pos = mm.geometry.attributes.position; for (let i = 0; i < pos.count; i += 2) pv.push(v.fromBufferAttribute(pos, i).clone().sub(pat.center).applyMatrix3(pat.S).sub(cenRel)); }
      g.push = [];
      for (let kf = 0; kf <= 140; kf += 5) {
        const tub = tubAt(g, x, kf, r0, true); const phi = PATELLA_KAPPA * (kf - r0); let push = 0;
        for (let it = 0; it < 4; it++) {
          const th = phi + C.patellaTautAngle((dth) => apexAt(g, phi + dth, phi, push).distanceTo(tub) - g.Ltaut).phi;
          const c = centroidAt(g, th, push); const q = C.axisAngleQ(g.axis, phi); let minSd = Infinity;
          for (const w of pv) minSd = Math.min(minSd, sdf(w.clone().applyQuaternion(q).add(c)));
          if (minSd >= 0.0003) break; push += 0.0003 - minSd;
        }
        g.push.push(push);
      }
    }
  }
  /** signed distance (≈, via nearest vertex + its normal) to a mesh set, in the femur-group frame rel. P */
  function makeSdf(meshes, center, S, P) {
    const cell = 0.005, grid = new Map(), key = (i, j, k) => `${i},${j},${k}`; const v = new THREE.Vector3(), nn = new THREE.Vector3();
    const N = new THREE.Matrix3().copy(S).invert().transpose();
    for (const m of meshes) {
      const pos = m.geometry.attributes.position, nor = m.geometry.attributes.normal;
      for (let i = 0; i < pos.count; i++) {
        v.fromBufferAttribute(pos, i).sub(center).applyMatrix3(S).sub(P); nn.fromBufferAttribute(nor, i).applyMatrix3(N).normalize();
        if (v.length() > 0.09) continue;
        const k = key(Math.floor(v.x / cell), Math.floor(v.y / cell), Math.floor(v.z / cell));
        let a = grid.get(k); if (!a) grid.set(k, a = []); a.push([v.x, v.y, v.z, nn.x, nn.y, nn.z]);
      }
    }
    return (p) => {
      const ci = Math.floor(p.x / cell), cj = Math.floor(p.y / cell), ck = Math.floor(p.z / cell); let best = null, bd = Infinity;
      for (let i = ci - 1; i <= ci + 1; i++) for (let j = cj - 1; j <= cj + 1; j++) for (let k = ck - 1; k <= ck + 1; k++) {
        const a = grid.get(key(i, j, k)); if (!a) continue;
        for (const q of a) { const d = (p.x - q[0]) ** 2 + (p.y - q[1]) ** 2 + (p.z - q[2]) ** 2; if (d < bd) { bd = d; best = q; } }
      }
      return best ? (p.x - best[0]) * best[3] + (p.y - best[1]) * best[4] + (p.z - best[2]) * best[5] : 1;
    };
  }
  /** centroid (rel. groove centre) for glide θ plus outward radial lift */
  function centroidAt(g, theta, push = 0) {
    const c = g.cenRel.clone().applyQuaternion(C.axisAngleQ(g.axis, theta));
    if (push) { const r = c.clone().sub(g.u.clone().multiplyScalar(c.dot(g.u))).normalize(); c.addScaledVector(r, push); }
    return c;
  }
  const pushAt = (g, kf) => { if (!g.push) return 0; const xk = Math.max(0, Math.min(g.push.length - 1.001, kf / 5)); const i = Math.floor(xk), f = xk - i; return g.push[i] * (1 - f) + g.push[i + 1] * f; };
  const _pq = new THREE.Quaternion();
  const _pq2 = new THREE.Quaternion(); const _tv = new THREE.Vector3();
  /** apex (femur-group frame) for glide θ and orientation φ */
  function apexAt(g, theta, phi, push = 0, target = new THREE.Vector3()) {
    _tv.copy(g.a0).applyQuaternion(C.axisAngleQ(g.axis, phi, _pq2));
    return target.copy(centroidAt(g, theta, push)).add(_tv).add(g.P);
  }
  function tubAt(g, x, kf, r0, coupling) {
    const rot = coupling ? C.screwHomeRotation(kf) - C.screwHomeRotation(r0) : 0;
    const tq = new THREE.Quaternion().multiplyQuaternions(Rq(`knee_${x}`, 'flexion', kf - r0), Rq(`knee_${x}`, 'axialRotation', rot));
    return g.tubLocal.clone().applyQuaternion(tq).add(g.K);
  }

  const state = { pose: completePose({}), coupling: true, autoHumeralRotation: false, useRoot: false, derived: {} };

  /** Compute every segment's LOCAL quaternion for a Pose. */
  function solve(pose, opts) {
    const coupling = opts.coupling !== false;
    const q = {}; const t = {}; const derived = {};
    for (const seg of order) q[seg.id] = new THREE.Quaternion();
    if (opts.useRoot && pose.root?.quaternion) q.pelvis.fromArray(pose.root.quaternion);

    // ---- spine & neck (h) ----
    const chain = (joints, angles, weights) => {
      const parts = C.distributeSpine(angles, weights);
      const segFor = (jid) => order.find((s) => s.joint === jid);
      const acc = new THREE.Quaternion();
      joints.forEach((jid, i) => {
        const s = segFor(jid); if (!s) return;
        if (i < joints.length - 1) { q[s.id] = spineQ(parts[i]); acc.multiply(q[s.id]); }
        else q[s.id] = acc.clone().invert().multiply(spineQ(angles)); // exact total
      });
    };
    chain(C.SPINE_JOINTS, pose.spine, coupling ? C.SPINE_WEIGHTS : C.SPINE_WEIGHTS_NAIVE);
    chain(C.NECK_JOINTS, pose.neck, coupling ? C.NECK_WEIGHTS : C.NECK_WEIGHTS_NAIVE);

    for (const side of SIDES) {
      const x = SFX[side]; const L = pose[side]; const r0 = REST[side]; const d = (derived[side] = {});
      // ---- shoulder girdle + GH (a) ----
      const sh = { ...L.shoulder };
      if (coupling && opts.autoHumeralRotation) sh.axialRotation = (sh.axialRotation || 0) + C.autoHumeralExternalRotation(sh.elevation, sh.plane);
      const D = shoulderQ(side, sh).multiply(shoulderQ(side, r0.shoulder).invert()); // HT delta (thorax frame)
      if (coupling) {
        const g = C.shoulderGirdleAngles(sh.elevation, sh.plane), g0 = C.shoulderGirdleAngles(r0.shoulder.elevation, r0.shoulder.plane);
        const sc = `sternoclavicular_${x}`, ac = `acromioclavicular_${x}`;
        const Cq = qmul(Rq(sc, 'elevation', g.clavicle.elevation - g0.clavicle.elevation),
          Rq(sc, 'protraction', g.clavicle.protraction - g0.clavicle.protraction),
          Rq(sc, 'posteriorRotation', g.clavicle.posteriorRotation - g0.clavicle.posteriorRotation));
        let T = qmul(Rq(ac, 'upwardRotation', g.scapula.upwardRotation - g0.scapula.upwardRotation),
          Rq(ac, 'internalRotation', -(g.scapula.externalRotation - g0.scapula.externalRotation)),
          Rq(ac, 'posteriorTilt', g.scapula.posteriorTilt - g0.scapula.posteriorTilt));
        // keep the scapula on the rib cage: the rigid rotation about the AC joint would sweep the
        // medial border / inferior angle into the thorax; add the smallest extra scapular internal
        // rotation (medial border lifts posteriorly, ≈ "gliding" on the chest wall) that avoids it.
        const ribFix = ribCorrection(side, Cq, T);
        if (ribFix) T = ribFix.q.multiply(T);
        d.scapulaRibCorrection = ribFix ? ribFix.ir : 0; d.scapulaRibTiltCorrection = ribFix ? ribFix.tilt : 0;
        q[`clavicle_${x}`] = Cq;
        q[`scapula_${x}`] = Cq.clone().invert().multiply(T);
        q[`humerus_${x}`] = T.clone().invert().multiply(D);
        Object.assign(d, { scapularUpwardRotation: g.scapula.upwardRotation, scapularPosteriorTilt: g.scapula.posteriorTilt,
          clavicleElevation: g.clavicle.elevation, claviclePosteriorRotation: g.clavicle.posteriorRotation,
          glenohumeralElevation: Math.max(0, sh.elevation - g.scapula.upwardRotation) });
      } else {
        q[`humerus_${x}`] = D;
        Object.assign(d, { scapularUpwardRotation: 0, glenohumeralElevation: sh.elevation });
      }
      // ---- elbow (b) / forearm (c) ----
      const em = elbowModel[side];
      const flexQ = C.axisAngleQ(em.axis, L.elbow.flexion).multiply(em.pre);
      const dPron = L.forearm.pronation - r0.forearm.pronation;
      if (coupling) { q[`ulna_${x}`] = flexQ; q[`radius_${x}`] = Rq(`radioulnar_${x}`, 'pronation', dPron); }
      else { q[`ulna_${x}`] = flexQ.multiply(Rq(`radioulnar_${x}`, 'pronation', dPron)); } // naive: rigid forearm twist
      // ---- wrist (d) ----
      const dF = L.wrist.flexion - r0.wrist.flexion, dU = L.wrist.ulnarDeviation - r0.wrist.ulnarDeviation;
      const w = coupling ? C.wristSplit(L.wrist.flexion, L.wrist.ulnarDeviation, r0.wrist.flexion, r0.wrist.ulnarDeviation) : { radiocarpal: { flexion: dF, ulnarDeviation: dU }, midcarpal: { flexion: 0, ulnarDeviation: 0 } };
      q[`carpalsProximal_${x}`] = qmul(Rq(`radiocarpal_${x}`, 'flexion', w.radiocarpal.flexion), Rq(`radiocarpal_${x}`, 'ulnarDeviation', w.radiocarpal.ulnarDeviation));
      q[`hand_${x}`] = qmul(Rq(`midcarpal_${x}`, 'flexion', w.midcarpal.flexion), Rq(`midcarpal_${x}`, 'ulnarDeviation', w.midcarpal.ulnarDeviation));
      d.wrist = w;
      // ---- fingers (e) ----
      const F = L.fingers;
      if (F) {
        const t = F.thumb || {};
        q[`thumbMeta_${x}`] = qmul(Rq(`cmc1_${x}`, 'flexion', t.cmcFlexion || 0), Rq(`cmc1_${x}`, 'abduction', t.cmcAbduction || 0));
        q[`thumbProx_${x}`] = Rq(`mcp1_${x}`, 'flexion', t.mcp || 0);
        q[`thumbDist_${x}`] = Rq(`ip1_${x}`, 'flexion', t.ip || 0);
        for (const [fname, n] of Object.entries(FINGERS)) {
          const f = F[fname] || {};
          const pip = f.pip || 0; const dip = f.dip ?? (coupling ? C.dipFromPip(pip) : 0);
          q[`${fname}Prox_${x}`] = qmul(Rq(`mcp${n}_${x}`, 'flexion', f.mcp || 0), Rq(`mcp${n}_${x}`, 'abduction', f.mcpAbduction || 0));
          q[`${fname}Mid_${x}`] = Rq(`pip${n}_${x}`, 'flexion', pip);
          q[`${fname}Dist_${x}`] = Rq(`dip${n}_${x}`, 'flexion', dip);
        }
      }
      // ---- hip (i) ----
      q[`femur_${x}`] = hipQ(side, L.hip).multiply(hipQ(side, r0.hip).invert());
      // ---- knee: flexion + screw-home (f) ----
      const kf = L.knee.flexion;
      const tibRot = coupling ? C.screwHomeRotation(kf) - C.screwHomeRotation(r0.knee.flexion) : 0;
      const tq = qmul(Rq(`knee_${x}`, 'flexion', kf - r0.knee.flexion), Rq(`knee_${x}`, 'axialRotation', tibRot));
      q[`tibia_${x}`] = tq;
      d.tibialRotation = coupling ? C.screwHomeRotation(kf) : 0;
      // ---- patella (f): orientation 0.7 × knee flexion, glides in the groove, tendon held ----
      const pg = patellaGeom[side];
      if (pg && coupling) {
        const tub = pg.tubLocal.clone().applyQuaternion(tq).add(pg.K); const tmp = new THREE.Vector3();
        const phi = PATELLA_KAPPA * (kf - r0.knee.flexion);
        const push = pushAt(pg, kf);
        const sol = C.patellaTautAngle((dth) => apexAt(pg, phi + dth, phi, push, tmp).distanceTo(tub) - pg.Ltaut);
        const theta = phi + sol.phi;
        const qPhi = C.axisAngleQ(pg.axis, phi);
        q[`patella_${x}`] = qPhi;
        // p' − c = R(φ)(p − centroid₀) + centroid(θ, lift)  (rotation φ about the moved centroid)
        t[`patella_${x}`] = centroidAt(pg, theta, push).sub(pg.cenRel.clone().applyQuaternion(qPhi));
        Object.assign(d, { patellaRotation: phi, patellaGlide: theta, patellaTendonTaut: sol.taut });
      }
      // ---- ankle (g) ----
      q[`talus_${x}`] = Rq(`talocrural_${x}`, 'dorsiflexion', L.ankle.dorsiflexion - r0.ankle.dorsiflexion);
      q[`foot_${x}`] = Rq(`subtalar_${x}`, 'inversion', L.ankle.inversion - r0.ankle.inversion);
    }
    return { q, t, derived };
  }

  function setPose(pose, opts = {}) {
    const p = completePose(pose);
    const o = { coupling: opts.coupling ?? state.coupling, autoHumeralRotation: opts.autoHumeralRotation ?? state.autoHumeralRotation, useRoot: opts.useRoot ?? state.useRoot };
    state.coupling = o.coupling; state.autoHumeralRotation = o.autoHumeralRotation; state.useRoot = o.useRoot;
    const { q, t, derived } = solve(p, o);
    for (const seg of order) {
      seg.group.quaternion.copy(q[seg.id]);
      seg.group.position.copy(seg.basePos); if (t[seg.id]) seg.group.position.add(t[seg.id]);
    }
    state.pose = p; state.derived = derived;
    return derived;
  }

  // ---------------- display ----------------
  let style = 'xray'; let highlighted = null; let skinOpacity = 0; let faded = new Set();
  function applyMaterials() {
    for (const seg of order) for (const m of seg.meshes) {
      const hl = m === highlighted, fd = !hl && faded.has(seg.id);
      m.visible = style !== 'hidden';
      m.material = style === 'bone' ? (hl ? mats.boneHL : fd ? mats.boneFaded : mats.bone) : (hl ? mats.xrayHL : fd ? mats.xrayFaded : mats.xray);
      m.renderOrder = style === 'bone' ? 0 : 1;
    }
    if (skinMesh) {
      skinMesh.material = style === 'bone' ? mats.skinBone : mats.skinXray;
      skinMesh.visible = skinOpacity > 0.001;
      mats.skinBone.opacity = skinOpacity * 0.5;
      mats.skinXray.uniforms.uOpacity.value = skinOpacity * 2;
      skinMesh.renderOrder = 5;
    }
  }
  function setStyle(s) { style = s; applyMaterials(); }
  function setSkin(opacity) { skinOpacity = Math.max(0, Math.min(1, opacity)); applyMaterials(); }
  function highlight(nodeName) {
    highlighted = nodeName ? meshes[nodeName] || null : null; applyMaterials();
    if (!highlighted) return null;
    return nodeInfo(nodeName);
  }
  function nodeInfo(nodeName) {
    const n = R.nodes[nodeName] || {}; const segId = meshes[nodeName]?.userData.segment ?? n.segment;
    return { nodeName, nameJa: n.nameJa, nameEn: n.nameEn, segment: segId, joint: R.segments[segId]?.joint ?? null, group: n.group };
  }
  /**
   * Low-confidence cue (camera mode): segments whose driving joint has
   * confidence < threshold (tracker's pose.confidence map, keys like
   * 'left.elbow' / 'left.elbow.flexion') are drawn faded, and so are their
   * descendants. Pass null to clear.
   */
  const SEG_CONF = [[/^humerus_/, 'shoulder'], [/^ulna_/, 'elbow'], [/^radius_/, 'forearm'], [/^(carpalsProximal|hand)_/, 'wrist'],
    [/^(thumb|index|middle|ring|little)/, 'fingers'], [/^femur_/, 'hip'], [/^(tibia|patella)_/, 'knee'], [/^(talus|foot)_/, 'ankle']];
  function setConfidence(conf, threshold = 0.4) {
    const next = new Set();
    if (conf) {
      const get = (k) => { if (k in conf) return conf[k]; let m = Infinity; for (const key in conf) if (key.startsWith(`${k}.`)) m = Math.min(m, conf[key]); return Number.isFinite(m) ? m : 1; };
      for (const seg of order) {
        if (seg.parent && next.has(seg.parent)) { next.add(seg.id); continue; }
        const sideKey = seg.id.endsWith('_L') ? 'left' : seg.id.endsWith('_R') ? 'right' : null;
        const hit = SEG_CONF.find(([re]) => re.test(seg.id));
        if (sideKey && hit) {
          const key = `${sideKey}.${hit[1]}`;
          if (hit[1] === 'fingers' ? (key in conf && conf[key] < threshold) : get(key) < threshold) next.add(seg.id);
        }
      }
    }
    const changed = next.size !== faded.size || [...next].some((x) => !faded.has(x));
    faded = next; if (changed) applyMaterials();
    return [...faded];
  }
  const showAxes = (b) => axisObjs.forEach((a) => { a.holder.visible = !!b; });
  const showLandmarks = (b) => lmGroup.forEach((o) => { o.visible = !!b; });

  // ---------------- per-person scaling ----------------
  const restDims = { ...R.dims };
  function setSegScale(id, S) {
    const seg = segs[id]; if (!seg) return;
    seg.S.copy(S);
    const m4 = new THREE.Matrix4().setFromMatrix3(S);
    seg.holder.matrix.copy(m4).multiply(new THREE.Matrix4().makeTranslation(-seg.center.x, -seg.center.y, -seg.center.z));
  }
  const axialS = (u, k) => { // I + (k−1) u uᵀ
    const n = u.clone().normalize(); const a = k - 1;
    return new THREE.Matrix3().set(1 + a * n.x * n.x, a * n.x * n.y, a * n.x * n.z, a * n.y * n.x, 1 + a * n.y * n.y, a * n.y * n.z, a * n.z * n.x, a * n.z * n.y, 1 + a * n.z * n.z);
  };
  const diagS = (x, y, z) => new THREE.Matrix3().set(x, 0, 0, 0, y, 0, 0, 0, z);
  const lim = (k) => Math.min(1.45, Math.max(0.7, Number.isFinite(k) ? k : 1));
  function scaleToDims(dims) {
    const k = (key) => lim((dims?.[key] ?? restDims[key]) / restDims[key]);
    for (const seg of order) setSegScale(seg.id, new THREE.Matrix3());
    const yT = k('thoraxHeight');
    for (const id of ['L5', 'L4', 'L3', 'L2', 'L1', 'thorax']) setSegScale(id, diagS(1, yT, 1));
    setSegScale('pelvis', diagS(k('hipHalfWidth'), yT, 1));
    const kn = k('neck'), kh = k('head');
    for (const id of ['C7', 'C6', 'C5', 'C4', 'C3', 'C2', 'C1']) setSegScale(id, diagS(1, kn, 1));
    setSegScale('skull', diagS(kh, kh, kh));
    for (const side of SIDES) {
      const x = SFX[side];
      const gh = V3(J[`glenohumeral_${x}`].center), hu = V3(J[`humeroulnar_${x}`].center), rc = V3(J[`radiocarpal_${x}`].center);
      const sc = V3(J[`sternoclavicular_${x}`].center), ac = V3(J[`acromioclavicular_${x}`].center);
      // clavicle: choose its x-scale so the GH centre lands at shoulderHalfWidth
      const target = (dims?.shoulderHalfWidth ?? restDims.shoulderHalfWidth);
      const sgn = side === 'left' ? 1 : -1;
      const rest = Math.abs(gh.x - (pelvisCenter.x));
      const kc = lim(1 + (target - rest) / Math.abs(ac.x - sc.x));
      setSegScale(`clavicle_${x}`, diagS(kc, 1, 1)); void sgn;
      setSegScale(`humerus_${x}`, axialS(hu.clone().sub(gh), k('upperArm')));
      const fa = axialS(rc.clone().sub(hu), k('forearm'));
      setSegScale(`ulna_${x}`, fa); setSegScale(`radius_${x}`, fa);
      const kH = k('hand');
      for (const seg of order) if (seg.id.endsWith(`_${x}`) && /^(carpalsProximal|hand|thumb|index|middle|ring|little)/.test(seg.id)) setSegScale(seg.id, diagS(kH, kH, kH));
      const hip = V3(J[`hip_${x}`].center), knee = V3(J[`knee_${x}`].center), ank = V3(J[`talocrural_${x}`].center);
      const th = axialS(knee.clone().sub(hip), k('thigh'));
      setSegScale(`femur_${x}`, th); setSegScale(`patella_${x}`, th);
      setSegScale(`tibia_${x}`, axialS(ank.clone().sub(knee), k('shank')));
      const kf = k('footLength'), ka = k('ankleHeight');
      setSegScale(`talus_${x}`, diagS(kf, ka, kf)); setSegScale(`foot_${x}`, diagS(kf, ka, kf));
    }
    // child pivots follow the parent's scaled geometry → joints stay connected
    for (const seg of order) if (seg.parent) { seg.basePos = restOffset(seg).applyMatrix3(segs[seg.parent].S); seg.group.position.copy(seg.basePos); }
    for (const a of axisObjs) a.holder.position.copy(a.seg.basePos);
    updatePatellaGeom(); updateThoraxHull();
    setPose(state.pose);
    root.updateMatrixWorld(true);
  }
  updatePatellaGeom(); updateThoraxHull();

  // ---------------- queries ----------------
  const tmpV = new THREE.Vector3();
  const landmarkWorld = (name, target = new THREE.Vector3()) => {
    const o = landmarkObjs[name]; if (!o) return null; o.updateWorldMatrix(true, false); return target.setFromMatrixPosition(o.matrixWorld);
  };
  const jointWorld = (segId, target = new THREE.Vector3()) => { const s = segs[segId]; s.group.updateWorldMatrix(true, false); return target.setFromMatrixPosition(s.group.matrixWorld); };
  const segmentWorldQuaternion = (segId, target = new THREE.Quaternion()) => { const s = segs[segId]; s.group.updateWorldMatrix(true, false); return s.group.getWorldQuaternion(target); };
  const pickables = () => order.flatMap((s) => s.meshes);
  void tmpV;

  setPose(state.pose);
  applyMaterials();
  if (scene) scene.add(root);

  return {
    root, segments: segs, order, meshes, rigData: R, materials: mats, skinMesh,
    setPose, getPose: () => structuredClone(state.pose), getDerived: () => state.derived,
    setCoupling: (b) => { state.coupling = !!b; setPose(state.pose); },
    setAutoHumeralRotation: (b) => { state.autoHumeralRotation = !!b; setPose(state.pose); },
    setUseRoot: (b) => { state.useRoot = !!b; setPose(state.pose); },
    highlight, nodeInfo, setStyle, setConfidence, getStyle: () => style, setSkin, showAxes, showLandmarks, scaleToDims,
    landmarkWorld, jointWorld, segmentWorldQuaternion, pickables, solve,
    restPose: REST,
    elbowInfo: (side) => ({ ...elbowModel[side] }),
    /** current scapula penetration into the rib-cage outline beyond its rest clearance (m) */
    scapulaRibPenetration: (side) => {
      const x = SFX[side]; const Cq = segs[`clavicle_${x}`].group.quaternion; const T = Cq.clone().multiply(segs[`scapula_${x}`].group.quaternion);
      return { maxDepth: Math.max(0, scapPenetration(side, Cq, T)) };
    },
    patellaInfo: (side) => ({ restTendonLength: patellaGeom[side]?.Lrest, tautTendonLength: patellaGeom[side]?.Ltaut }),
  };
}
