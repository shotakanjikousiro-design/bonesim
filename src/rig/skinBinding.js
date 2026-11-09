// =====================================================================
//  skinBinding.js — bind the static BodyParts3D skin surface to the bone
//  segments so the translucent skin follows the skeleton.
//
//  Approximation: linear-blend skinning with ≤ 2 influences per vertex.
//  For each skin vertex we find the nearest bone-surface sample of each
//  nearby segment (uniform grid, 3 cm cells), keep the two closest segments
//  that are anatomically adjacent (parent/child/sibling), and weight them by
//  inverse distance^4. Good enough for a semi-transparent "body outline";
//  it does not model muscle bulging, skin sliding, etc.
// =====================================================================
import * as THREE from 'three';

export function computeSkinWeights(skinPositions, segPoints, segIndex, adjacency, cell = 0.03) {
  // segPoints: Float32Array of xyz, segIndex: Int16/Int32Array (same count)
  const grid = new Map();
  const key = (i, j, k) => `${i},${j},${k}`;
  const nPts = segIndex.length;
  for (let p = 0; p < nPts; p++) {
    const i = Math.floor(segPoints[3 * p] / cell), j = Math.floor(segPoints[3 * p + 1] / cell), k = Math.floor(segPoints[3 * p + 2] / cell);
    const kk = key(i, j, k); let a = grid.get(kk); if (!a) grid.set(kk, a = []); a.push(p);
  }
  const nV = skinPositions.length / 3;
  const idx = new Uint16Array(nV * 4), wts = new Float32Array(nV * 4);
  const best = new Map();
  for (let v = 0; v < nV; v++) {
    const x = skinPositions[3 * v], y = skinPositions[3 * v + 1], z = skinPositions[3 * v + 2];
    const ci = Math.floor(x / cell), cj = Math.floor(y / cell), ck = Math.floor(z / cell);
    best.clear(); let found = -1;
    for (let r = 0; r <= 8; r++) {
      for (let i = ci - r; i <= ci + r; i++) for (let j = cj - r; j <= cj + r; j++) for (let k = ck - r; k <= ck + r; k++) {
        if (Math.max(Math.abs(i - ci), Math.abs(j - cj), Math.abs(k - ck)) !== r) continue; // shell only
        const a = grid.get(key(i, j, k)); if (!a) continue;
        for (const p of a) {
          const dx = segPoints[3 * p] - x, dy = segPoints[3 * p + 1] - y, dz = segPoints[3 * p + 2] - z;
          const d2 = dx * dx + dy * dy + dz * dz, s = segIndex[p];
          const b = best.get(s); if (b === undefined || d2 < b) best.set(s, d2);
        }
      }
      if (best.size && found < 0) found = r;
      if (found >= 0 && r >= found + 1) break;
    }
    let s1 = -1, d1 = Infinity;
    for (const [s, d2] of best) if (d2 < d1) { d1 = d2; s1 = s; }
    let s2 = -1, d2b = Infinity;
    for (const [s, d2] of best) if (s !== s1 && d2 < d2b && adjacency(s1, s)) { d2b = d2; s2 = s; }
    if (s1 < 0) { idx[4 * v] = 0; wts[4 * v] = 1; continue; }
    d1 = Math.sqrt(d1); const dd2 = Math.sqrt(d2b);
    let w1 = 1, w2 = 0;
    if (s2 >= 0 && dd2 < d1 * 1.8 + 0.01) {
      const a = 1 / Math.pow(d1 + 0.004, 4), b = 1 / Math.pow(dd2 + 0.004, 4);
      w1 = a / (a + b); w2 = b / (a + b);
    }
    idx[4 * v] = s1; wts[4 * v] = w1;
    idx[4 * v + 1] = s2 >= 0 ? s2 : 0; wts[4 * v + 1] = w2;
  }
  return { skinIndex: idx, skinWeight: wts };
}

/**
 * Turn `skinMesh` (vertices in world rest coordinates) into a SkinnedMesh
 * bound to `bones` (Object3Ds whose matrixWorld at call time is the rest).
 * `parent` must map world rest coords to its local frame as identity offset
 * (i.e. a holder whose children use world rest coordinates).
 */
export function makeSkinnedSkin(skinMesh, bones, boneMeshes, adjacency, parent, near = adjacency) {
  const pts = []; const sid = [];
  boneMeshes.forEach((meshes, b) => {
    for (const m of meshes) {
      const pos = m.geometry.attributes.position; const step = Math.max(1, Math.floor(pos.count / 400));
      for (let i = 0; i < pos.count; i += step) { pts.push(pos.getX(i), pos.getY(i), pos.getZ(i)); sid.push(b); }
    }
  });
  const geom = skinMesh.geometry.clone();
  const { skinIndex, skinWeight } = computeSkinWeights(geom.attributes.position.array, new Float32Array(pts), Int32Array.from(sid), adjacency);
  geom.setAttribute('skinIndex', new THREE.Uint16BufferAttribute(skinIndex, 4));
  geom.setAttribute('skinWeight', new THREE.Float32BufferAttribute(skinWeight, 4));
  // drop "bridge" triangles whose corners belong to anatomically distant segments
  // (e.g. inner arm ↔ chest wall, hand ↔ thigh): in the rest mesh they touch, but
  // they would stretch into long slivers as soon as the limb moves.
  if (geom.index) {
    const idx = geom.index.array; const keep = [];
    for (let t = 0; t < idx.length; t += 3) {
      const a = skinIndex[4 * idx[t]], b = skinIndex[4 * idx[t + 1]], c = skinIndex[4 * idx[t + 2]];
      if (near(a, b) && near(b, c) && near(a, c)) keep.push(idx[t], idx[t + 1], idx[t + 2]);
    }
    geom.setIndex(keep);
  }
  const sm = new THREE.SkinnedMesh(geom, skinMesh.material);
  sm.name = 'skin'; sm.frustumCulled = false;
  parent.add(sm);
  parent.updateMatrixWorld(true);
  const skeleton = new THREE.Skeleton(bones);
  sm.bind(skeleton);
  return sm;
}
