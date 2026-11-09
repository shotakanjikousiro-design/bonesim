// Small dependency-free 3D math helpers used by the tracking code.
// Vectors are plain arrays [x, y, z]. 3x3 matrices are row-major arrays of 9
// numbers; a rotation matrix built with cols(X, Y, Z) maps local -> world
// (its columns are the local axes expressed in world coordinates).
// Euler helpers follow three.js conventions (intrinsic, R = R1 * R2 * R3).

export const D2R = Math.PI / 180;
export const R2D = 180 / Math.PI;

export const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
export const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
export const scale = (a, k) => [a[0] * k, a[1] * k, a[2] * k];
export const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
export const cross = (a, b) => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];
export const len = (a) => Math.hypot(a[0], a[1], a[2]);
export const dist = (a, b) => len(sub(a, b));
export const mid = (a, b) => [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2, (a[2] + b[2]) / 2];
export const lerp = (a, b, t) => [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t];
export function normalize(a, fallback = [0, 0, 0]) {
  const l = len(a);
  return l > 1e-12 ? [a[0] / l, a[1] / l, a[2] / l] : fallback.slice();
}
/** component of v perpendicular to unit axis */
export const perp = (v, axis) => sub(v, scale(axis, dot(v, axis)));
export const clamp = (x, lo, hi) => Math.min(hi, Math.max(lo, x));
/** unsigned angle between two vectors, degrees */
export function angleBetween(a, b) {
  const la = len(a), lb = len(b);
  if (la < 1e-12 || lb < 1e-12) return 0;
  // atan2 formulation is accurate near 0 and 180
  return Math.atan2(len(cross(a, b)), dot(a, b)) * R2D;
}
/** wrap degrees to [-180, 180) */
export function wrapDeg(a) {
  let x = ((a + 180) % 360 + 360) % 360 - 180;
  if (x === -180 && a > 0) x = 180;
  return x;
}
export function smoothstep(x, a, b) {
  const t = clamp((x - a) / (b - a), 0, 1);
  return t * t * (3 - 2 * t);
}
/** w*a + (1-w)*b along the shortest arc (degrees) */
export function angleBlend(a, b, w) {
  if (w >= 1) return a;
  if (w <= 0) return b;
  return b + wrapDeg(a - b) * w;
}
/** weighted circular mean of angles (degrees) — fine for clustered angles */
export function angleWeightedMean(angles, weights) {
  let sx = 0, sy = 0, sw = 0;
  for (let i = 0; i < angles.length; i++) {
    const w = weights[i]; if (!(w > 0)) continue;
    sx += w * Math.cos(angles[i] * D2R); sy += w * Math.sin(angles[i] * D2R); sw += w;
  }
  if (sw <= 0) return { angle: 0, weight: 0 };
  return { angle: Math.atan2(sy, sx) * R2D, weight: sw };
}

// ---------------- 3x3 matrices (row-major) ----------------
export const I3 = () => [1, 0, 0, 0, 1, 0, 0, 0, 1];
export const cols = (x, y, z) => [x[0], y[0], z[0], x[1], y[1], z[1], x[2], y[2], z[2]];
export const col = (m, j) => [m[j], m[3 + j], m[6 + j]];
export const transpose = (m) => [m[0], m[3], m[6], m[1], m[4], m[7], m[2], m[5], m[8]];
export function mul(a, b) {
  const r = new Array(9);
  for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++)
    r[i * 3 + j] = a[i * 3] * b[j] + a[i * 3 + 1] * b[3 + j] + a[i * 3 + 2] * b[6 + j];
  return r;
}
export const mulN = (...ms) => ms.reduce((a, b) => mul(a, b));
export const apply = (m, v) => [
  m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
  m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
  m[6] * v[0] + m[7] * v[1] + m[8] * v[2],
];
export function rotX(deg) { const c = Math.cos(deg * D2R), s = Math.sin(deg * D2R); return [1, 0, 0, 0, c, -s, 0, s, c]; }
export function rotY(deg) { const c = Math.cos(deg * D2R), s = Math.sin(deg * D2R); return [c, 0, s, 0, 1, 0, -s, 0, c]; }
export function rotZ(deg) { const c = Math.cos(deg * D2R), s = Math.sin(deg * D2R); return [c, -s, 0, s, c, 0, 0, 0, 1]; }
export function axisAngle(axis, deg) {
  const [x, y, z] = normalize(axis); const a = deg * D2R;
  const c = Math.cos(a), s = Math.sin(a), t = 1 - c;
  return [t * x * x + c, t * x * y - s * z, t * x * z + s * y,
          t * x * y + s * z, t * y * y + c, t * y * z - s * x,
          t * x * z - s * y, t * y * z + s * x, t * z * z + c];
}
/** Build an orthonormal right-handed frame. primary axis kept exactly; secondary orthogonalised. */
export function frameXY(x, yHint) {
  const X = normalize(x);
  const Y = normalize(perp(yHint, X));
  return cols(X, Y, cross(X, Y));
}
export function frameYZ(y, zHint) {
  const Y = normalize(y);
  const Z = normalize(perp(zHint, Y));
  return cols(cross(Y, Z), Y, Z);
}
export function frameZY(z, yHint) {
  const Z = normalize(z);
  const Y = normalize(perp(yHint, Z));
  return cols(cross(Y, Z), Y, Z);
}
/** minimal rotation taking (0,-1,0) to unit vector d (same as THREE setFromUnitVectors) */
export function swingFromNegY(d) {
  const c = -d[1];
  if (1 + c < 1e-9) return rotZ(180); // degenerate (d = +Y); conventions clamp elevation < 180 anyway
  const k = [-d[2], 0, d[0]]; // (0,-1,0) x d
  const K = [0, -k[2], k[1], k[2], 0, -k[0], -k[1], k[0], 0];
  const K2 = mul(K, K); const f = 1 / (1 + c);
  const r = I3();
  for (let i = 0; i < 9; i++) r[i] += K[i] + K2[i] * f;
  return r;
}
/** R = Rx(a) Rz(b) Ry(c) -> [a, b, c] degrees (three.js 'XZY') */
export function eulerXZY(m) {
  const m12 = m[1];
  const b = Math.asin(clamp(-m12, -1, 1));
  let a, c;
  if (Math.abs(m12) < 0.9999999) { a = Math.atan2(m[7], m[4]); c = Math.atan2(m[2], m[0]); }
  else { a = Math.atan2(-m[5], m[8]); c = 0; }
  return [a * R2D, b * R2D, c * R2D];
}
/** R = Ry(a) Rx(b) Rz(c) -> [a, b, c] degrees (three.js 'YXZ') */
export function eulerYXZ(m) {
  const m23 = m[5];
  const b = Math.asin(clamp(-m23, -1, 1));
  let a, c;
  if (Math.abs(m23) < 0.9999999) { a = Math.atan2(m[2], m[8]); c = Math.atan2(m[3], m[4]); }
  else { a = Math.atan2(-m[6], m[0]); c = 0; }
  return [a * R2D, b * R2D, c * R2D];
}
/** rotation matrix -> quaternion [x, y, z, w] */
export function matToQuat(m) {
  const [m11, m12, m13, m21, m22, m23, m31, m32, m33] = m;
  const tr = m11 + m22 + m33; let x, y, z, w;
  if (tr > 0) { const s = 0.5 / Math.sqrt(tr + 1); w = 0.25 / s; x = (m32 - m23) * s; y = (m13 - m31) * s; z = (m21 - m12) * s; }
  else if (m11 > m22 && m11 > m33) { const s = 2 * Math.sqrt(1 + m11 - m22 - m33); w = (m32 - m23) / s; x = 0.25 * s; y = (m12 + m21) / s; z = (m13 + m31) / s; }
  else if (m22 > m33) { const s = 2 * Math.sqrt(1 + m22 - m11 - m33); w = (m13 - m31) / s; x = (m12 + m21) / s; y = 0.25 * s; z = (m23 + m32) / s; }
  else { const s = 2 * Math.sqrt(1 + m33 - m11 - m22); w = (m21 - m12) / s; x = (m13 + m31) / s; y = (m23 + m32) / s; z = 0.25 * s; }
  const n = Math.hypot(x, y, z, w);
  return [x / n, y / n, z / n, w / n];
}
export function quatToMat([x, y, z, w]) {
  const x2 = x + x, y2 = y + y, z2 = z + z;
  const xx = x * x2, xy = x * y2, xz = x * z2, yy = y * y2, yz = y * z2, zz = z * z2;
  const wx = w * x2, wy = w * y2, wz = w * z2;
  return [1 - (yy + zz), xy - wz, xz + wy, xy + wz, 1 - (xx + zz), yz - wx, xz - wy, yz + wx, 1 - (xx + yy)];
}
/** angle (deg) of the relative rotation between two rotation matrices */
export function rotationAngleBetween(a, b) {
  const r = mul(transpose(a), b);
  return Math.acos(clamp((r[0] + r[4] + r[8] - 1) / 2, -1, 1)) * R2D;
}
export function median(arr) {
  const a = arr.filter(Number.isFinite).sort((p, q) => p - q);
  if (!a.length) return NaN;
  const m = a.length >> 1;
  return a.length % 2 ? a[m] : (a[m - 1] + a[m]) / 2;
}
