// =====================================================================
//  filters.js — temporal stabilisation for tracking.
//   * OneEuroFilter (scalar), OneEuroVector (isotropic, n-D), OneEuroAngle
//     (degrees, wrap-aware), LandmarkFilter (arrays of 3D/2D points).
//   * enforceSegmentLengths(): bone-length constancy after calibration.
//   * PoseStabilizer: per-angle velocity limit -> confidence-weighted One Euro
//     -> hold & decay when unobservable -> ROM clamp (conventions.clampPose).
//  One Euro filter: Casiez, Roussel & Vogel, CHI 2012.
// =====================================================================
import { ROM, SIDES, clampPose } from '../shared/conventions.js';
import { wrapDeg, clamp, normalize, sub, add, scale, mid, perp } from './math3.js';

const alphaFor = (cutoff, dt) => { const tau = 1 / (2 * Math.PI * cutoff); return 1 / (1 + tau / dt); };

export class OneEuroFilter {
  constructor({ minCutoff = 1, beta = 0, dCutoff = 1 } = {}) { Object.assign(this, { minCutoff, beta, dCutoff }); this.reset(); }
  reset() { this.x = undefined; this.dx = 0; this.t = undefined; }
  /** @param t seconds. alphaScale in (0,1] slows the filter (used for low confidence). */
  filter(x, t, alphaScale = 1) {
    if (this.x === undefined || !Number.isFinite(this.x)) { this.x = x; this.t = t; this.dx = 0; return x; }
    const dt = Math.max(1e-4, t - this.t); this.t = t;
    const dxRaw = (x - this.x) / dt;
    this.dx += alphaFor(this.dCutoff, dt) * (dxRaw - this.dx);
    const cutoff = this.minCutoff + this.beta * Math.abs(this.dx);
    this.x += alphaFor(cutoff, dt) * alphaScale * (x - this.x);
    return this.x;
  }
}

/** Angle filter in degrees: the input is unwrapped against the current state. */
export class OneEuroAngle extends OneEuroFilter {
  filter(x, t, alphaScale = 1) {
    if (this.x !== undefined) x = this.x + wrapDeg(x - this.x);
    return super.filter(x, t, alphaScale);
  }
}

/** n-D One Euro filter whose cutoff uses the speed magnitude (isotropic). */
export class OneEuroVector {
  constructor({ minCutoff = 1, beta = 0, dCutoff = 1 } = {}) { Object.assign(this, { minCutoff, beta, dCutoff }); this.reset(); }
  reset() { this.x = undefined; this.dx = undefined; this.t = undefined; }
  filter(x, t, alphaScale = 1) {
    if (!this.x) { this.x = x.slice(); this.dx = x.map(() => 0); this.t = t; return this.x.slice(); }
    const dt = Math.max(1e-4, t - this.t); this.t = t;
    const ad = alphaFor(this.dCutoff, dt);
    let sp = 0;
    for (let i = 0; i < x.length; i++) { this.dx[i] += ad * ((x[i] - this.x[i]) / dt - this.dx[i]); sp += this.dx[i] * this.dx[i]; }
    const a = alphaFor(this.minCutoff + this.beta * Math.sqrt(sp), dt) * alphaScale;
    for (let i = 0; i < x.length; i++) this.x[i] += a * (x[i] - this.x[i]);
    return this.x.slice();
  }
}

/** Filters an array of points (each [x,y,z] or [x,y]); low-visibility points move slower. */
export class LandmarkFilter {
  constructor(params = {}) { this.params = params; this.filters = []; }
  reset() { this.filters = []; }
  filter(points, t, vis) {
    return points.map((p, i) => {
      if (!p) return p;
      const f = (this.filters[i] ||= new OneEuroVector(this.params));
      const a = vis ? clamp(0.15 + vis[i], 0.15, 1) : 1;
      return f.filter(p, t, a);
    });
  }
}

/**
 * Rescale every segment of the 33 body points (BoneSim camera frame) to the
 * calibrated lengths in `dims` (DEFAULT_DIMS keys), keeping each segment's
 * direction. Chains are rebuilt outward from the hip midpoint so that
 * distal points inherit the corrected proximal positions. Points that are
 * not on a chain (face, hand knuckles, heel/toe) are translated rigidly with
 * their parent joint.
 */
export function enforceSegmentLengths(points, dims) {
  const P = points.map((p) => p.slice());
  const T = (idxs, delta) => { for (const i of idxs) P[i] = add(points[i], delta); };
  const hL = points[23], hR = points[24], sL = points[11], sR = points[12];
  const pm = mid(hL, hR);
  const xh = normalize(sub(hL, hR), [1, 0, 0]);
  P[23] = add(pm, scale(xh, dims.hipHalfWidth)); P[24] = add(pm, scale(xh, -dims.hipHalfWidth));
  const sm = mid(sL, sR);
  const trunk = normalize(sub(sm, pm), [0, 1, 0]);
  const sm2 = add(pm, scale(trunk, dims.thoraxHeight));
  // shoulder axis, orthogonal-ish to trunk as in stickFK
  const xs = normalize(perp(sub(sL, sR), trunk), [1, 0, 0]);
  P[11] = add(sm2, scale(xs, dims.shoulderHalfWidth)); P[12] = add(sm2, scale(xs, -dims.shoulderHalfWidth));
  T([0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10], sub(sm2, sm));
  const chain = (iA, iB, L) => { P[iB] = add(P[iA], scale(normalize(sub(points[iB], points[iA]), [0, -1, 0]), L)); };
  for (const [sh, el, wr, extra] of [[11, 13, 15, [17, 19, 21]], [12, 14, 16, [18, 20, 22]]]) {
    chain(sh, el, dims.upperArm); chain(el, wr, dims.forearm);
    T(extra, sub(P[wr], points[wr]));
  }
  for (const [hp, kn, an, extra] of [[23, 25, 27, [29, 31]], [24, 26, 28, [30, 32]]]) {
    chain(hp, kn, dims.thigh); chain(kn, an, dims.shank);
    T(extra, sub(P[an], points[an]));
  }
  return P;
}

// ---------------- pose-level stabiliser ----------------
/** Per-angle filter tuning. Units: degrees and seconds (beta in Hz per deg/s). */
export const DEFAULT_ANGLE_FILTER = {
  // Tuned with tests/tracking.test.mjs (1 cm landmark noise, 30 fps): the landmark-level
  // filter does most of the smoothing; this second stage mainly removes residual
  // angle jitter, so its beta is high (little extra lag while moving).
  default:  { minCutoff: 2.0, beta: 0.15, dCutoff: 1.0, maxVel: 900 },
  large:    { minCutoff: 2.0, beta: 0.15, dCutoff: 1.0, maxVel: 1000 }, // elevation, elbow, hip, knee
  axial:    { minCutoff: 1.4, beta: 0.15, dCutoff: 1.0, maxVel: 720 },  // humeral / hip rotation
  forearm:  { minCutoff: 2.0, beta: 0.15, dCutoff: 1.0, maxVel: 1000 }, // pronation, wrist
  trunk:    { minCutoff: 1.4, beta: 0.15, dCutoff: 1.0, maxVel: 400 },
  fingers:  { minCutoff: 2.0, beta: 0.15, dCutoff: 1.0, maxVel: 1500 },
};
const groupOf = (path) => {
  if (path.startsWith('spine') || path.startsWith('neck')) return 'trunk';
  if (path.includes('fingers')) return 'fingers';
  if (path.endsWith('axialRotation')) return 'axial';
  if (path.includes('forearm') || path.includes('wrist')) return 'forearm';
  if (/elevation|plane|elbow|hip\.(flexion|abduction)|knee/.test(path)) return 'large';
  return 'default';
};
const DISTAL_RE = /\.(elbow|forearm|wrist|knee|ankle|fingers)\./;
/** neutral prior that unobservable angles decay toward */
const priorOf = () => 0; // anatomical neutral (pronation 0 = thumb forward, relaxed)

function flatten(obj, prefix = '', out = {}) {
  for (const [k, v] of Object.entries(obj)) {
    if (k === 'root' || k === 'confidence') continue;
    const p = prefix ? `${prefix}.${k}` : k;
    if (typeof v === 'number') out[p] = v; else if (v && typeof v === 'object') flatten(v, p, out);
  }
  return out;
}
function setPath(obj, path, val) {
  const ks = path.split('.'); let o = obj;
  for (let i = 0; i < ks.length - 1; i++) o = (o[ks[i]] ||= {});
  o[ks[ks.length - 1]] = val;
}
/** confidence for an angle path: exact key, else joint key, else 1 */
function confFor(conf, path) {
  if (!conf) return 1;
  if (path in conf) return conf[path];
  const parts = path.split('.');
  for (let n = parts.length - 1; n > 0; n--) { const k = parts.slice(0, n).join('.'); if (k in conf) return conf[k]; }
  return 1;
}

export class PoseStabilizer {
  /**
   * @param opts {filters: partial DEFAULT_ANGLE_FILTER, minConfidence: 0.15,
   *   minConfidenceDistal: 0.3 (elbow/forearm/wrist/knee/ankle/fingers: MediaPipe reports
   *   visibility ≈ 0.2–0.5 for extrapolated off-screen joints, which must hold, not drive),
   *   holdDecayTau: 4 (s) — time constant with which unobservable/held angles relax
   *   toward the neutral prior, clamp: true}
   */
  constructor(opts = {}) { this.setOptions(opts); this.reset(); }
  setOptions(opts = {}) {
    this.opts = { minConfidence: 0.15, minConfidenceDistal: 0.3, holdDecayTau: 4, clamp: true, ...this.opts, ...opts };
    this.cfg = { ...DEFAULT_ANGLE_FILTER, ...(this.opts.filters || {}) };
    this.filters = new Map();
  }
  reset() { this.filters = new Map(); this.last = null; this.lastT = undefined; this.rootF = null; }

  /**
   * @param raw  Pose from computePose (with optional .root)
   * @param conf confidence map (computePose().confidence)
   * @param obs  observability map (computePose().observability)
   * @param t    seconds
   * @returns stabilised Pose (with .confidence copied)
   */
  update(raw, conf, obs, t) {
    const dt = this.lastT === undefined ? 1 / 30 : clamp(t - this.lastT, 1e-3, 0.5);
    this.lastT = t;
    const flat = raw ? flatten(raw) : {};
    const prevFlat = this.last ? flatten(this.last) : {};
    const out = {};
    const paths = new Set([...Object.keys(prevFlat), ...Object.keys(flat)]);
    for (const path of paths) {
      const g = this.cfg[groupOf(path)];
      let f = this.filters.get(path);
      if (!f) { f = new OneEuroAngle(g); this.filters.set(path, f); }
      const prev = prevFlat[path];
      let x = flat[path];
      const c = raw ? confFor(conf, path) : 0;
      const minC = DISTAL_RE.test(path) ? this.opts.minConfidenceDistal : this.opts.minConfidence;
      if (x === undefined || !Number.isFinite(x) || c < minC) {
        // hold, decaying slowly toward the neutral prior
        if (prev === undefined) continue;
        const prior = priorOf(path);
        const v = prior + (prev - prior) * Math.exp(-dt / this.opts.holdDecayTau);
        f.x = v; f.t = t; setPath(out, path, v); continue;
      }
      if (prev !== undefined) {
        // velocity limit (rejects single-frame flips / left-right swaps)
        const maxStep = g.maxVel * dt;
        x = prev + clamp(wrapDeg(x - prev), -maxStep, maxStep);
      }
      // confidence-weighted smoothing: low confidence -> slower
      const a = clamp(0.2 + 0.8 * c, 0.2, 1);
      let v = f.filter(x, t, a);
      // partially unobservable angles: decay held part toward prior
      const ob = obs?.[path];
      if (ob !== undefined && ob < 1) {
        const prior = priorOf(path);
        const k = 1 - Math.exp(-dt * (1 - ob) / this.opts.holdDecayTau);
        v += (prior - v) * k; f.x = v;
      }
      setPath(out, path, v);
    }
    // root quaternion: One Euro on sign-aligned components
    if (raw?.root?.quaternion) {
      let q = raw.root.quaternion.slice();
      const pq = this.last?.root?.quaternion;
      if (pq && q[0] * pq[0] + q[1] * pq[1] + q[2] * pq[2] + q[3] * pq[3] < 0) q = q.map((v) => -v);
      this.rootF ||= new OneEuroVector({ minCutoff: 1.0, beta: 0.3, dCutoff: 1 });
      const fq = this.rootF.filter(q, t, clamp(0.2 + 0.8 * confFor(conf, 'root'), 0.2, 1));
      const n = Math.hypot(...fq);
      out.root = { position: raw.root.position?.slice() ?? [0, 0, 0], quaternion: fq.map((v) => v / n) };
    } else if (this.last?.root) out.root = structuredClone(this.last.root);
    // periodic angles: clamp to the circularly NEAREST ROM bound, so values near the
    // ±180° seam do not flip between the two bounds (e.g. hand behind the back)
    if (this.opts.clamp) for (const side of SIDES) {
      const L = out[side]; if (!L) continue;
      if (L.shoulder?.plane !== undefined) L.shoulder.plane = circularClamp(L.shoulder.plane, ROM.shoulder.plane);
      if (L.forearm?.pronation !== undefined) L.forearm.pronation = circularClamp(L.forearm.pronation, ROM.forearm.pronation);
    }
    let res = this.opts.clamp ? clampPose(out) : out;
    if (this.opts.clamp) res = clampFingers(res);
    // feed clamped values back into the filter state (no wind-up beyond ROM)
    const rf = flatten(res);
    for (const [p, v] of Object.entries(rf)) { const f = this.filters.get(p); if (f && f.x !== undefined) f.x = v; }
    this.last = res;
    res.confidence = raw ? { ...conf } : {};
    return res;
  }

  /** values computePose should fall back on when an angle is unobservable */
  holdState() {
    const h = {};
    for (const side of SIDES) {
      const L = this.last?.[side]; if (!L) continue;
      h[side] = {
        shoulderAxial: L.shoulder?.axialRotation, shoulderPlane: L.shoulder?.plane,
        hipAxial: L.hip?.axialRotation, wristUlnar: L.wrist?.ulnarDeviation,
      };
    }
    return h;
  }
}

/** clamp a periodic angle (deg) into [lo, hi] choosing the circularly nearest bound */
export function circularClamp(a, [lo, hi]) {
  const x = wrapDeg(a);
  if (x >= lo && x <= hi) return x;
  const dLo = Math.abs(wrapDeg(x - lo)), dHi = Math.abs(wrapDeg(x - hi));
  return dLo < dHi ? lo : hi;
}

function clampFingers(pose) {
  for (const side of SIDES) {
    const F = pose[side]?.fingers; if (!F) continue;
    for (const [name, f] of Object.entries(F)) {
      const lim = name === 'thumb' ? ROM.thumb : ROM.finger;
      for (const k of Object.keys(f)) if (lim[k]) f[k] = clamp(f[k], lim[k][0], lim[k][1]);
    }
  }
  return pose;
}
