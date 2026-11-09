// =====================================================================
//  pipeline.js — MediaPipe-agnostic per-frame processing:
//    raw landmarks -> (un-mirror) -> BoneSim camera frame -> One Euro on
//    landmarks -> bone-length constancy (after calibration) -> computePose
//    -> PoseStabilizer -> overlay alignment.
//  tracker.js feeds it MediaPipe results; tests feed it synthetic frames.
// =====================================================================
import { DEFAULT_DIMS, SIDES } from '../shared/conventions.js';
import { computePose, prepareBody, prepareHand, mirrorPoseLandmarks, mirrorHandLandmarks, DEFAULT_ANGLE_OPTIONS } from './angles.js';
import { LandmarkFilter, PoseStabilizer, OneEuroVector, enforceSegmentLengths } from './filters.js';
import { createCalibrator, createAutoDims } from './calibration.js';
import { computeAlignment, fitWorldToImage, refineWithImage, estimateFovY } from './alignment.js';
import { median } from './math3.js';

export const DEFAULT_PIPELINE_OPTIONS = {
  /** frames given to MediaPipe are already horizontally mirrored */
  inputMirrored: false,
  useHands: true,
  /** rescale segments to calibrated lengths (active once calibrated, or once autoDims is ready) */
  enforceBoneLengths: true,
  /** until calibrate() is run, estimate dims continuously (rolling medians of visible segments) */
  autoDims: true,
  /** One Euro on 3D body landmarks (metres, seconds) */
  landmarkFilter: { minCutoff: 1.0, beta: 10.0, dCutoff: 1.0 },
  /** One Euro on 2D landmarks (pixels) used for the overlay fit */
  landmark2DFilter: { minCutoff: 1.0, beta: 0.025, dCutoff: 1.0 },
  /** One Euro on hand landmarks (hand-local metres) */
  handFilter: { minCutoff: 2.0, beta: 40.0, dCutoff: 1.0 },
  /** PoseStabilizer options {filters:{group:{minCutoff,beta,dCutoff,maxVel}}, minConfidence, holdDecayTau} */
  stabilizer: {},
  /** overrides for angles.DEFAULT_ANGLE_OPTIONS */
  angles: {},
  /** camera vertical FOV (deg) for the perspective alignment and the image refinement:
   *  'auto' = running median of per-frame estimates (fovYDefault until enough frames) */
  fovY: 'auto',
  fovYDefault: 55,
  /** 0..1: replace world-landmark X/Y by the back-projected 2D landmarks (keeps MediaPipe depth).
   *  Makes bone directions agree with the video; 0 = use MediaPipe world landmarks as-is. */
  imageRefine: 1,
  /** max normalised distance (fraction of image diagonal) between a hand wrist and a pose wrist */
  handMatchMaxDist: 0.12,
};

export function createPipeline(options = {}) {
  let o = mergeOpts(DEFAULT_PIPELINE_OPTIONS, options);
  let dims = { ...DEFAULT_DIMS };
  let calibration = null;
  const autoDims = createAutoDims();
  let dimsSource = 'default';
  let calibrator = null, calibResolve = null, calibReject = null;
  let lf3, lf2, hf, stab, camT;
  const handLastSeen = { left: -1e9, right: -1e9 };
  let fovSamples = [];
  /** current FOV and whether it is trustworthy (fixed by the user or >= 15 informative frames) */
  function currentFov() {
    if (typeof o.fovY === 'number') return { fov: o.fovY, sure: true };
    if (fovSamples.length >= 15) return { fov: median(fovSamples), sure: true };
    return { fov: o.fovYDefault, sure: false };
  }
  const resetFilters = () => {
    lf3 = new LandmarkFilter(o.landmarkFilter);
    lf2 = new LandmarkFilter(o.landmark2DFilter);
    hf = { left: new LandmarkFilter(o.handFilter), right: new LandmarkFilter(o.handFilter) };
    stab = new PoseStabilizer(o.stabilizer);
    camT = new OneEuroVector({ minCutoff: 0.5, beta: 0.5, dCutoff: 1 });
  };
  resetFilters();
  let lastPose = null;

  function angleOptions() {
    const a = { ...DEFAULT_ANGLE_OPTIONS, ...o.angles };
    if (calibration?.neckPitchOffset !== undefined && o.angles.neckPitchOffset === undefined) a.neckPitchOffset = calibration.neckPitchOffset;
    if (calibration?.gravityUp && !o.angles.gravityUp) a.gravityUp = calibration.gravityUp;
    return a;
  }

  /** assign detected hands to the subject's anatomical sides by wrist proximity */
  function assignHands(hands, lm2d, width, height) {
    const out = {};
    if (!o.useHands || !hands?.length || !lm2d) return out;
    const diag = Math.hypot(width, height);
    const cands = [];
    hands.forEach((h, hi) => {
      if (!h?.landmarks?.length || !h.worldLandmarks?.length) return;
      const n = o.inputMirrored ? mirrorHandLandmarks(h.landmarks, 'normalized') : h.landmarks;
      for (const [side, wi] of [['left', 15], ['right', 16]]) {
        const pw = lm2d[wi]; if (!pw) continue;
        const d = Math.hypot((n[0].x - pw.x) * width, (n[0].y - pw.y) * height) / diag;
        cands.push({ d, hi, side, n, h });
      }
    });
    cands.sort((a, b) => a.d - b.d);
    const usedH = new Set();
    for (const c of cands) {
      if (usedH.has(c.hi) || out[c.side] || c.d > o.handMatchMaxDist) continue;
      usedH.add(c.hi);
      const score = c.h.handedness?.[0]?.score ?? c.h.score ?? 1;
      out[c.side] = { points: prepareHand(c.h.worldLandmarks, { inputMirrored: o.inputMirrored }), landmarks2D: c.n, score };
    }
    return out;
  }

  /**
   * @param frame {t: ms, width, height, landmarks?: 33 normalized, worldLandmarks?: 33 world,
   *               hands?: [{landmarks, worldLandmarks, handedness:[{categoryName,score}]}]}
   */
  function process(frame) {
    const t = frame.t / 1000;
    const { width = 640, height = 480 } = frame;
    if (!frame.worldLandmarks || !frame.landmarks) {
      const pose = lastPose ? stab.update(null, null, null, t) : null;
      lastPose = pose;
      return { t: frame.t, detected: false, pose, confidence: 0, landmarks2D: null, worldLandmarks: null, bodyPoints: null, hands: [], alignment: null, dims, dimsSource, calibrated: !!calibration };
    }
    const lm2dRaw = o.inputMirrored ? mirrorPoseLandmarks(frame.landmarks, 'normalized') : frame.landmarks;
    let worldRaw = o.inputMirrored ? mirrorPoseLandmarks(frame.worldLandmarks, 'world') : frame.worldLandmarks;
    // joints outside the image are extrapolated by MediaPipe (often still with visibility
    // 0.2–0.5): treat them as invisible so they hold/decay instead of driving the rig
    const off = (l) => !l || l.x < -0.05 || l.x > 1.05 || l.y < -0.05 || l.y > 1.05;
    if (lm2dRaw.some(off)) worldRaw = worldRaw.map((w, i) => (off(lm2dRaw[i]) ? { ...w, visibility: 0, presence: 0 } : w));
    const body = prepareBody(worldRaw, { inputMirrored: false });
    let pts3 = body.points;
    if (o.fovY === 'auto') {
      const e = estimateFovY(body.points, body.vis, lm2dRaw, width, height);
      if (e?.informative) { fovSamples.push(e.fovY); if (fovSamples.length > 150) fovSamples.shift(); }
    }
    const fov = currentFov();
    if (o.imageRefine > 0) {
      const fit = fitWorldToImage(body.points, body.vis, lm2dRaw, width, height, fov.fov);
      if (fit) {
        const T = camT.filter(fit.T, t);
        pts3 = refineWithImage(body.points, body.vis, lm2dRaw, width, height, { T, f: fit.f }, o.imageRefine * (fov.sure ? 1 : 0.75));
      }
    }
    const pts = lf3.filter(pts3, t, body.vis);

    // 2D (pixels) filter for the overlay fit
    const px = lm2dRaw.map((l) => [l.x * width, l.y * height]);
    const pxF = lf2.filter(px, t, body.vis);
    const lm2d = lm2dRaw.map((l, i) => ({ ...l, x: pxF[i][0] / width, y: pxF[i][1] / height }));

    // hands
    const handsBySide = assignHands(frame.hands, lm2dRaw, width, height);
    for (const side of SIDES) {
      const h = handsBySide[side];
      if (!h) continue;
      if (t - handLastSeen[side] > 0.3) hf[side].reset();
      handLastSeen[side] = t;
      h.points = hf[side].filter(h.points, t);
    }

    if (calibrator) {
      calibrator.add({ points: pts, vis: body.vis, hands: handsBySide }, frame.t);
      if (calibrator.isDone(frame.t)) {
        try { const c = calibrator.finish(); setCalibration(c); calibResolve?.(c); }
        catch (e) { calibReject?.(e); }
        calibrator = null; calibResolve = calibReject = null;
      }
    }

    if (!calibration && o.autoDims) {
      const a = autoDims.add({ points: pts, vis: body.vis, hands: handsBySide });
      if (a) { dims = { ...DEFAULT_DIMS, ...a.dims }; dimsSource = 'auto'; }
    }
    const ptsL = (calibration || (o.autoDims && autoDims.ready)) && o.enforceBoneLengths ? enforceSegmentLengths(pts, dims) : pts;
    const ang = computePose({ points: ptsL, vis: body.vis }, { ...angleOptions(), hands: handsBySide, hold: stab.holdState() });
    const pose = stab.update(ang.pose, ang.confidence, ang.observability, t);
    lastPose = pose;

    const alignment = computeAlignment({ pose, dims, landmarks2D: lm2d, width, height, inputMirrored: o.inputMirrored, fovY: fov.fov });
    if (alignment) alignment.fovEstimated = o.fovY === 'auto' ? fov.sure : null;
    const torso = [11, 12, 23, 24].map((i) => body.vis[i]);
    return {
      t: frame.t, detected: true, pose,
      confidence: Math.min(...torso),
      landmarks2D: {
        normalized: lm2d, raw: lm2dRaw,
        pixels: lm2d.map((l) => [l.x * width, l.y * height]),
        width, height, inputMirrored: o.inputMirrored,
      },
      worldLandmarks: worldRaw, bodyPoints: ptsL,
      hands: SIDES.filter((s) => handsBySide[s]).map((s) => ({ side: s, score: handsBySide[s].score, landmarks2D: handsBySide[s].landmarks2D, worldPoints: handsBySide[s].points })),
      alignment, dims, dimsSource, calibrated: !!calibration,
      camera: { fovY: fov.fov, fovEstimated: o.fovY === 'auto' ? fov.sure : null },
      debug: { rawPose: ang.pose, observability: ang.observability },
    };
  }

  function setCalibration(c) {
    calibration = c || null;
    dims = c?.dims ? { ...DEFAULT_DIMS, ...c.dims } : { ...(autoDims.current?.dims ? { ...DEFAULT_DIMS, ...autoDims.current.dims } : DEFAULT_DIMS) };
    dimsSource = c ? 'calibrated' : autoDims.current ? 'auto' : 'default';
  }

  return {
    process,
    /** start a calibration capture; resolves with the calibration result */
    calibrate({ durationMs = 2500 } = {}) {
      calibReject?.(new Error('calibration restarted'));
      calibrator = createCalibrator({ durationMs });
      return new Promise((res, rej) => { calibResolve = res; calibReject = rej; });
    },
    get calibrating() { return !!calibrator; },
    setCalibration, getCalibration: () => calibration,
    get dims() { return dims; },
    setOptions(opts) {
      const prev = o; o = mergeOpts(o, opts);
      if (opts.landmarkFilter || opts.landmark2DFilter || opts.handFilter || opts.stabilizer || opts.inputMirrored !== undefined && opts.inputMirrored !== prev.inputMirrored) resetFilters();
    },
    get options() { return o; },
    reset() { resetFilters(); lastPose = null; fovSamples = []; autoDims.reset(); if (!calibration) { dims = { ...DEFAULT_DIMS }; dimsSource = 'default'; } },
    get fovY() { return currentFov(); },
  };
}

function mergeOpts(base, extra) {
  const out = { ...base };
  for (const [k, v] of Object.entries(extra || {})) {
    out[k] = v && typeof v === 'object' && !Array.isArray(v) && base[k] && typeof base[k] === 'object' ? { ...base[k], ...v } : v;
  }
  return out;
}
