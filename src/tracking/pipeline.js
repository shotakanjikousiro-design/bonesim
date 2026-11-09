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
import { classifyFraming, createFramingTracker, fitHead, fitHeadFromFace, jawAngleFromBlendshape, handToCamera, createQuatSmoother, defaultFovY, landmarkOk } from './closeup.js';

export const DEFAULT_PIPELINE_OPTIONS = {
  /** frames given to MediaPipe are already horizontally mirrored */
  inputMirrored: false,
  useHands: true,
  /** rescale segments to calibrated lengths (active once calibrated, or once autoDims is ready) */
  enforceBoneLengths: true,
  /** until calibrate() is run, estimate dims continuously (rolling medians of visible segments) */
  autoDims: true,
  /** One Euro on 3D body landmarks (metres, seconds) */
  landmarkFilter: { minCutoff: 1.5, beta: 14.0, dCutoff: 1.0 },
  /** One Euro on 2D landmarks (pixels) used for the overlay fit */
  landmark2DFilter: { minCutoff: 2.0, beta: 0.05, dCutoff: 1.0 },
  /** One Euro on hand landmarks (hand-local metres) */
  handFilter: { minCutoff: 2.0, beta: 40.0, dCutoff: 1.0 },
  /** PoseStabilizer options {filters:{group:{minCutoff,beta,dCutoff,maxVel}}, minConfidence, holdDecayTau} */
  stabilizer: {},
  /** overrides for angles.DEFAULT_ANGLE_OPTIONS */
  angles: {},
  /** camera vertical FOV (deg) for the perspective alignment and the image refinement:
   *  'auto' = running median of per-frame estimates (fovYDefault until enough frames) */
  fovY: 'auto',
  /** FOV used until enough informative frames: 'auto' = long image side spans ≈ 70° (webcams/phones) */
  fovYDefault: 'auto',
  /** framing changes must persist this many frames (hysteresis) */
  framingHysteresis: 5,
  /** 0..1: replace world-landmark X/Y by the back-projected 2D landmarks (keeps MediaPipe depth).
   *  Makes bone directions agree with the video; 0 = use MediaPipe world landmarks as-is. */
  imageRefine: 1,
  /** max normalised distance (fraction of image diagonal) between a hand wrist and a pose wrist */
  handMatchMaxDist: 0.12,
  /** use the face tracker result (frame.face) for the head fit and the mouth opening */
  useFace: true,
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
  let frameW = 640, frameH = 480;
  function currentFov() {
    if (typeof o.fovY === 'number') return { fov: o.fovY, sure: true };
    if (fovSamples.length >= 15) return { fov: median(fovSamples), sure: true };
    return { fov: typeof o.fovYDefault === 'number' ? o.fovYDefault : defaultFovY(frameW, frameH), sure: false };
  }
  const framingT = createFramingTracker({ frames: o.framingHysteresis });
  const headQ = createQuatSmoother(0.5), headQ2 = createQuatSmoother(0.6), faceQ = createQuatSmoother(0.7);
  let faceD, jawF, faceLastT = -1e9;
  const focal = (fov, h) => (h / 2) / Math.tan((fov * Math.PI / 180) / 2);
  /** never let learned / calibrated head-neck lengths blow up (close-ups, bad frames) */
  const sane = (d) => ({ ...d, neck: Math.min(0.14, Math.max(0.04, d.neck)), head: Math.min(0.26, Math.max(0.17, d.head)) });
  /** hands → camera-frame 3D points (for close-up hand alignment) */
  function handsOut(handsBySide, width, height, f) {
    return SIDES.filter((sd) => handsBySide[sd]).map((sd) => {
      const h = handsBySide[sd];
      return { side: sd, score: h.score, matched: h.matched !== false, landmarks2D: h.landmarks2D, worldPoints: h.points, cameraPoints: handToCamera(h.points, h.landmarks2D, width, height, f) };
    });
  }
  const resetFilters = () => {
    lf3 = new LandmarkFilter(o.landmarkFilter);
    lf2 = new LandmarkFilter(o.landmark2DFilter);
    hf = { left: new LandmarkFilter(o.handFilter), right: new LandmarkFilter(o.handFilter) };
    stab = new PoseStabilizer(o.stabilizer);
    camT = new OneEuroVector({ minCutoff: 0.5, beta: 0.5, dCutoff: 1 });
    faceD = new OneEuroVector({ minCutoff: 1.0, beta: 6, dCutoff: 1 });
    jawF = new OneEuroVector({ minCutoff: 4, beta: 0.4, dCutoff: 1 });
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
      out[c.side] = { points: prepareHand(c.h.worldLandmarks, { inputMirrored: o.inputMirrored }), landmarks2D: c.n, score, matched: true };
    }
    floatingHands(hands, out, usedH);
    return out;
  }
  /**
   * Hands that could not be tied to a pose wrist (arm outside the picture, wrist occluded, hand
   * much closer to the camera than the body…) are still shown: side from MediaPipe's handedness
   * label, `matched: false` (they are drawn on their own landmarks and do not drive the arm pose).
   */
  function floatingHands(hands, out, usedH = new Set()) {
    if (!o.useHands) return out;
    (hands || []).forEach((h, hi) => {
      if (usedH.has(hi) || !h?.landmarks?.length || !h.worldLandmarks?.length) return;
      const n = o.inputMirrored ? mirrorHandLandmarks(h.landmarks, 'normalized') : h.landmarks;
      const lab = (h.handedness?.[0]?.categoryName || 'Right').toLowerCase();
      let sd = lab === 'left' ? 'left' : 'right'; if (o.inputMirrored) sd = sd === 'left' ? 'right' : 'left';
      if (out[sd]) sd = sd === 'left' ? 'right' : 'left';
      if (out[sd]) return;
      out[sd] = { points: prepareHand(h.worldLandmarks, { inputMirrored: o.inputMirrored }), landmarks2D: n, score: h.handedness?.[0]?.score ?? 1, matched: false };
    });
    return out;
  }
  function filterHands(hb, t) {
    for (const side of SIDES) {
      const h = hb[side]; if (!h) continue;
      if (t - handLastSeen[side] > 0.3) hf[side].reset();
      handLastSeen[side] = t;
      h.points = hf[side].filter(h.points, t);
    }
    return hb;
  }
  /** head from the face tracker: {ao, quaternion, rmsPx, n, source:'face', depthScale} or null; also the jaw */
  function faceHead(face, width, height, f, t) {
    if (!o.useFace || o.inputMirrored || !face?.landmarks) return { head: null, jaw: null };
    const fit = fitHeadFromFace(face, width, height, f);
    if (!fit) return { head: null, jaw: null };
    if (t - faceLastT > 0.4) { faceD.reset(); jawF.reset(); }
    faceLastT = t;
    // depth is the noisy part of the fit: smooth it, and keep the (stable) image direction of the head
    const D = faceD.filter([-fit.ao[2]], t)[0]; const k = D / -fit.ao[2];
    const head = { ao: [fit.ao[0] * k, fit.ao[1] * k, -D], quaternion: faceQ(fit.R).q, rmsPx: fit.rmsPx, n: fit.n, source: 'face', depthScale: fit.depthScale };
    let jaw = null;
    if (Number.isFinite(face.jawOpen)) { const v = jawF.filter([face.jawOpen], t)[0]; jaw = { open: jawAngleFromBlendshape(v), raw: face.jawOpen }; }
    return { head, jaw };
  }

  /**
   * @param frame {t: ms, width, height, landmarks?: 33 normalized, worldLandmarks?: 33 world,
   *               face?: {landmarks: 478 normalized, matrix?: 16 numbers (column-major), jawOpen?: 0..1},
   *               hands?: [{landmarks, worldLandmarks, handedness:[{categoryName,score}]}]}
   */
  function process(frame) {
    const t = frame.t / 1000;
    const { width = 640, height = 480 } = frame;
    frameW = width; frameH = height;
    if (!frame.worldLandmarks || !frame.landmarks) {
      const pose = lastPose ? stab.update(null, null, null, t) : null;
      lastPose = pose;
      // no body detected: a hand close to the camera and / or a face can still be shown
      const hb = filterHands(floatingHands(frame.hands, {}), t);
      const f0 = focal(currentFov().fov, height);
      const handsRes = handsOut(hb, width, height, f0);
      const fh = faceHead(frame.face, width, height, f0, t);
      const raw = fh.head ? 'face' : handsRes.length ? 'hand' : 'none';
      const framing = framingT.update(raw);
      return { t: frame.t, detected: handsRes.length > 0 || !!fh.head, bodyDetected: false, framing, framingRaw: raw, pose, confidence: fh.head ? 1 : 0, landmarks2D: null, worldLandmarks: null, bodyPoints: null, hands: handsRes, head: fh.head, jaw: fh.jaw, alignment: null, dims, dimsSource, calibrated: !!calibration, camera: { fovY: currentFov().fov, fovEstimated: null } };
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
    const handsAll = filterHands(assignHands(frame.hands, lm2dRaw, width, height), t);
    // only hands tied to a pose wrist drive the arm angles (pronation / wrist / fingers)
    const handsBySide = {}; for (const sd of SIDES) if (handsAll[sd]?.matched) handsBySide[sd] = handsAll[sd];

    if (calibrator) {
      calibrator.add({ points: pts, vis: body.vis, hands: handsBySide }, frame.t);
      if (calibrator.isDone(frame.t)) {
        try { const c = calibrator.finish(); setCalibration(c); calibResolve?.(c); }
        catch (e) { calibReject?.(e); }
        calibrator = null; calibResolve = calibReject = null;
      }
    }

    // framing (per frame + hysteresis); close-up frames must not teach the body dims
    const f = focal(fov.fov, height);
    // several people in view: the face tracker and the pose tracker may pick different persons —
    // then the pose's own (coarser) face points are used so that head and body belong together
    let faceIn = frame.face;
    if (faceIn?.landmarks?.length >= 468 && landmarkOk(lm2dRaw[0], 0.5, 0.02)) {
      const L = faceIn.landmarks;
      const d = Math.hypot((L[1].x - lm2dRaw[0].x) * width, (L[1].y - lm2dRaw[0].y) * height);
      const fw = Math.hypot((L[454].x - L[234].x) * width, (L[454].y - L[234].y) * height);
      if (d > 0.8 * fw) faceIn = null;
    }
    const fh = faceHead(faceIn, width, height, f, t);
    const framingRaw = classifyFraming(lm2dRaw, SIDES.filter((sd) => handsAll[sd]).map((sd) => handsAll[sd]), !!fh.head);
    const framing = framingT.update(framingRaw);
    const torsoGood = [11, 12, 23, 24].every((i) => landmarkOk(lm2dRaw[i], 0.7) && body.vis[i] >= 0.7);
    if (!calibration && o.autoDims && (framingRaw === 'full' || framingRaw === 'upper') && torsoGood) {
      const a = autoDims.add({ points: pts, vis: body.vis, hands: handsBySide });
      if (a) { dims = sane({ ...DEFAULT_DIMS, ...a.dims }); dimsSource = 'auto'; }
    }
    const ptsL = (calibration || (o.autoDims && autoDims.ready)) && o.enforceBoneLengths ? enforceSegmentLengths(pts, dims) : pts;
    const ang = computePose({ points: ptsL, vis: body.vis }, { ...angleOptions(), hands: handsBySide, hold: stab.holdState() });
    const pose = stab.update(ang.pose, ang.confidence, ang.observability, t);
    lastPose = pose;

    const alignment = computeAlignment({ pose, dims, landmarks2D: lm2d, width, height, inputMirrored: o.inputMirrored, fovY: fov.fov });
    if (alignment) alignment.fovEstimated = o.fovY === 'auto' ? fov.sure : null;
    // head fitted directly to the face (eyes / ears / nose / mouth), in the camera frame
    let head = fh.head;
    if (!head && ang.frames?.head) {
      const hq = headQ(ang.frames.head);
      const fit = fitHead(lm2d, hq.R, width, height, f);
      if (fit) head = { ao: fit.ao, quaternion: headQ2(fit.R).q, rmsPx: fit.rmsPx, n: fit.n, source: 'pose', depthScale: 1 };
    }
    // confidence of what is actually in the picture (hips are not expected in a close-up)
    const visOf = (ids) => Math.min(...ids.map((i) => (landmarkOk(lm2dRaw[i], 0, 0.02) ? body.vis[i] : 0)));
    const confidence = framing === 'full' || framing === 'upper' ? visOf([11, 12, 23, 24])
      : framing === 'headShoulders' ? visOf([11, 12]) : framing === 'face' ? (head ? 1 : 0) : framing === 'hand' ? 1 : 0;
    return {
      t: frame.t, detected: true, pose,
      confidence,
      landmarks2D: {
        normalized: lm2d, raw: lm2dRaw,
        pixels: lm2d.map((l) => [l.x * width, l.y * height]),
        width, height, inputMirrored: o.inputMirrored,
      },
      worldLandmarks: worldRaw, bodyPoints: ptsL,
      hands: handsOut(handsAll, width, height, f),
      framing, framingRaw, head, jaw: fh.jaw, bodyDetected: true,
      alignment, dims, dimsSource, calibrated: !!calibration,
      camera: { fovY: fov.fov, fovEstimated: o.fovY === 'auto' ? fov.sure : null },
      debug: { rawPose: ang.pose, observability: ang.observability },
    };
  }

  function setCalibration(c) {
    calibration = c || null;
    dims = c?.dims ? sane({ ...DEFAULT_DIMS, ...c.dims }) : { ...(autoDims.current?.dims ? { ...DEFAULT_DIMS, ...autoDims.current.dims } : DEFAULT_DIMS) };
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
    reset() { resetFilters(); lastPose = null; fovSamples = []; autoDims.reset(); framingT.reset(); if (!calibration) { dims = { ...DEFAULT_DIMS }; dimsSource = 'default'; } },
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
