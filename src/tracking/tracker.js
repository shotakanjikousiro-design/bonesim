// =====================================================================
//  tracker.js — webcam -> MediaPipe PoseLandmarker (+ HandLandmarker) ->
//  pipeline.js -> onResult({pose, confidence, landmarks2D, worldLandmarks,
//  hands, alignment, fps, ...}).
//
//  Model loading (local-first): vendor/mediapipe/models/*.task (vendored,
//  Apache-2.0, see vendor/mediapipe/README.md) -> official Google URL
//  (storage.googleapis.com). WASM: vendor/mediapipe/wasm -> jsDelivr.
//
//  Usage:
//    const tracker = await createTracker({ video, onResult, options: { hands: true } });
//    tracker.start();  ...  await tracker.calibrate();  ...  tracker.dispose();
// =====================================================================
import './noTelemetry.js'; // must come first: blocks MediaPipe's usage-statistics upload
import { FilesetResolver, PoseLandmarker, HandLandmarker } from '../../vendor/mediapipe/vision_bundle.mjs';
import { createPipeline } from './pipeline.js';
import { loadBinary, isOwnFile } from '../shared/binAsset.js';

const VENDOR = new URL('../../vendor/mediapipe/', import.meta.url);
const GCS = 'https://storage.googleapis.com/mediapipe-models';
export const MODEL_SOURCES = {
  pose_lite: [new URL('models/pose_landmarker_lite.task', VENDOR).href, `${GCS}/pose_landmarker/pose_landmarker_lite/float16/latest/pose_landmarker_lite.task`],
  pose_full: [new URL('models/pose_landmarker_full.task', VENDOR).href, `${GCS}/pose_landmarker/pose_landmarker_full/float16/latest/pose_landmarker_full.task`],
  pose_heavy: [new URL('models/pose_landmarker_heavy.task', VENDOR).href, `${GCS}/pose_landmarker/pose_landmarker_heavy/float16/latest/pose_landmarker_heavy.task`],
  hand: [new URL('models/hand_landmarker.task', VENDOR).href, `${GCS}/hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task`],
};
export const WASM_SOURCES = [new URL('wasm', VENDOR).href.replace(/\/$/, ''), 'https://cdn.jsdelivr.net/npm/@mediapipe/tasks-vision@1.2.2/wasm'];

export const DEFAULT_TRACKER_OPTIONS = {
  modelVariant: 'full',          // 'lite' (phones) | 'full' | 'heavy'
  hands: true,                   // run HandLandmarker (better pronation / wrist / fingers)
  handsEvery: 1,                 // run hands every N frames (2 on slow phones)
  delegate: 'GPU',               // 'GPU' with automatic CPU fallback, or 'CPU'
  autoDelegate: true,            // if GPU inference is pathologically slow (software GL), switch to CPU
  minPoseDetectionConfidence: 0.5, minPosePresenceConfidence: 0.5, minTrackingConfidence: 0.5,
  minHandDetectionConfidence: 0.5, minHandPresenceConfidence: 0.5,
  onStatus: null,                // (msg:string) => void
  // ...plus any pipeline option (inputMirrored, landmarkFilter, stabilizer, angles, fovY, ...)
};

const bufferCache = new Map();
async function fetchFirst(urls, status) {
  let lastErr;
  for (const url of urls) {
    if (bufferCache.has(url)) return { url, buffer: bufferCache.get(url) };
    try {
      let buffer;
      if (isOwnFile(url)) buffer = await loadBinary(url); // fetch, or base64 module on the hosted page
      else {
        const r = await fetch(url);
        if (!r.ok) throw new Error(`${r.status} ${r.statusText}`);
        buffer = new Uint8Array(await r.arrayBuffer());
      }
      if (buffer.byteLength < 1000) throw new Error('file too small');
      bufferCache.set(url, buffer);
      return { url, buffer };
    } catch (e) { lastErr = e; status?.(`model not available at ${url} (${e.message}), trying next source`); }
  }
  throw new Error(`could not load model from any source: ${urls.join(', ')} — ${lastErr?.message}`);
}

let filesetPromise = null;
async function getFileset(status) {
  if (!filesetPromise) filesetPromise = (async () => {
    for (const base of WASM_SOURCES) {
      try {
        const r = await fetch(`${base}/vision_wasm_internal.js`, { method: 'HEAD' });
        if (!r.ok) throw new Error(r.status);
        const w = await fetch(`${base}/vision_wasm_internal.wasm`, { method: 'HEAD' }).catch(() => null);
        if (w?.ok) return await FilesetResolver.forVisionTasks(base);
        if (!isOwnFile(base)) throw new Error('wasm missing');
        // hosted page build: the .wasm is shipped as a base64 module -> blob: URL
        const bytes = await loadBinary(`${base}/vision_wasm_internal.wasm`);
        const wasmBinaryPath = URL.createObjectURL(new Blob([bytes], { type: 'application/wasm' }));
        return { wasmLoaderPath: `${base}/vision_wasm_internal.js`, wasmBinaryPath };
      } catch (e) { status?.(`wasm not available at ${base}`); }
    }
    throw new Error('MediaPipe wasm not found');
  })();
  return filesetPromise;
}

async function createWithFallback(Task, fileset, buffer, delegate, extra, status) {
  const tries = delegate === 'GPU' ? ['GPU', 'CPU'] : ['CPU'];
  let err;
  for (const d of tries) {
    try {
      const task = await Task.createFromOptions(fileset, { baseOptions: { modelAssetBuffer: buffer, delegate: d }, runningMode: 'VIDEO', ...extra });
      return { task, delegate: d };
    } catch (e) { err = e; status?.(`${d} delegate failed (${e.message ?? e}), falling back`); }
  }
  throw err;
}

/**
 * @param {{video: HTMLVideoElement, onResult: (r)=>void, options?: object}} args
 */
export async function createTracker({ video, onResult, options = {} }) {
  let o = { ...DEFAULT_TRACKER_OPTIONS, ...options };
  const status = (m) => { o.onStatus?.(m); };
  const pipeline = createPipeline(options);
  const fileset = await getFileset(status);
  const info = { poseModel: null, handModel: null, poseDelegate: null, handDelegate: null };

  let pose = null, hand = null;
  async function loadPose() {
    const key = `pose_${o.modelVariant}`;
    status(`loading ${key} …`);
    const { url, buffer } = await fetchFirst(MODEL_SOURCES[key], status);
    const { task, delegate } = await createWithFallback(PoseLandmarker, fileset, buffer, o.delegate, {
      numPoses: 1, outputSegmentationMasks: false,
      minPoseDetectionConfidence: o.minPoseDetectionConfidence, minPosePresenceConfidence: o.minPosePresenceConfidence,
      minTrackingConfidence: o.minTrackingConfidence,
    }, status);
    pose?.close(); pose = task;
    Object.assign(info, { poseModel: url, poseDelegate: delegate, poseVariant: o.modelVariant });
    status(`pose model ${key}: ${url.startsWith(VENDOR.href) ? 'local' : 'remote'} (${delegate})`);
  }
  async function loadHand() {
    const { url, buffer } = await fetchFirst(MODEL_SOURCES.hand, status);
    const { task, delegate } = await createWithFallback(HandLandmarker, fileset, buffer, o.delegate, {
      numHands: 2, minHandDetectionConfidence: o.minHandDetectionConfidence,
      minHandPresenceConfidence: o.minHandPresenceConfidence, minTrackingConfidence: o.minTrackingConfidence,
    }, status);
    hand?.close(); hand = task;
    Object.assign(info, { handModel: url, handDelegate: delegate });
  }
  await loadPose();
  if (o.hands) { try { await loadHand(); } catch (e) { status(`hands disabled: ${e.message}`); o.hands = false; } }

  // GPU sanity benchmark: some machines expose only a software WebGL; CPU (XNNPACK) is then much faster
  let benchTimes = [], benchDone = !o.autoDelegate;
  function benchmark(ms) {
    if (benchDone || info.poseDelegate !== 'GPU') return;
    benchTimes.push(ms);
    if (benchTimes.length < 10) return;
    benchDone = true;
    const tail = benchTimes.slice(3).sort((a, b) => a - b);
    const med = tail[tail.length >> 1];
    if (med > 120) {
      status(`GPU inference slow (${med.toFixed(0)} ms/frame) — switching to CPU`);
      o.delegate = 'CPU';
      loadPose().then(() => (o.hands ? loadHand() : null)).catch((e) => status(`CPU fallback failed: ${e.message}`));
    }
  }
  let running = false, rafId = 0, vfcId = 0, lastTs = 0, lastVideoTime = -1, frameNo = 0;
  let lastHands = null, fps = 0, lastFrameT = 0, busy = false;
  const hasVFC = 'requestVideoFrameCallback' in HTMLVideoElement.prototype;
  const schedule = () => {
    if (!running) return;
    if (hasVFC) vfcId = video.requestVideoFrameCallback(step); else rafId = requestAnimationFrame(step);
  };
  function step() {
    schedule();
    if (busy || !pose || video.readyState < 2 || !video.videoWidth) return;
    if (!hasVFC && video.currentTime === lastVideoTime) return;
    lastVideoTime = video.currentTime;
    busy = true;
    try {
      let ts = performance.now(); if (ts <= lastTs) ts = lastTs + 1; lastTs = ts;
      const t0 = performance.now();
      const pr = pose.detectForVideo(video, ts);
      benchmark(performance.now() - t0);
      let handsRes = lastHands;
      if (o.hands && hand && (frameNo % Math.max(1, o.handsEvery) === 0)) {
        const hr = hand.detectForVideo(video, ts);
        handsRes = (hr.landmarks || []).map((lm, i) => ({ landmarks: lm, worldLandmarks: hr.worldLandmarks?.[i], handedness: hr.handedness?.[i] ?? hr.handednesses?.[i] }));
        lastHands = handsRes;
      } else if (!o.hands) handsRes = null;
      frameNo++;
      const lm = pr.landmarks?.[0], wl = pr.worldLandmarks?.[0];
      // world landmarks carry no reliable visibility in some builds: copy from image landmarks
      const world = wl && lm ? wl.map((w, i) => ({ x: w.x, y: w.y, z: w.z, visibility: lm[i]?.visibility ?? w.visibility, presence: lm[i]?.presence ?? w.presence })) : null;
      const res = pipeline.process({ t: ts, width: video.videoWidth, height: video.videoHeight, landmarks: lm || null, worldLandmarks: world, hands: handsRes });
      const dt = lastFrameT ? (ts - lastFrameT) / 1000 : 0; lastFrameT = ts;
      if (dt > 0) fps = fps ? fps + (1 / dt - fps) * 0.1 : 1 / dt;
      res.fps = fps; res.info = info; res.calibrating = pipeline.calibrating;
      onResult?.(res);
    } catch (e) {
      status(`tracking error: ${e.message ?? e}`);
    } finally { busy = false; }
  }

  const stop = () => { running = false; if (hasVFC && vfcId) video.cancelVideoFrameCallback?.(vfcId); cancelAnimationFrame(rafId); };
  return {
    start() { if (running) return; running = true; schedule(); },
    stop,
    /** stand still facing the camera, arms slightly away from the body */
    calibrate(opts) { return pipeline.calibrate(opts); },
    setCalibration: (c) => pipeline.setCalibration(c),
    getCalibration: () => pipeline.getCalibration(),
    async setOptions(opts = {}) {
      const prev = o; o = { ...o, ...opts };
      pipeline.setOptions(opts);
      if (opts.modelVariant && opts.modelVariant !== prev.modelVariant || opts.delegate && opts.delegate !== prev.delegate) await loadPose();
      if (o.hands && !hand) { try { await loadHand(); } catch (e) { status(`hands unavailable: ${e.message}`); o.hands = false; } }
      if (!o.hands) lastHands = null;
    },
    get options() { return { ...o, ...pipeline.options }; },
    get info() { return info; },
    pipeline,
    dispose() { stop(); pose?.close(); hand?.close(); pose = hand = null; },
  };
}
