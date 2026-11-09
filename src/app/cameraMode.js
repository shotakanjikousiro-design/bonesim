// =====================================================================
//  cameraMode.js — Webカメラ → src/tracking/tracker.js → Pose → リグ → 映像に重畳
//
//  Overlay recipe (see src/tracking/alignment.js):
//   * the <video> is shown at its exact aspect ratio inside the "video area"
//     (contain-fit, CSS box = displayed frame), and the WebGL viewport/scissor
//     is set to the same rectangle;
//   * an OrthographicCamera(0, W, 0, −H) maps decoded-video pixels to it;
//   * rig.root.matrix = alignment.matrix (pelvis centre at the rig origin;
//     the rig applies pose.root.quaternion to the pelvis — useRoot = true);
//   * mirror (selfie) display: the <video> is mirrored with CSS and the
//     overlay with a pixel-space flip x → W − x premultiplied to the matrix
//     (negative determinant; three.js flips the face winding itself);
//   * the rig is rescaled to the tracker's dims (calibrated or default) so
//     the bone lengths are the same ones the alignment was fitted with.
// =====================================================================
import * as THREE from 'three';
import { DEFAULT_DIMS } from '../shared/conventions.js';

const CAL_KEY = 'bonesim.calibration.v1';
const isMobile = () => matchMedia('(pointer: coarse)').matches || /Android|iPhone|iPad/i.test(navigator.userAgent);

export function createCameraMode({ renderer, scene, rig, viewport, video, simCamera, ui }) {
  const $ = ui.$;
  const pelvisCenter = new THREE.Vector3(...rig.rigData.pelvisCenter);
  const ortho = new THREE.OrthographicCamera(0, 640, 0, -480, -10000, 10000);
  let active = false, tracker = null, stream = null, facing = 'user', starting = false;
  let fileUrl = null; // video-file source (blob: URL)
  const live = () => stream || fileUrl;
  let demo = null; const demoCanvas = document.getElementById('demo2d');
  let last = null; let dimsKey = ''; let lastScaleT = 0;
  let side3D = false, mirror = true;
  const rects = { video: null, view3D: null };
  const status = (m) => { $('camStatus').textContent = m || ''; };

  // ---------- UI ----------
  $('camModel').value = isMobile() ? 'lite' : 'full';
  $('camStart').addEventListener('click', () => {
    const src = $('camSource').value;
    if (src === 'demo') startDemo(); else if (src === 'file') $('camFile').click(); else start();
  });
  $('camFile').addEventListener('change', (e) => { const f = e.target.files?.[0]; e.target.value = ''; if (f) start(f); });
  $('camStop').addEventListener('click', () => stop());
  $('camCalib').addEventListener('click', () => calibrate());
  $('camFlip').addEventListener('click', async () => {
    facing = facing === 'user' ? 'environment' : 'user';
    mirror = facing === 'user'; $('camMirror').checked = mirror; applyVideoClasses();
    if (stream) { await openStream(); }
  });
  $('camMirror').addEventListener('change', (e) => { mirror = e.target.checked; applyVideoClasses(); });
  $('camSide3D').addEventListener('change', (e) => { side3D = e.target.checked; if (side3D) ui.setView('front', true); });
  $('camDim').addEventListener('change', applyVideoClasses);
  $('camHands').addEventListener('change', (e) => tracker?.setOptions({ hands: e.target.checked }));
  $('camFade').addEventListener('change', (e) => { if (!e.target.checked) rig.setConfidence(null); });
  $('camModel').addEventListener('change', (e) => tracker?.setOptions({ modelVariant: e.target.value }).catch((err) => status(`モデル切替に失敗: ${err.message}`)));
  function applyVideoClasses() {
    video.classList.toggle('mirror', mirror); demoCanvas.classList.toggle('mirror', mirror);
    video.classList.toggle('dim', $('camDim').checked);
  }
  applyVideoClasses();
  showCalibInfo(loadCalibration());

  function loadCalibration() { try { return JSON.parse(localStorage.getItem(CAL_KEY) || 'null'); } catch { return null; } }
  function showCalibInfo(c) {
    $('camCalibInfo').textContent = c?.dims
      ? `キャリブレーション済み：推定身長 ≈ ${(c.height * 100).toFixed(0)} cm（上腕 ${(c.dims.upperArm * 100).toFixed(1)} cm・前腕 ${(c.dims.forearm * 100).toFixed(1)} cm・大腿 ${(c.dims.thigh * 100).toFixed(1)} cm）`
      : 'キャリブレーション未実施（標準体格を使用）';
  }

  // ---------- camera stream ----------
  function closeFile() {
    if (!fileUrl) return;
    video.pause(); video.removeAttribute('src'); video.load();
    URL.revokeObjectURL(fileUrl); fileUrl = null; video.loop = false;
  }
  async function openFile(file) {
    stream?.getTracks().forEach((t) => t.stop()); stream = null; video.srcObject = null;
    closeFile();
    fileUrl = URL.createObjectURL(file);
    video.muted = true; video.playsInline = true; video.loop = true;
    video.src = fileUrl;
    mirror = false; $('camMirror').checked = false; applyVideoClasses();
    await video.play();
  }
  async function openStream() {
    closeFile();
    stream?.getTracks().forEach((t) => t.stop());
    if (!navigator.mediaDevices?.getUserMedia) throw new Error('この環境ではカメラを使えません（HTTPS または localhost で開いてください）');
    stream = await navigator.mediaDevices.getUserMedia({ audio: false, video: { facingMode: facing, width: { ideal: 1280 }, height: { ideal: 720 } } });
    video.srcObject = stream;
    await video.play();
  }

  async function start(file) {
    if (starting) return; starting = true;
    stopDemo();
    try {
      if (file) { status('動画を読み込み中…'); await openFile(file); }
      else { status('カメラを起動中…'); await openStream(); }
      video.style.display = 'block';
      if (!tracker) {
        status('姿勢推定モデルを読み込み中…（初回は数秒〜数十秒かかります）');
        const { createTracker } = await import('../tracking/tracker.js');
        tracker = await createTracker({
          video, onResult,
          options: { modelVariant: $('camModel').value, hands: $('camHands').checked, handsEvery: isMobile() ? 2 : 1, onStatus: (m) => status(m) },
        });
        const c = loadCalibration(); if (c) tracker.setCalibration(c);
      }
      tracker.start();
      status(file ? '動画を解析中（ループ再生）' : '追跡中');
    } catch (e) {
      console.warn(e);
      if (file) {
        status(`動画を再生できませんでした：${e.message || e}`);
        alertBox(`動画を再生できませんでした。<br><small>${e.message || e}<br>・MP4（H.264）形式の動画をお試しください</small>`);
      } else {
        status(`カメラを開始できませんでした：${e.message || e}`);
        alertBox(`カメラを開始できませんでした。<br><small>${e.message || e}<br>・ブラウザのカメラ許可を確認してください<br>・スマホでは HTTPS（または PC の localhost）が必要です<br>・使えない場合は入力を「動画ファイル」にして、撮影した動画を読み込んでください</small>`);
      }
    } finally { starting = false; }
  }
  function stop() {
    stopDemo();
    tracker?.stop();
    stream?.getTracks().forEach((t) => t.stop()); stream = null;
    video.srcObject = null; closeFile(); video.style.display = 'none'; last = null;
    status('停止中');
  }

  async function calibrate() {
    const src = demo ? demo.pipeline : (tracker && live() ? tracker : null);
    if (!src) { alertBox('先に「開始」を押してください'); return; }
    const cd = $('countdown');
    try {
      for (let i = 3; i > 0; i--) {
        cd.innerHTML = `${i}<small>カメラに正対し、腕を体から少し離して静止してください</small>`; cd.hidden = false;
        await new Promise((r) => setTimeout(r, 1000));
      }
      cd.innerHTML = '計測中…<small>そのまま静止</small>';
      const c = await src.calibrate({ durationMs: 2500 });
      localStorage.setItem(CAL_KEY, JSON.stringify(c));
      showCalibInfo(c);
      cd.innerHTML = '完了<small>骨の長さを体格に合わせました</small>';
      await new Promise((r) => setTimeout(r, 900));
    } catch (e) {
      cd.innerHTML = `失敗<small>${e.message || e}</small>`;
      await new Promise((r) => setTimeout(r, 1500));
    } finally { cd.hidden = true; }
  }
  function alertBox(html) {
    const cd = $('countdown'); cd.innerHTML = `<span style="font-size:1rem">${html}</span>`; cd.hidden = false;
    setTimeout(() => { cd.hidden = true; }, 4000);
  }

  // ---------- demo source: synthetic MediaPipe-format frames (src/tracking/synth.js) ----------
  async function startDemo() {
    if (starting) return;
    tracker?.stop(); stream?.getTracks().forEach((t) => t.stop()); stream = null; video.srcObject = null; closeFile(); video.style.display = 'none';
    const [{ createPipeline }, { synthesizeFrame, makeRng }] = await Promise.all([import('../tracking/pipeline.js'), import('../tracking/synth.js')]);
    const c = loadCalibration();
    const pipeline = createPipeline({}); if (c) pipeline.setCalibration(c);
    demo = { pipeline, synthesizeFrame, rng: makeRng(5), t0: performance.now(), lastT: 0, fps: 0 };
    demoCanvas.style.display = 'block';
    status('デモ（合成データ）を再生中');
  }
  function stopDemo() { if (!demo) return; demo = null; demoCanvas.style.display = 'none'; last = null; }
  /** demo motion: right shoulder abduction, left elbow + pronation, knee bend, grip */
  function demoPose(t) {
    const w = (p, ph = 0) => 0.5 - 0.5 * Math.cos(2 * Math.PI * (t / p) + ph);
    const limb = (side) => ({
      shoulder: { elevation: side === 'right' ? 10 + 110 * w(6) : 20, plane: side === 'right' ? 0 : 30, axialRotation: 0 },
      elbow: { flexion: side === 'left' ? 20 + 100 * w(5) : 10 }, forearm: { pronation: side === 'left' ? -80 + 160 * w(4, 1) : -60 },
      wrist: { flexion: 0, ulnarDeviation: 0 },
      hip: { flexion: side === 'left' ? 40 * w(7) : 0, abduction: 5, axialRotation: 0 }, knee: { flexion: side === 'left' ? 70 * w(7) : 0 },
      ankle: { dorsiflexion: 0, inversion: 0 },
      fingers: { thumb: { cmcFlexion: 10, cmcAbduction: 20, mcp: 20 * w(3), ip: 30 * w(3) },
        index: { mcp: 70 * w(3), pip: 80 * w(3), dip: 50 * w(3) }, middle: { mcp: 70 * w(3), pip: 80 * w(3), dip: 50 * w(3) },
        ring: { mcp: 70 * w(3), pip: 80 * w(3), dip: 50 * w(3) }, little: { mcp: 70 * w(3), pip: 80 * w(3), dip: 50 * w(3) } },
    });
    return { spine: { flexion: 0, lateralBend: 0, axialRotation: 15 * Math.sin(t / 3) }, neck: { flexion: 0, lateralBend: 0, axialRotation: 20 * Math.sin(t / 2.5) }, left: limb('left'), right: limb('right') };
  }
  function stepDemo() {
    const now = performance.now(); const t = (now - demo.t0) / 1000;
    const fr = demo.synthesizeFrame(demoPose(t), { width: 640, height: 480, noise: 0.003, pixelNoise: 0.8, rng: demo.rng, center: [0.5, 0.52], camDistance: 3.0 });
    const res = demo.pipeline.process({ t: now, width: 640, height: 480, landmarks: fr.landmarks, worldLandmarks: fr.worldLandmarks, hands: fr.hands });
    const dt = demo.lastT ? (now - demo.lastT) / 1000 : 0; demo.lastT = now;
    if (dt > 0) demo.fps = demo.fps ? demo.fps + (1 / dt - demo.fps) * 0.1 : 1 / dt;
    res.fps = demo.fps; res.calibrating = demo.pipeline.calibrating;
    onResult(res);
    drawDemo2D(fr);
  }
  const BONES2D = [[11, 12], [11, 13], [13, 15], [12, 14], [14, 16], [11, 23], [12, 24], [23, 24], [23, 25], [25, 27], [24, 26], [26, 28], [27, 31], [28, 32], [27, 29], [28, 30], [0, 2], [0, 5]];
  function drawDemo2D(fr) {
    const g = demoCanvas.getContext('2d'); const W = 640, H = 480;
    g.fillStyle = '#20262e'; g.fillRect(0, 0, W, H);
    g.strokeStyle = '#2c3540'; g.lineWidth = 1;
    for (let x = 0; x < W; x += 40) { g.beginPath(); g.moveTo(x, 0); g.lineTo(x, H); g.stroke(); }
    for (let y = 0; y < H; y += 40) { g.beginPath(); g.moveTo(0, y); g.lineTo(W, y); g.stroke(); }
    const P = fr.landmarks.map((l) => [l.x * W, l.y * H]);
    g.strokeStyle = 'rgba(255,170,60,.85)'; g.lineWidth = 3;
    for (const [a, b] of BONES2D) { g.beginPath(); g.moveTo(...P[a]); g.lineTo(...P[b]); g.stroke(); }
    g.fillStyle = '#ffb347'; for (const p of P) { g.beginPath(); g.arc(p[0], p[1], 3.5, 0, 7); g.fill(); }
    g.fillStyle = '#ff7ab0';
    for (const h of fr.hands || []) for (const l of h.landmarks) { g.beginPath(); g.arc(l.x * W, l.y * H, 2, 0, 7); g.fill(); }
    g.save(); if (mirror) { g.translate(W, 0); g.scale(-1, 1); } // keep the caption readable
    g.fillStyle = '#c8d4e0'; g.font = '14px sans-serif'; g.fillText('デモ：合成ランドマーク（橙）＋推定した骨', 10, 22); g.restore();
  }

  // ---------- results ----------
  function onResult(res) {
    last = res;
    $('camFps').textContent = `${(res.fps ?? 0).toFixed(0)} fps`;
    if (!res.detected) status('人物が見つかりません（全身または上半身が映るように）');
    else status(`${res.calibrated ? '' : '未キャリブレーション・'}信頼度 ${(res.confidence * 100).toFixed(0)}%`);
    if (res.dims) { // bone lengths may be auto-learned each frame → rescale at most every 0.5 s
      const k = Object.values(res.dims).map((v) => v.toFixed(3)).join(',');
      const now = performance.now();
      if (k !== dimsKey && (now - lastScaleT > 500 || !dimsKey)) { dimsKey = k; lastScaleT = now; rig.scaleToDims(res.dims); }
    }
    if (res.pose) rig.setPose(res.pose, { useRoot: true });
    rig.setConfidence($('camFade').checked ? res.pose?.confidence : null, 0.4);
  }

  // ---------- layout & rendering ----------
  function layout() {
    const Cw = viewport.clientWidth, Ch = viewport.clientHeight;
    let area = { x: 0, y: 0, w: Cw, h: Ch }; let a3 = null;
    if (side3D) {
      if (Cw >= Ch) { area = { x: 0, y: 0, w: Cw / 2, h: Ch }; a3 = { x: Cw / 2, y: 0, w: Cw / 2, h: Ch }; }
      else { area = { x: 0, y: 0, w: Cw, h: Ch / 2 }; a3 = { x: 0, y: Ch / 2, w: Cw, h: Ch / 2 }; }
    }
    const W = demo ? 640 : (video.videoWidth || 640), H = demo ? 480 : (video.videoHeight || 480);
    const k = Math.min(area.w / W, area.h / H);
    const vr = { x: area.x + (area.w - W * k) / 2, y: area.y + (area.h - H * k) / 2, w: W * k, h: H * k, W, H };
    const box = { left: `${vr.x}px`, top: `${vr.y}px`, width: `${vr.w}px`, height: `${vr.h}px` };
    Object.assign(video.style, box); Object.assign(demoCanvas.style, box);
    rects.video = vr; rects.view3D = a3;
    return { Cw, Ch };
  }
  const flipM = new THREE.Matrix4();
  function render() {
    if (demo) stepDemo();
    const { Cw, Ch } = layout();
    renderer.setScissorTest(false);
    renderer.setViewport(0, 0, Cw, Ch);
    renderer.clear();
    const ac = renderer.autoClear; renderer.autoClear = false;
    renderer.setScissorTest(true);
    const root = rig.root;
    // --- overlay on the video ---
    const vr = rects.video; const al = last?.alignment;
    if ((live() || demo) && al?.valid && last?.detected !== false) {
      ortho.left = 0; ortho.right = al.width; ortho.top = 0; ortho.bottom = -al.height; ortho.updateProjectionMatrix();
      root.matrixAutoUpdate = false;
      root.matrix.fromArray(al.matrix);
      if (mirror) { flipM.set(-1, 0, 0, al.width, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1); root.matrix.premultiply(flipM); }
      root.matrixWorldNeedsUpdate = true; root.updateMatrixWorld(true);
      setRect(vr, Ch);
      renderer.render(scene, ortho);
    }
    // --- side-by-side 3D view ---
    if (rects.view3D) {
      const r = rects.view3D;
      root.matrixAutoUpdate = true; root.position.copy(pelvisCenter); root.quaternion.identity(); root.scale.set(1, 1, 1);
      root.updateMatrixWorld(true);
      simCamera.aspect = r.w / r.h; simCamera.updateProjectionMatrix();
      setRect(r, Ch);
      renderer.render(scene, simCamera);
    }
    renderer.setScissorTest(false);
    renderer.autoClear = ac;
  }
  function setRect(r, Ch) {
    const y = Ch - r.y - r.h;
    renderer.setViewport(r.x, y, r.w, r.h); renderer.setScissor(r.x, y, r.w, r.h);
  }

  return {
    enter() {
      active = true; $('camHud').hidden = false; ui.setTeach(null);
      rig.setUseRoot(true);
      if (last?.dims) { dimsKey = ''; } else rig.scaleToDims(tracker?.pipeline?.dims || loadCalibration()?.dims || defaultDims());
      if (!live() && !demo) status('「開始」を押してください（カメラがなければ入力を「デモ」に）');
    },
    exit() {
      active = false; stop(); $('camHud').hidden = true; rig.setConfidence(null);
      rig.root.matrixAutoUpdate = true; rig.setUseRoot(false); dimsKey = '';
      simCamera.aspect = viewport.clientWidth / viewport.clientHeight; simCamera.updateProjectionMatrix();
    },
    render,
    /** camera + screen rect for tap-picking in camera mode (3D side view only) */
    pickCamera(e) {
      const r3 = rects.view3D; if (!r3) return null;
      const vb = viewport.getBoundingClientRect();
      const x = e.clientX - vb.left, y = e.clientY - vb.top;
      if (x < r3.x || x > r3.x + r3.w || y < r3.y || y > r3.y + r3.h) return null;
      return { camera: simCamera, rect: { left: vb.left + r3.x, top: vb.top + r3.y, width: r3.w, height: r3.h } };
    },
    get active() { return active; },
    get last() { return last; },
    startDemo, start, stop,
  };
}

function defaultDims() { return { ...DEFAULT_DIMS }; } // what the tracker uses before calibration
