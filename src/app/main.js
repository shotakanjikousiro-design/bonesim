// BoneSim アプリ本体：描画・モード切替・表示オプション・骨のタップ選択
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { createRig } from '../rig/anatomicalRig.js';
import { createSimulator } from './simulator.js';
import { createCameraMode } from './cameraMode.js';
import { VIEWS, jointNameJa } from './labels.js';

const $ = (id) => document.getElementById(id);
const viewport = $('viewport'), canvas = $('gl');

// ---------------- renderer / scene ----------------
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true, powerPreference: 'high-performance' });
renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
renderer.setClearColor(0x000000, 0);
const scene = new THREE.Scene();
scene.add(new THREE.HemisphereLight(0xdfe9ff, 0x2a2016, 1.3));
const sun = new THREE.DirectionalLight(0xffffff, 1.6); sun.position.set(1.5, 3, 2.5); scene.add(sun);
const rim = new THREE.DirectionalLight(0x9fc4ff, 0.8); rim.position.set(-2, 1.5, -2.5); scene.add(rim);
const floor = new THREE.Mesh(new THREE.RingGeometry(0.02, 0.55, 64), new THREE.MeshBasicMaterial({ color: 0x5cc8ff, transparent: true, opacity: 0.08, side: THREE.DoubleSide, depthWrite: false }));
floor.rotation.x = -Math.PI / 2; floor.position.set(0, 0, 0.06); scene.add(floor);

const camera = new THREE.PerspectiveCamera(35, 1, 0.02, 50);
const controls = new OrbitControls(camera, canvas);
controls.enableDamping = true; controls.dampingFactor = 0.12; controls.minDistance = 0.15; controls.maxDistance = 8;
controls.target.set(...VIEWS.front.target); camera.position.set(...VIEWS.front.pos);

let viewTween = null;
function setView(id, instant = false) {
  const v = VIEWS[id]; if (!v) return;
  const from = { p: camera.position.clone(), t: controls.target.clone() };
  const to = { p: new THREE.Vector3(...v.pos), t: new THREE.Vector3(...v.target) };
  // narrow / portrait screens: step back so the same width stays in view
  const k = Math.max(1, 1 / Math.max(0.3, camera.aspect));
  to.p.sub(to.t).multiplyScalar(k).add(to.t);
  if (instant) { camera.position.copy(to.p); controls.target.copy(to.t); controls.update(); return; }
  viewTween = { from, to, t: 0 };
}
function stepTween(dt) {
  if (!viewTween) return;
  viewTween.t = Math.min(1, viewTween.t + dt / 0.7);
  const u = viewTween.t * viewTween.t * (3 - 2 * viewTween.t);
  camera.position.lerpVectors(viewTween.from.p, viewTween.to.p, u);
  controls.target.lerpVectors(viewTween.from.t, viewTween.to.t, u);
  if (viewTween.t >= 1) viewTween = null;
}

function resize() {
  const w = viewport.clientWidth, h = viewport.clientHeight; if (!w || !h) return;
  renderer.setSize(w, h, false);
  camera.aspect = w / h; camera.updateProjectionMatrix();
}
new ResizeObserver(resize).observe(viewport);
resize();

// ---------------- UI helpers ----------------
const teach = $('teach');
const ui = {
  setTeach(html) { if (html) { teach.innerHTML = html; teach.hidden = false; } else teach.hidden = true; },
  setView, $,
};

// ---------------- rig ----------------
let rig;
try {
  rig = await createRig({ scene, glbUrl: './assets/skeleton.glb', rigUrl: './assets/rig.json' });
} catch (e) {
  $('loading').textContent = `骨格モデルの読み込みに失敗しました：${e.message}（start.bat などでローカルサーバーから開いてください）`;
  throw e;
}
$('loading').remove();
const pelvisCenter = new THREE.Vector3(...rig.rigData.pelvisCenter);

// ---------------- display options ----------------
let styleKey = 'xray';
function applyStyle() {
  if (styleKey === 'skin') { rig.setStyle('bone'); rig.setSkin(0.6); }
  else { rig.setStyle(styleKey); rig.setSkin(0); }
  document.querySelectorAll('#styleSeg button').forEach((b) => b.classList.toggle('active', b.dataset.style === styleKey));
}
document.querySelectorAll('#styleSeg button').forEach((b) => b.addEventListener('click', () => { styleKey = b.dataset.style; applyStyle(); }));
applyStyle();
// film-like inversion: render on an opaque black background, then CSS-invert the canvas
$('invert').addEventListener('change', (e) => { viewport.classList.toggle('invert', e.target.checked); renderer.setClearColor(0x05080c, e.target.checked ? 1 : 0); });
$('showAxes').addEventListener('change', (e) => rig.showAxes(e.target.checked));
$('showLandmarks').addEventListener('change', (e) => rig.showLandmarks(e.target.checked));
$('autoER').addEventListener('change', (e) => rig.setAutoHumeralRotation(e.target.checked));
const vb = $('viewButtons');
for (const [id, v] of Object.entries(VIEWS)) {
  if (v.hidden) continue;
  const b = document.createElement('button'); b.textContent = v.label; b.addEventListener('click', () => setView(id)); vb.appendChild(b);
}

// panel collapse (mobile)
$('panelToggle').addEventListener('click', () => {
  const p = $('panel'); p.classList.toggle('collapsed');
  $('panelToggle').textContent = p.classList.contains('collapsed') ? '▴ 操作パネルを開く' : '▾ 操作パネル';
});

// ---------------- picking ----------------
const raycaster = new THREE.Raycaster(); const ndc = new THREE.Vector2();
let downXY = null;
canvas.addEventListener('pointerdown', (e) => { downXY = [e.clientX, e.clientY]; });
canvas.addEventListener('pointerup', (e) => {
  if (!downXY || Math.hypot(e.clientX - downXY[0], e.clientY - downXY[1]) > 6) return;
  const pc = mode === 'sim' ? { camera, rect: canvas.getBoundingClientRect() } : cam.pickCamera(e);
  if (!pc) return;
  const r = pc.rect;
  ndc.set(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1);
  raycaster.setFromCamera(ndc, pc.camera);
  const hit = raycaster.intersectObjects(rig.pickables(), false)[0];
  showPick(hit ? hit.object.userData.nodeName : null);
});
function showPick(nodeName) {
  const info = rig.highlight(nodeName); const box = $('pickInfo');
  if (!info) { box.hidden = true; return; }
  box.innerHTML = `<button class="close" aria-label="閉じる">×</button><b>${info.nameJa || info.nodeName}</b><br>
    <span class="hint">${info.nameEn || ''}</span><br>この骨を動かす関節：${jointNameJa(info.joint)}`;
  box.hidden = false;
  box.querySelector('.close').addEventListener('click', () => showPick(null));
}

// ---------------- modes ----------------
const sim = createSimulator({ rig, ui });
const cam = createCameraMode({ renderer, scene, rig, viewport, video: $('video'), simCamera: camera, controls, ui, pelvisCenter });
let mode = 'sim';
function setMode(m) {
  if (m === mode) return;
  mode = m;
  $('tabSim').classList.toggle('active', m === 'sim'); $('tabCam').classList.toggle('active', m === 'cam');
  $('tabSim').setAttribute('aria-selected', m === 'sim'); $('tabCam').setAttribute('aria-selected', m === 'cam');
  $('simPanel').hidden = m !== 'sim'; $('camPanel').hidden = m !== 'cam';
  floor.visible = m === 'sim';
  if (m === 'cam') { sim.exit(); cam.enter(); } else { cam.exit(); sim.enter(); resize(); }
}
$('tabSim').addEventListener('click', () => setMode('sim'));
$('tabCam').addEventListener('click', () => setMode('cam'));
sim.enter();

// ---------------- loop ----------------
let last = performance.now();
function frame(now) {
  const dt = Math.min(0.1, (now - last) / 1000); last = now;
  if (mode === 'sim') {
    stepTween(dt); controls.update(); sim.update(dt);
    rig.root.matrixAutoUpdate = true; rig.root.position.copy(pelvisCenter); rig.root.quaternion.identity();
    renderer.setScissorTest(false);
    renderer.setViewport(0, 0, viewport.clientWidth, viewport.clientHeight);
    renderer.render(scene, camera);
  } else {
    controls.update();
    cam.render(dt);
  }
  requestAnimationFrame(frame);
}
requestAnimationFrame(frame);

// test / debugging hook (used by the headless smoke test)
window.bonesim = { rig, sim, cam, setView, setMode, setStyle: (s) => { styleKey = s; applyStyle(); }, camera, controls, showPick };
