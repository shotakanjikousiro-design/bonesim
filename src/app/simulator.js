// シミュレーターモード：スライダー・プリセット・連動運動の切替
import { anatomicalPose } from '../shared/conventions.js';
import { completePose } from '../rig/anatomicalRig.js';
import { fistFingers } from '../rig/coupling.js';
import { SLIDER_GROUPS } from './labels.js';
import { PRESETS } from './presets.js';

const getPath = (o, path) => path.split('.').reduce((a, k) => (a == null ? undefined : a[k]), o);
const setPath = (o, path, v) => { const ks = path.split('.'); const last = ks.pop(); const t = ks.reduce((a, k) => (a[k] ??= {}), o); t[last] = v; };

export function createSimulator({ rig, ui }) {
  const $ = ui.$;
  let pose = completePose(anatomicalPose());
  let grip = { left: 0, right: 0 };
  let preset = null, playing = false, t = 0, speed = 1, teachTimer = 0;
  const inputs = [];

  // ---------- sliders ----------
  const host = $('sliders');
  SLIDER_GROUPS.forEach((g, gi) => {
    const det = document.createElement('details'); if (gi === 1) det.open = true;
    det.innerHTML = `<summary>${g.title}</summary>`;
    for (const [path, label, min, max] of g.items) {
      const id = `sl_${path.replace(/\./g, '_')}`;
      const wrap = document.createElement('div'); wrap.className = 'slider';
      const isGrip = path.endsWith('.grip');
      wrap.innerHTML = `<label for="${id}">${label}</label><output></output><input id="${id}" type="range" min="${min}" max="${max}" step="1">`;
      const input = wrap.querySelector('input'), out = wrap.querySelector('output');
      const rec = { path, input, out, isGrip, last: null };
      input.addEventListener('input', () => {
        stopPreset(false);
        const v = Number(input.value);
        if (isGrip) { const side = path.split('.')[0]; grip[side] = v; pose[side].fingers = fistFingers(v / 100); }
        else setPath(pose, path, v);
        apply();
      });
      inputs.push(rec);
      det.appendChild(wrap);
    }
    host.appendChild(det);
  });
  function syncSliders() {
    for (const r of inputs) {
      const v = r.isGrip ? grip[r.path.split('.')[0]] : getPath(pose, r.path);
      const rv = Math.round(v ?? 0);
      if (rv !== r.last) { r.input.value = rv; r.out.textContent = r.isGrip ? `${rv}%` : `${rv}°`; r.last = rv; }
    }
  }

  // ---------- presets ----------
  const pbHost = $('presetButtons'); const pbs = {};
  for (const p of PRESETS) {
    const b = document.createElement('button'); b.textContent = p.label; pbs[p.id] = b;
    b.addEventListener('click', () => startPreset(p.id));
    pbHost.appendChild(b);
  }
  function startPreset(id) {
    preset = PRESETS.find((p) => p.id === id); t = 0; playing = true;
    Object.values(pbs).forEach((b) => b.classList.remove('active')); pbs[id].classList.add('active');
    ui.setView(preset.view);
    updatePlayBtn(); teachTimer = 0;
  }
  function stopPreset(clearTeach = true) {
    if (!preset) return;
    playing = false; updatePlayBtn();
    if (clearTeach) { preset = null; Object.values(pbs).forEach((b) => b.classList.remove('active')); ui.setTeach(null); }
  }
  const playBtn = $('playPause');
  function updatePlayBtn() { playBtn.textContent = playing ? '⏸ 一時停止' : '▶ 再生'; playBtn.disabled = !preset; }
  playBtn.addEventListener('click', () => { if (!preset) return; playing = !playing; updatePlayBtn(); });
  $('speed').addEventListener('input', (e) => { speed = Number(e.target.value); $('speedOut').textContent = `${speed.toFixed(2).replace(/0$/, '')}×`; });

  // ---------- coupling ----------
  const coup = $('coupling');
  coup.addEventListener('change', () => {
    rig.setCoupling(coup.checked);
    coup.nextElementSibling.textContent = coup.checked ? '連動運動 ON' : '連動運動 OFF（単純モデル）';
    teachTimer = 0; apply();
  });

  // ---------- resets ----------
  function reset(kind) {
    stopPreset();
    pose = completePose(anatomicalPose()); grip = { left: 0, right: 0 };
    if (kind === 'neutral') { pose.left.forearm.pronation = 0; pose.right.forearm.pronation = 0; }
    apply();
  }
  $('resetAnat').addEventListener('click', () => reset('anat'));
  $('resetNeutral').addEventListener('click', () => reset('neutral'));

  function apply() {
    rig.setPose(pose);
    syncSliders();
  }

  function updateTeach(force) {
    if (!preset) return;
    if (!force && teachTimer > 0) return;
    teachTimer = 0.12;
    ui.setTeach(preset.explain(pose, rig.getDerived(), coup.checked));
  }

  apply(); updatePlayBtn();
  return {
    enter() { rig.setUseRoot(false); rig.scaleToDims(rig.rigData.dims); apply(); if (preset) updateTeach(true); },
    exit() { playing = false; updatePlayBtn(); ui.setTeach(null); },
    update(dt) {
      teachTimer -= dt;
      if (preset && playing) {
        t += dt * speed;
        pose = completePose(preset.pose(t));
        grip = { left: 0, right: 0 };
        if (pose.right.fingers) grip.right = Math.round((pose.right.fingers.middle?.pip || 0)); // ≈ %
        if (pose.left.fingers) grip.left = Math.round((pose.left.fingers.middle?.pip || 0));
        apply();
      }
      updateTeach(false);
    },
    startPreset, stopPreset, reset,
    setPose(p) { stopPreset(); pose = completePose(p); apply(); },
    get pose() { return pose; },
  };
}
