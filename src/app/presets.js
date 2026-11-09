// プリセット動作（ループ）と、その解剖学的な説明文
import { anatomicalPose } from '../shared/conventions.js';
import { fistFingers } from '../rig/coupling.js';

/** smooth 0→1→0 over one period */
const wave = (t, period) => 0.5 - 0.5 * Math.cos((2 * Math.PI * t) / period);
const lerp = (a, b, u) => a + (b - a) * u;
const r1 = (x) => Math.round(x);
const smoothstep = (a, b, x) => { const t = Math.min(1, Math.max(0, (x - a) / (b - a))); return t * t * (3 - 2 * t); };

export const PRESETS = [
  {
    id: 'elbow', label: '肘の屈曲・伸展', period: 5, view: 'armSide',
    pose(t) { const p = anatomicalPose(); p.right.elbow.flexion = lerp(0, 140, wave(t, this.period)); return p; },
    explain: (p) => `<b>肘の屈曲 ${r1(p.right.elbow.flexion)}°</b><br>
      尺骨は上腕骨滑車のまわりを回ります（腕尺関節）。伸展位では前腕が上腕に対してやや外側を向き（肘角・運搬角：本モデルで約10°）、
      屈伸軸が斜め（内側端がやや下）なので、屈曲するにつれて前腕は上腕骨の上に重なっていきます。橈骨は尺骨と一緒に動き、橈骨頭は上腕骨小頭の上を転がります。`,
  },
  {
    id: 'pronation', label: '前腕の回内・回外', period: 6, view: 'arm',
    pose(t) { const p = anatomicalPose(); p.right.elbow.flexion = 10; p.right.forearm.pronation = lerp(-90, 90, wave(t, this.period)); return p; },
    explain: (p, d, coupling) => `<b>前腕 ${p.right.forearm.pronation >= 0 ? '回内' : '回外'} ${r1(Math.abs(p.right.forearm.pronation))}°</b>（0°＝中間位）<br>
      回内では<b>橈骨が尺骨のまわりを回り</b>（橈骨頭中心と尺骨頭中心を結ぶ軸）、尺骨はほとんど動きません。手根骨と手は橈骨についていくので、
      回内位では橈骨と尺骨が交差します。${coupling ? '' : '<br><span class="warn">連動運動OFF：前腕全体を1本の棒としてねじる簡易モデルです（尺骨も回ってしまう）。</span>'}`,
  },
  {
    id: 'abduction', label: '肩の外転（肩甲上腕リズム）', period: 8, view: 'back',
    pose(t) {
      const p = anatomicalPose(); const el = lerp(0, 170, wave(t, this.period));
      // full abduction needs humeral external rotation (greater tuberosity clears the acromion)
      p.right.shoulder = { elevation: el, plane: 0, axialRotation: -35 * smoothstep(80, 160, el) }; return p;
    },
    explain: (p, d, coupling) => shoulderText('外転', p, d, coupling) + (p.right.shoulder.elevation > 90 ? '<br>※ 高挙上では上腕骨が外旋します（大結節が肩峰に当たらないように）。' : ''),
  },
  {
    id: 'flexion', label: '肩の屈曲', period: 8, view: 'side',
    pose(t) { const p = anatomicalPose(); p.right.shoulder = { elevation: lerp(0, 170, wave(t, this.period)), plane: 90, axialRotation: 0 }; return p; },
    explain: (p, d, coupling) => shoulderText('屈曲', p, d, coupling),
  },
  {
    id: 'knee', label: '膝の屈伸（スクリューホーム）', period: 7, view: 'knee',
    pose(t) { const p = anatomicalPose(); p.right.knee.flexion = lerp(0, 120, wave(t, this.period)); return p; },
    explain: (p, d, coupling) => {
      const r = d?.right || {};
      return `<b>膝の屈曲 ${r1(p.right.knee.flexion)}°</b>　脛骨の回旋 ${r1(Math.abs(r.tibialRotation || 0))}°${(r.tibialRotation || 0) < -0.5 ? '（外旋）' : ''}
      ${r.patellaRotation !== undefined ? `／膝蓋骨の傾き ${r1(r.patellaRotation)}°` : ''}<br>
      伸展の最後の約20°で脛骨が最大約10°外旋して膝が「締まり」ます（スクリューホーム機構）。屈曲を始めると逆に内旋してロックが外れます。
      膝蓋骨は大腿骨の滑車溝を滑り降りながら、膝の屈曲角の約0.7倍だけ傾きます。膝蓋腱は伸展位ではややたるみ、張ってからは長さを保ちます。${coupling ? '' : '<br><span class="warn">連動運動OFF：回旋なしの単純な蝶番、膝蓋骨は大腿骨に固定。</span>'}`;
    },
  },
  {
    id: 'wrist', label: '手関節の掌屈・背屈', period: 6, view: 'armSide',
    pose(t) { const p = anatomicalPose(); p.right.elbow.flexion = 90; p.right.forearm.pronation = 0; p.right.wrist.flexion = lerp(-70, 80, wave(t, this.period)); return p; },
    explain: (p, d, coupling) => `<b>手関節 ${p.right.wrist.flexion >= 0 ? '掌屈' : '背屈'} ${r1(Math.abs(p.right.wrist.flexion))}°</b><br>
      手関節の運動は<b>橈骨手根関節</b>（橈骨と近位手根列：舟状骨・月状骨・三角骨）と<b>手根中央関節</b>で分担します。
      <b>掌屈</b>では手根中央関節が主（橈骨手根：手根中央 ≈ 4：6）、<b>背屈</b>では橈骨手根関節が主（≈ 2：1、Sarrafian 1977）。
      近位手根列も動くので、月状骨の傾きにも注目してください。${coupling ? '' : '<br><span class="warn">連動運動OFF：すべて橈骨手根関節で動かしています。</span>'}`,
  },
  {
    id: 'trunk', label: '体幹の回旋', period: 7, view: 'top',
    pose(t) { const p = anatomicalPose(); const u = wave(t, this.period); p.spine.axialRotation = lerp(-35, 35, u); p.neck.axialRotation = lerp(-30, 30, u); return p; },
    explain: (p, d, coupling) => `<b>体幹の回旋 ${r1(Math.abs(p.spine.axialRotation))}°（${p.spine.axialRotation >= 0 ? '左' : '右'}）・頸部 ${r1(Math.abs(p.neck.axialRotation))}°</b><br>
      腰椎は椎間関節の向きのため1分節あたり1〜2°程度しか回旋せず、体幹の回旋の多くは胸椎で起こります（本モデルでは胸郭を1つの剛体とし、胸腰移行部に集約）。
      頸部の回旋の約半分は環軸関節（C1/C2）で起こります。${coupling ? '' : '<br><span class="warn">連動運動OFF：腰仙関節・C7/T1の1か所だけで回しています。</span>'}`,
  },
  {
    id: 'fist', label: 'グー・パー', period: 4, view: 'arm',
    pose(t) {
      const p = anatomicalPose(); const u = wave(t, this.period);
      p.right.elbow.flexion = 30; p.right.forearm.pronation = -60; p.right.fingers = fistFingers(u);
      p.left.elbow.flexion = 30; p.left.forearm.pronation = -60; p.left.fingers = fistFingers(u);
      return p;
    },
    explain: () => `<b>グー・パー</b><br>
      指の屈曲はMP関節・PIP関節・DIP関節で起こります。DIP関節はPIP関節と連動しやすく（約2/3）、各指を曲げると指先は舟状骨結節（手掌の橈側・近位）の方向を向きます。
      母指はCM関節（鞍関節）で対立し、ほかの指を包み込みます。`,
  },
];

function shoulderText(kind, p, d, coupling) {
  const r = d?.right || {};
  const el = p.right.shoulder.elevation;
  const gh = r.glenohumeralElevation ?? el, st = r.scapularUpwardRotation ?? 0;
  return `<b>肩の${kind} ${r1(el)}°</b> ＝ 肩甲上腕関節 ≈ ${r1(gh)}° ＋ 肩甲骨の上方回旋 ≈ ${r1(st)}°
    ${coupling ? `／鎖骨挙上 ≈ ${r1(r.clavicleElevation || 0)}°・後方回旋 ≈ ${r1(r.claviclePosteriorRotation || 0)}°` : ''}<br>
    最初の約30°は主に肩甲上腕関節が動き（セッティング相）、それ以降は肩甲骨の上方回旋が加わって、挙上全体ではおよそ<b>肩甲上腕関節2：肩甲骨1</b>になります（Inman）。
    肩甲骨は上方回旋しながら後傾し、鎖骨は胸鎖関節で挙上、挙上後半では長軸まわりに後方回旋します。
    ${coupling ? '' : '<br><span class="warn">連動運動OFF：肩甲骨・鎖骨は固定し、上腕骨だけを動かしています。</span>'}`;
}
