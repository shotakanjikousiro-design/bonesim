// 日本語ラベル・スライダー定義・視点プリセット
import { ROM } from '../shared/conventions.js';

export const JOINT_JA = {
  sternoclavicular: '胸鎖関節', acromioclavicular: '肩鎖関節', glenohumeral: '肩甲上腕関節',
  humeroulnar: '腕尺関節（肘関節）', radioulnar: '橈尺関節（前腕の回内・回外）',
  radiocarpal: '橈骨手根関節', midcarpal: '手根中央関節',
  cmc1: '母指CM関節', mcp1: '母指MP関節', ip1: '母指IP関節',
  hip: '股関節', knee: '膝関節（脛骨大腿関節）', patellofemoral: '膝蓋大腿関節',
  talocrural: '距腿関節（足関節）', subtalar: '距骨下関節',
  lumbosacral: '腰仙関節（L5/S1）', temporomandibular: '顎関節', atlantooccipital: '環椎後頭関節', C1C2: '環軸関節（C1/C2）',
  T12L1: 'T12/L1 椎間（胸腰移行部）', C7T1: 'C7/T1 椎間',
};
const FINGER_JA = { 2: '示指', 3: '中指', 4: '環指', 5: '小指' };

export function jointNameJa(jointId) {
  if (!jointId) return '（体幹の基準：骨盤）';
  const m = jointId.match(/^(.*?)(?:_([LR]))?$/); const base = m[1]; const side = m[2] ? (m[2] === 'L' ? '左' : '右') : '';
  let name = JOINT_JA[base];
  if (!name) {
    let k;
    if ((k = base.match(/^mcp([2-5])$/))) name = `${FINGER_JA[k[1]]}MP関節`;
    else if ((k = base.match(/^pip([2-5])$/))) name = `${FINGER_JA[k[1]]}PIP関節`;
    else if ((k = base.match(/^dip([2-5])$/))) name = `${FINGER_JA[k[1]]}DIP関節`;
    else if ((k = base.match(/^([LC]\d)([LC]\d)$/))) name = `${k[2]}/${k[1]} 椎間`;
    else name = base;
  }
  return side ? `${name}（${side}）` : name;
}

/** slider definitions per group: [path, label, min, max] — paths into Pose */
const limb = (side) => {
  const s = side;
  return [
    [`${s}.shoulder.elevation`, '肩 挙上', ...ROM.shoulder.elevation],
    [`${s}.shoulder.plane`, '肩 挙上面（0=外転, 90=屈曲, −=伸展）', ...ROM.shoulder.plane],
    [`${s}.shoulder.axialRotation`, '肩 回旋（+内旋 / −外旋）', ...ROM.shoulder.axialRotation],
    [`${s}.elbow.flexion`, '肘 屈曲（−過伸展）', ...ROM.elbow.flexion],
    [`${s}.forearm.pronation`, '前腕 回内（+）/ 回外（−）', ...ROM.forearm.pronation],
    [`${s}.wrist.flexion`, '手関節 掌屈（+）/ 背屈（−）', ...ROM.wrist.flexion],
    [`${s}.wrist.ulnarDeviation`, '手関節 尺屈（+）/ 橈屈（−）', ...ROM.wrist.ulnarDeviation],
  ];
};
const leg = (s) => [
  [`${s}.hip.flexion`, '股 屈曲（+）/ 伸展（−）', ...ROM.hip.flexion],
  [`${s}.hip.abduction`, '股 外転（+）/ 内転（−）', ...ROM.hip.abduction],
  [`${s}.hip.axialRotation`, '股 内旋（+）/ 外旋（−）', ...ROM.hip.axialRotation],
  [`${s}.knee.flexion`, '膝 屈曲', ...ROM.knee.flexion],
  [`${s}.ankle.dorsiflexion`, '足 背屈（+）/ 底屈（−）', ...ROM.ankle.dorsiflexion],
  [`${s}.ankle.inversion`, '足 内がえし（+）/ 外がえし（−）', ...ROM.ankle.inversion],
];
export const SLIDER_GROUPS = [
  { title: '体幹・頸部', items: [
    ['spine.flexion', '体幹 屈曲（+）/ 伸展（−）', ...ROM.spine.flexion],
    ['spine.lateralBend', '体幹 側屈（+右 / −左）', ...ROM.spine.lateralBend],
    ['spine.axialRotation', '体幹 回旋（+左 / −右）', ...ROM.spine.axialRotation],
    ['neck.flexion', '頸部 屈曲（+）/ 伸展（−）', ...ROM.neck.flexion],
    ['neck.lateralBend', '頸部 側屈（+右 / −左）', ...ROM.neck.lateralBend],
    ['neck.axialRotation', '頸部 回旋（+左 / −右）', ...ROM.neck.axialRotation],
    ['jaw.open', '顎関節 開口（下顎の回転＋前方滑走）', 0, 32],
  ] },
  { title: '右上肢', items: limb('right') },
  { title: '左上肢', items: limb('left') },
  { title: '右下肢', items: leg('right') },
  { title: '左下肢', items: leg('left') },
  { title: '手指', items: [
    ['right.grip', '右手 握り（0=パー, 100=グー）%', 0, 100],
    ['left.grip', '左手 握り（0=パー, 100=グー）%', 0, 100],
  ] },
];

/** camera presets: position / target in world metres (subject faces +Z, left = +X) */
export const VIEWS = {
  front: { label: '正面', pos: [0, 0.92, 2.9], target: [0, 0.86, 0.08] },
  side: { label: '側面', pos: [-2.9, 0.92, 0.08], target: [0, 0.86, 0.08] },
  back: { label: '後面', pos: [0, 0.92, -2.8], target: [0, 0.86, 0.08] },
  arm: { label: '上肢アップ', pos: [-0.22, 0.96, 0.95], target: [-0.17, 0.93, 0.08] },
  knee: { label: '膝アップ', pos: [-0.78, 0.46, 0.42], target: [-0.08, 0.38, 0.04] },
  // used by presets only (no button)
  armSide: { label: '上肢（側面）', pos: [-1.0, 1.02, 0.42], target: [-0.17, 0.98, 0.18], hidden: true },
  shoulderBack: { label: '肩（後面）', pos: [-0.35, 1.3, -1.25], target: [-0.15, 1.3, 0.05], hidden: true },
  top: { label: '斜め上', pos: [0, 2.7, 1.5], target: [0, 1.1, 0.08], hidden: true },
};
