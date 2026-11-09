# BoneSim — 骨透かしシミュレーター 計画書

## ゴール
カメラに映った人の骨格を、関節の「動き方」まで解剖学的に再現して重ねて表示する。
- **シミュレーターモード**: カメラなし。スライダー／プリセット動作で各関節を動かし、骨同士がどう動くか（橈骨が尺骨の上を回る、肩甲骨が回旋する、膝のスクリューホーム等）を3Dで観察。
- **カメラモード**: Webカメラ映像から関節角度を推定し、同じリグを駆動してX線風に重ねる。
- 教育用（診療放射線技師の学生・一般）。医療判断には使わない旨を画面に明記。

## 既存アプリ（X-RAY PRO 等）との違い ＝ 今回の重点
1. **骨ごとの正しい関節運動**（剛体の骨を、実際の関節中心・関節軸のまわりに回す）
2. **連動運動**: 肩甲上腕リズム、前腕回内外（橈骨が尺骨のまわりを回り手は橈骨についていく）、手根骨近位列の分担、膝スクリューホーム＋膝蓋骨の滑走、脊柱の分節分配
3. **ノイズに強い追跡**: One Euro フィルタ、関節可動域(ROM)制限、骨長一定のキャリブレーション、信頼度が低い関節は保持して減衰
4. 将来: 撮影体位テンプレート、体表から触れる骨指標、中心線・照射野の表示

## 技術構成
- ブラウザ（PC/スマホ）で動く ES Modules 構成。ビルド不要。`three` を `vendor/` に同梱、importmap で読み込み。
- 起動: Windows では `start.bat`（Python の http.server）→ `http://localhost:8000`
- 骨モデル: BodyParts3D（© DBCLS, CC BY-SA 2.1 JP）→ GLB
- 追跡: MediaPipe Tasks Vision（Pose Landmarker + Hand Landmarker + Face Landmarker）。顔の追跡で頭蓋を顔に合わせ、jawOpen ブレンドシェイプで下顎（顎関節）を動かす。解析したフレームそのものを `<canvas id="camFrame">` に表示して、映像と骨のずれ（遅延）をなくす。

## ディレクトリ
```
bonesim/
  index.html, start.bat, README.md, PLAN.md
  src/shared/conventions.js   ← 座標系・関節角度の唯一の定義（基準FK stickFK）
  src/rig/        ← Agent C: GLB+rig.json を読み Pose で骨を動かす（連動運動含む）
  src/tracking/   ← Agent B: カメラ → Pose
  src/app/        ← Agent C: UI・描画・モード切替
  assets/skeleton.glb, assets/rig.json, assets/LICENSE-ASSETS.md   ← Agent A
  vendor/         ← three, mediapipe wasm 等
  tools/          ← アセット生成スクリプト（Python）
  tests/          ← node テスト
```

## 共通仕様（全エージェント必読）
`src/shared/conventions.js` の冒頭コメントが正。要点:
- 右手系・メートル、**+X=被写体の左、+Y=上、+Z=前**。安静姿勢＝解剖学的正位（手掌前向き＝前腕完全回外）。
- `Pose` の形と各関節の回転の定義、ROM（日本整形外科学会/日本リハビリテーション医学会の基準）。
- `stickFK(pose)` が基準の順運動学。追跡（逆算）もリグ（メッシュ）もこれと一致させること。
- 前腕 `pronation`: 0=中間位, -90=解剖学的正位（回外）, +90=回内。

## rig.json スキーマ（Agent A が出力、Agent C が使用）
```jsonc
{
  "units": "m", "frame": {"left": "+X", "up": "+Y", "forward": "+Z"},
  "source": "BodyParts3D ...",
  "nodes": {            // GLB 内のメッシュノード名 → 情報
    "radius_R": {"segment": "radius_R", "nameJa": "橈骨（右）", "nameEn": "Right radius", "fma": "FMA23464", "group": "upperLimb"}
  },
  "segments": {         // 剛体として一緒に動く単位
    "radius_R": {"parent": "ulna_R", "joint": "radioulnar_R", "nodes": ["radius_R"]}
  },
  "joints": {           // 関節中心と解剖学的な軸（rest pose・ワールド座標）
    "humeroulnar_R": {"center": [x,y,z], "axes": {"flexion": [x,y,z]}, "method": "cylinder fit of trochlea+capitulum"},
    "radioulnar_R":  {"center": [..], "axes": {"pronation": [..]}, "proximal": [..], "distal": [..], "method": "radial head centre → ulnar head centre"}
  },
  "landmarks": { "ASIS_R": [x,y,z], "xiphoid": [..], "jugularNotch": [..], "medialEpicondyle_R": [..], ... },
  "dims": { ...DEFAULT_DIMS と同じキー。メッシュから測った値 }
}
```
Segment ID 一覧（サイド接尾辞 `_L` / `_R`）:
`pelvis`(寛骨+仙骨+尾骨, root) · `L5`…`L1` · `thorax`(胸椎+肋骨+肋軟骨+胸骨) · `C7`…`C1` · `skull`(頭蓋+下顎+歯)
上肢: `clavicle` `scapula` `humerus` `ulna` `radius` `carpalsProximal`(舟状・月状・三角・豆状) `hand`(遠位手根列+第2–5中手骨) `thumbMeta` `thumbProx` `thumbDist` `{index,middle,ring,little}{Prox,Mid,Dist}`
下肢: `femur` `patella` `tibia`(脛骨+腓骨) `talus` `foot`(踵骨+残りの足根骨・中足骨・趾骨)

Joint ID 一覧（`_L`/`_R`）:
`sternoclavicular` `acromioclavicular` `glenohumeral` `humeroulnar` `radioulnar` `radiocarpal` `midcarpal` `cmc1` `mcp1..5` `pip2..5` `dip2..5` `ip1` · `hip` `knee` `patellofemoral` `talocrural` `subtalar` · `lumbosacral` `L1L2`…`L4L5` `T12L1` `C7T1` `C1C2…` `atlantooccipital`

## 連動運動の仕様（Agent C が実装）
- **肩**: Pose の shoulder は「上腕骨と胸郭の相対角(humerothoracic)」。最初の約30°（setting phase）以降、上腕挙上の約1/3を肩甲骨上方回旋で担う（全体で GH:ST ≈ 2:1, Inman）。鎖骨は挙上90°まで 10°あたり約4°挙上、それ以降は後方回旋。肩甲骨は胸郭上を滑る（SC/AC関節まわりの回転で近似）。GH 回転 = HT 回転 ∘ (肩甲骨の寄与)^-1 として、上腕骨の最終的な向きは Pose どおりにする。
- **肘**: ulna は上腕骨の humeroulnar 軸まわりに回転（軸は幾何学的に求めた斜めの軸 → 運搬角が自然に出る）。radius は ulna の子。
- **前腕**: radius は radioulnar 軸（橈骨頭中心→尺骨頭中心）まわりに回転。尺骨は動かない。手根骨・手は radius の子（＝手は橈骨についていく）。
- **手関節**: 掌屈/背屈は 橈骨手根関節:手根中央関節 ≈ 4:6、橈屈/尺屈も近位列が一部担う。
- **膝**: femur に対し tibia が屈曲。伸展最終20°で脛骨が最大約10°外旋（スクリューホーム）。膝蓋骨は膝蓋腱長一定で大腿骨溝を滑る（屈曲の約0.7倍で回転＋下方移動の近似で可）。
- **脊柱**: spine 角を L5–L1 と thorax に分配（屈曲は腰椎重め、回旋は胸椎重め）、neck を C7–C1 と頭部に分配（回旋は C1-C2 に約50%）。

## エージェント分担
| | 担当 | 成果物 |
|---|---|---|
| A | 骨アセット | `assets/skeleton.glb`, `assets/rig.json`, `assets/LICENSE-ASSETS.md`, `tools/build_assets.py`, 確認画像 `docs/asset_check_*.png` |
| B | カメラ追跡 | `src/tracking/*.js`, `vendor/mediapipe/*`, `tests/tracking.test.mjs`（stickFK との往復テスト）, `tracking-debug.html` |
| C | リグ＋アプリ | `src/rig/*.js`, `src/app/*.js`, `index.html`, `start.bat`, `README.md`, テスト |
| D | 独立レビュー | `docs/REVIEW.md` |

## 環境メモ
- 作業環境から到達可能: npm レジストリ, github.com / raw.githubusercontent.com（git clone 可）
- 到達不可: storage.googleapis.com（MediaPipe モデル）, dbarchive.biosciencedbc.jp
- ヘッドレス確認: `/usr/bin/google-chrome`, playwright 1.63
- BodyParts3D STL: `git clone https://github.com/Kevin-Mattheus-Moerman/BodyParts3D`（CC BY-SA 2.1 JP）
