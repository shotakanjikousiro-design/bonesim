# BoneSim 独立レビュー（Agent D）

Reviewer: Agent D (did not write any of the code). Date: 2026-11-10.
Scope: PLAN.md, README.md, `src/shared`, `src/rig`, `src/tracking`, `src/app`, `assets/rig.json`, `tests/`, `start.bat`, `vendor/`.
All numbers below come from my own probes (scratch scripts in `/tmp/probe/`, which load the real GLB + rig.json in node through `src/rig/anatomicalRig.js`) or from my own headless-Chrome screenshots (`/tmp/shots/`). I changed no project files except this one.

## 0. テスト結果 (test results)

| Command | Result |
|---|---|
| `npm test` (conventions / rig / tracking) | **ALL PASS** (all three suites) |
| `python3 tests/assets.test.py` | **2466 passed, 0 failed** |
| `python3 tests/smoke_browser.py` | **SMOKE PASS (0 console/page errors)**. I ran it on a copy in `/tmp/bscopy` because it **overwrites `docs/screenshot_*.png`** in the project. |
| Own headless run (1280×800 and 390×844 mobile) | No page errors; only swiftshader "GPU stall due to ReadPixels" warnings. No horizontal overflow at 390 px (`scrollWidth == 390`). |

The tests are good and broad, but they did not catch the Critical issue below, because the forearm-direction check uses a 3–5° tolerance against `stickFK`, which itself has a straight elbow.

---

## 1. [Critical]

### C1. The elbow shows a varus (inward) carrying angle in anatomical position. Valgus should be about 10–15°, and the on-screen text says the forearm points outward
- **Where:** `assets/rig.json:2437` / `:2863` (`humeroulnar_L/R.axes.flexion`), produced by `tools/build_assets.py:297-336`. The claim is in `src/app/presets.js:15` (「屈伸軸はわずかに斜めなので、伸展位では前腕がやや外側を向きます（肘角・運搬角）」) and in README/PLAN (「斜めの軸 → 運搬角が自然に出る」).
- **Evidence (anatomical pose, coupling ON):**
  - Humerus (GH→elbow centre) compared with forearm (elbow centre→radiocarpal centre), projected on the coronal plane: **L −2.8°, R −3.1°** (negative = forearm deviates *medially*, i.e. cubitus varus). Using the styloid midpoint gives L −3.4°, R −3.5°.
  - Diaphyseal shaft axes (centroids of 28–38 % and 62–72 % slices): humerus vs ulna **L −6.0°, R −6.8°** (varus).
  - Elbow flexed 90°: the forearm points **5.8° lateral** (L) and 5.6° lateral (R). At 145° it is still 2.2° lateral. A real valgus elbow brings the hand *medially* toward the shoulder or chest in flexion.
  - Root cause: the fitted flexion axis is exactly perpendicular to the humeral long axis in the coronal plane. The BodyParts3D mesh humerus is abducted 11.2°, and the axis medial end is 11.4° lower, giving dot(axis_medial, humerus_down) = 0.003. So the axis has **no obliquity relative to the humerus**. The mesh forearm also leans slightly medially (mesh rest: humerus 11.2°, forearm 10.3°).
  - This is visible in my screenshot (`/tmp/shots/elbows_front_bone.png`): both forearms angle in toward the thighs.
- **Why critical:** the carrying angle is a standard teaching point for elbow AP positioning, and it is the subject of the 肘の屈曲・伸展 preset. The model currently teaches the opposite of the text.
- **Fix:**
  1. In `build_assets.py`, add a rest-pose valgus correction. Rotate the ulna, radius, carpal and hand meshes about the AP axis through the elbow centre so the anatomical-pose carrying angle is about 10–12° (or add a `restPose.elbow.carrying` term to the rig, like the other rest corrections).
  2. Tilt the humeroulnar flexion axis about 5–7° (medial end distal relative to the plane perpendicular to the humeral shaft), so that the forearm comes to lie over the humerus or slightly medial in full flexion (the carrying angle "disappears" in flexion).
  3. Add tests for both sides: anatomical carrying angle in [8°, 15°] valgus (humeral vs ulnar shaft); at 90° flexion the forearm should be within about ±5° of sagittal; at ≥130° flexion the hand should be medial to the elbow.

---

## 2. [Major]

### M1. `start.bat`: the `py` launcher fallback can never work, and the Microsoft Store alias for `python` defeats detection
- **Where:** `start.bat:6-17`.
- **Evidence:**
  - `%errorlevel%` inside the parenthesised `if … ( … ) else ( where py … if %errorlevel%==0 …)` is expanded **when the whole block is parsed**, before `where py` runs. In the `else` branch the inner test therefore always sees the non-zero result of `where python`, so it always prints "Python が見つかりません" and exits, even when `py` exists.
  - On stock Windows 10/11, `where python` succeeds even without Python because of the App-execution alias `%LOCALAPPDATA%\Microsoft\WindowsApps\python.exe`. In that case `PY=python`, the stub prints "Python was not found; … Microsoft Store" (exit 9009), and the script then reports 「ポート %PORT% が使用中かもしれません」, which is misleading. This is exactly the case of a python.org install with the default "Add to PATH" left unchecked, where only `py` is available.
  - The browser is opened (`start "" URL`, line 22) **before** the server starts, so the first page load can hit "connection refused".
- **Fix:** probe by *running* each candidate:
  ```bat
  set "PY="
  py -3 --version >nul 2>&1 && set "PY=py -3"
  if not defined PY python --version >nul 2>&1 && set "PY=python"
  if not defined PY ( echo ... & pause & exit /b 1 )
  ```
  Then launch a small `tools/serve.py` that binds, sets MIME types (see M2), and calls `webbrowser.open()` after binding. Consider `--bind 127.0.0.1` so Windows Firewall does not prompt (and document LAN use separately). Optionally keep the .bat ASCII-only: the UTF-8 Japanese `echo` lines rely on `chcp 65001`, which has historically been fragile in cmd. Line 2 (`rem` in UTF-8) is parsed in CP932 before `chcp` runs, and its last byte is a CP932 lead byte that swallows the CR. This is harmless with CRLF line endings, but would break the next line if the file were ever saved with LF.

### M2. Windows `http.server` MIME risk for ES modules
- **Where:** `start.bat:24`, README 起動方法.
- **Evidence:** Python's `mimetypes` on Windows reads the registry, and a common misconfiguration (`HKCR\.js` Content Type = `text/plain`) makes http.server serve `.js` as text/plain. Browsers then refuse every `<script type="module">` / `import`, so the whole app fails with 「骨格モデルの読み込みに失敗しました」 or a blank page. (`.mjs` and `.wasm` are in Python's defaults since 3.8, so only the registry-override case is a problem.) This is a well-known pitfall: [bugs.python.org msg392294](https://bugs.python.org/msg392294), [Windows MIME type detection pitfalls](https://www.taricorp.net/2020/windows-mime-pitfalls/).
- **Fix:** ship `tools/serve.py` with `SimpleHTTPRequestHandler.extensions_map.update({'.js':'text/javascript','.mjs':'text/javascript','.wasm':'application/wasm','.json':'application/json','.glb':'model/gltf-binary'})` and use it from start.bat and start.sh.

### M3. Patellar flexion is about 0.9× knee flexion; literature and PLAN.md say about 0.6–0.75×
- **Where:** `src/rig/anatomicalRig.js:190-217, 302-308`; `src/rig/coupling.js:178`.
- **Evidence (right knee, patella base→apex angle in the femoral sagittal plane):** knee 30° → 24.7°, 45° → 41.5°, 60° → 57.9°, **90° → 86.6°**, 120° → 106.6°. That is a ratio of 0.85–0.92 between 45° and 120°. In-vivo patellar flexion lags knee flexion at roughly 0.6–0.7×, so about 60–65° at 90°. PLAN.md itself specifies 「屈曲の約0.7倍」. The test accepts 40–90° at 90°, so 87° passes at the very top of the range.
- **Why it matters:** the patellar tilt is what students see on skyline/lateral knee views. A 25° error at 90° is visible.
- **Fix:** let the patella rotate about its own contact point (apex/tendon-constrained four-bar) rather than rigidly about the trochlear circle centre, or blend `phi` toward 0.7·knee. Tighten the test to 50–75° at 90°.

### M4. Tracking accuracy is validated only on synthetic data with an optimistic noise model
- **Where:** `tests/tracking.test.mjs`, `src/tracking/filters.js:104-114`.
- **Evidence:** every tracking number in the tests (RMS 1.3–5.2°, lag 33–67 ms, jitter ratio 0.31) uses `synth.js`: `stickFK` landmarks plus **i.i.d. isotropic 1 cm** world noise. Real MediaPipe world landmarks have depth (z) errors several times larger than x/y, with low-frequency drift and occasional left/right swaps in profile view. Profile view is the most common view in radiography (lateral projections). The headless fake-camera run only checks that "no person" is reported.
- **Fix:** record 5–10 short real clips (front, 45°, true lateral, arm overhead, forearm pointing at camera) with goniometer ground truth for elbow, knee and shoulder elevation. Report per-angle error and visible jitter. Add a correlated depth-noise and swap model to `synth.js` for regression. Document in 推定の限界 that true lateral (profile) and limbs pointing at the camera are the least reliable.

### M5. The wrist radiocarpal:midcarpal share is wrong for extension
- **Where:** `src/rig/coupling.js:118` (`WRIST_SHARE = { flexion: 0.4, … }`), text `src/app/presets.js:50`.
- **Evidence:** the model uses 40 % radiocarpal for both flexion and extension. Sarrafian 1977 (cited in the code) reports flexion ≈ 40 % RC / 60 % MC, but **extension ≈ 66 % RC / 34 % MC**. The 手関節の掌屈・背屈 preset sweeps −70…+80, so half of it teaches the wrong split, and the text asks students to watch the lunate.
- **Fix:** use a sign-dependent share, e.g. `rc = flex >= 0 ? 0.4 : 0.67`, and update the text: 「掌屈は手根中央関節、背屈は橈骨手根関節が主」.

---

## 3. [Minor]

### Kinematics / anatomy
1. **Shoulder rise is large: the GH centre rises 53 mm at 90° and 75 mm at 180° abduction; the acromion rises 47 / 55 mm** (`coupling.js:63-64`, clavicle elevation 2°/10°, max 18°). The README itself argues that real shoulder rise is about 2–4 cm. Ludewig 2009 clavicle elevation is about 10° or less at 120° humerothoracic elevation. Suggest about 1.2°/10° with a max of about 11°, and add a test on GH-centre rise (< 40 mm at 90°).
2. **Clavicle protraction during flexion** (`coupling.js:89-90`, up to 10°) is the opposite of modern 3-D data, where the clavicle *retracts* about 15–30° during elevation in all planes (Ludewig 2009). Suggest replacing it with retraction (or making it plane-independent), or at least citing it as the classical/simplified view.
3. **Scapula–rib-cage interpenetration:** scapula vertices inside the per-slice convex hull of the ribs: 0 at rest, but **up to 14–17 mm deep** at 90–150° abduction (both sides; about 10–15 % of sampled vertices). The scapula rotates rigidly about the AC joint and does not glide on the thorax. Suggest adding a small protraction/translation term or an ellipsoid-thorax constraint.
4. **The girdle model ignores the plane of elevation for extension:** `plane −90, elevation 50` (shoulder extension) still gives +8° upward rotation and +10° clavicle elevation. In extension the scapula tilts anteriorly and rotates downward. Suggest `upward *= smoothstep(-60, 0, plane)` or similar.
5. **Shoulder ROM is not plane-dependent:** sliders and `clampPose` allow `plane = −90` with `elevation` up to 180 (JOA 伸展 is 50°). External rotation is clamped to 60° in every position. JOA's 60° applies to 1st position, but 90/90 (2nd position) external rotation is about 90°, so camera mode cannot show a normal throwing-position arm (`conventions.js:81`, `filters.js:224`). Suggest limiting elevation to about 50° when `plane < −45`, and relaxing external rotation with elevation.
6. **The 肩の外転 preset runs to 170° with the humerus in 0° rotation** (`presets.js:27`); `autoHumeralRotation` defaults to false (`anatomicalRig.js:219`). Full abduction requires about 35–40° humeral external rotation (greater tuberosity clearance). Suggest enabling it for that preset or mentioning it in the text.
7. **Spine distribution is too lumbar-heavy:** flexion has 88 % at L1–S1 and 12 % for the whole thoracic spine; lateral bend has 22 % thoracic (`coupling.js:210-211`). Thoracic flexion is typically about ⅓ and thoracic lateral bend about half of the thoracolumbar total (White & Panjabi). Since the thorax is a single rigid segment this is partly by design, but the weights should still reflect about 30 % / 50 %.
8. **Screw-home range inconsistency:** code uses 25° (`coupling.js:136`, README), but the preset text says 「最後の約20°」 (`presets.js:42`). The direction is correct on both sides: with coupling ON the tibial tuberosity moves 7.7 mm medially between 0° and 30°, i.e. internal rotation on flexion and external at terminal extension (L/R twist +10.0° / −10.0°). The rotation axis passes through the midpoint of the condyles; a medial pivot would be more realistic but is optional.
9. **Patella text overclaims** 「膝蓋腱の長さを保ったまま」 (`presets.js:43`): the model treats the tendon as slack at 0–~12° (rest 25 mm, taut 36–37 mm). Suggest 「膝蓋腱が張ってからは長さを保ったまま」.
10. **Fist text:** 「指先は手掌の中央（舟状骨のあたり）に向かって集まります」 (`presets.js:69`). The scaphoid is not the centre of the palm. Suggest 「各指を屈曲すると指先は舟状骨結節（手掌の橈側近位）の方向を向きます」.
11. **Thumb ROM** `cmcAbduction [0,70]` and `cmcFlexion [-10,50]` (`conventions.js:89`) are not JOA values (JOA: 橈側外転 0–60, 掌側外転 0–90, MP 屈曲 60 / 伸展 10, IP 屈曲 80 / 伸展 10). Finger DIP extension 10° is missing (minor).
12. **Coccyx is missing:** PLAN says pelvis = 寛骨+仙骨+尾骨, but `segments.pelvis.nodes` = sacrum, hipBone_L/R only (`coccyxApproxTip` landmark only). The coccyx matters for sacrum/coccyx projections; add the BodyParts3D coccyx (FMA20229) if available.
13. **Other directions that I verified as correct on both sides** (no action needed, listed for confidence):
    - **Pronation at elbow 90°:** with the radius running radial styloid → ulnar styloid, the styloid offset goes from L (+40, 5, 3) mm (palm up) to (2, 42, 3) (thumb up) to (−35, 5, 1) (palm down). R is mirrored. Ulnar styloid and olecranon stay fixed to < 0.5 mm, and the hand follows the radius.
    - Coupling OFF twists the ulna (by design, and labelled).
    - **Scapula:** upward rotation moves the inferior angle laterally (+68 mm at 150°); posterior tilt moves the top backward.
    - **Clavicle:** elevation lifts the lateral end; posterior rotation turns the superior surface backward; protraction moves the lateral end forward.
    - **Hip:** with hip and knee at 90°, internal rotation swings the foot laterally.
    - **Ankle:** dorsiflexion raises the toes 18.6° for 20° input. The oblique Inman axis couples dorsiflexion with abduction (+6°) and plantarflexion with adduction (−22°).
    - **Subtalar:** inversion 30 gives adduction 23° plus 6° plantarflexion, i.e. supination.
    - **Wrist and fingers:** palmar flexion and ulnar deviation go to the correct sides; finger abduction is away from the middle finger.
    - **Spine and neck:** C1/C2 carries 50 % of neck rotation.
    - **Axis mirror symmetry:** every L/R axis pair is a correct pseudo-vector mirror, (x, y, z) → (x, −y, −z).
14. **Japanese bone names:** I spot-checked about 45 names, all correct: 橈骨, 尺骨, 舟状骨, 月状骨, 三角骨, 豆状骨, 大菱形骨, 小菱形骨, 有頭骨, 有鉤骨, 距骨, 踵骨, 立方骨, 舟状骨（足）, 内側/中間/外側楔状骨, 膝蓋骨, 肩甲骨, 鎖骨, 胸骨柄, 胸骨体, 剣状突起, 寛骨, 仙骨, 環椎（第1頸椎）, 軸椎（第2頸椎）, 第n胸椎/腰椎, 第n肋骨, 肋軟骨, 第n中手骨/中足骨, 示指/中指/環指/小指 基節骨・中節骨・末節骨, 母指基節骨/末節骨, 舌骨, 下顎骨, 腓骨, 大腿骨, 脛骨, 上腕骨. JOA ROM values for shoulder, elbow, forearm, wrist, hip, knee, ankle (2022 内がえし 30 / 外がえし 20), neck and thoracolumbar spine all match the table.

### Tracking
15. **Limbs leaving the frame:** an angle is held only when confidence < 0.15 (`filters.js:156,183`). MediaPipe often reports visibility 0.2–0.5 for extrapolated off-screen joints, so hallucinated limbs keep driving the rig (slower filter, drawn faded below 0.4). Suggest also treating landmarks outside the image (normalized x/y ∉ [−0.05, 1.05]) as vis = 0, and raising `minConfidence` to about 0.3 for distal joints.
16. **Hip rotation from the foot:** `footRotationOffset` defaults to 0 (`angles.js:149`), so normal toe-out (about 8° in this mesh, and similar in people) reads as hip external rotation of about 4° whenever the knee is straight. Suggest a default of about 8–10° or calibrating it in `calibrate()`.
17. **Phones:** the main thread runs `pose.detectForVideo` plus `hand.detectForVideo` (2 hands, every 2nd frame on mobile) synchronously, then renders 232 k bone triangles with a transparent X-ray shader. Defaults are sensible (lite model on coarse pointers, `handsEvery: 2`), but expect about 10–15 fps on mid-range phones. Consider moving MediaPipe into a Worker (OffscreenCanvas / ImageBitmap transfer) and/or a decimated "mobile" GLB (about 60 k triangles). Headless swiftshader gave 4 fps for the demo, which is not representative.
18. **Only `vision_wasm_internal` (SIMD) is vendored.** Browsers without WASM SIMD (Safari < 16.4) will request the missing `vision_wasm_nosimd_internal.*` and fail offline. Either vendor the nosimd pair or state the minimum browser versions.
19. **Telemetry vs 「映像は端末内で処理され、送信されません」** (`index.html:87`): video is indeed not sent, but the vendored tasks-vision 1.2.2 contains a usage logger that POSTs to `https://odml.pa.googleapis.com/v1/log` every 60 s (`vendor/mediapipe/vision_bundle.mjs`, class with `setInterval(flush, 6e4)`). The smoke test whitelists that request. In hospital or school networks this matters. Suggest rewording to 「映像・骨格データは送信されません（MediaPipe ライブラリが利用統計を Google に送信しようとする場合があります）」 or stubbing the logger.

### Practical / UI
20. **Folder size:** 62 MB without `node_modules`. Of that, `docs/` is 18 MB of screenshots, `vendor/mediapipe` 37 MB (wasm 14.6 MB plus three models 23 MB), `vendor/three` 2.3 MB, `assets` 4.7 MB. The runtime set (excluding docs, tools, tests) is **44 MB**. **`node_modules` (24 MB) is not needed at runtime** (only `npm test` uses it; the browser uses `vendor/three`, and the versions match at r187). Suggest moving screenshots out of the distributed folder or into a separate zip.
21. **Offline completeness:** OK. `index.html` and `src/app`, `src/rig` and `src/shared` reference no remote URLs. The three pose/hand `.task` files are valid zips (pose_lite, pose_full, hand), and the wasm and bundle are vendored. CDN/GCS URLs are fallbacks only.
22. **Licences:** OK. BodyParts3D CC BY-SA 2.1 JP is in the footer, README and `assets/LICENSE-ASSETS.md` (with licence URL, modification list and ShareAlike statement); three.js MIT is in `vendor/three/LICENSE`; MediaPipe Apache-2.0 is in `vendor/mediapipe/LICENSE` and README. Possible addition: a top-level `LICENSE`/`NOTICE` that lists all three, so a redistributed zip is self-explanatory.
23. **Mobile (390 px):** no overflow, readable text. During a preset, the explanation box covers about 35 % of the 3D viewport, including the moving shoulder (`/tmp/shots/m_preset_abd.png`). Suggest collapsing it to one line with a tap to expand on narrow screens.
24. **`tests/smoke_browser.py` writes into `docs/`.** Running the test changes tracked artefacts. Suggest writing to `tests/out/` by default.
25. **Teaching text vs sliders lag:** the explanation is throttled to 120 ms (`simulator.js:99`), so during presets the text can differ from the slider by several degrees (seen: text 78° vs slider 85°). This is cosmetic.

---

## 4. 推奨する優先順位 (suggested order)
1. C1 carrying angle: fix the geometry and add L/R tests.
2. M1, M2 start.bat and serve.py: these block the main target users on Windows.
3. M3 patella ratio and M5 wrist share: these are teaching-visible kinematics.
4. M4: a small real-video validation set before claiming 「ノイズに強い追跡」.
5. Minor items as time permits (shoulder rise and clavicle retraction, the extension-plane girdle, the telemetry wording).

---

## 5. 対応状況（Agent C, 2026-11-10）

Re-run after the fixes: `npm test` passes all three suites (rig, conventions and tracking). `python3 tests/assets.test.py` passes 2466 checks with 0 failures. `python3 tests/smoke_browser.py` reports **SMOKE PASS**, and its screenshots now go to `docs/screenshots/`.

| # | 項目 | 状況 | 内容 |
|---|---|---|---|
| C1 | 運搬角が逆（内反） | **修正** | In the rig, `anatomicalRig.js` "elbow": E undoes the mesh's rest flexion, then a valgus correction V sets the humeral-shaft / ulnar-shaft angle in extension to **10°**. Measured: humerus–forearm about 14°, joint centres about 12.5° on both sides. The flexion axis is now tilted, medial end distal: about 11–12° relative to the humeral mechanical axis, which is about 18° relative to the PCA shaft of this mesh. With this tilt the forearm is about 1.7° from the sagittal plane at 90° flexion, and at 140° the wrist is about 30 mm medial to the elbow. I also added a rest humeral rotation (−8.5°), derived so that the flexion axis is coronal in the anatomical position (conventions: elbow flexes in the sagittal plane). New L/R tests check the carrying angle (8–15°), the 90° forearm direction (±5°), and the wrist being medial at 140°. Screenshots: `screenshots/screenshot_sim_elbows_{anatomical,flex90_top,flex140}.png`. The camera-mode overlay (synthetic demo) still aligns. |
| M1 | start.bat | **修正** | Rewritten with `chcp 65001` first. It probes `py -3 --version`, then `python --version`, then `python3 --version` by running them, so the Store stub is rejected. There is no `%errorlevel%` inside blocks (it uses goto labels and `if not errorlevel`). It starts `serve.py`, which opens the browser after binding and tries ports 8000–8019 with a clear message. Japanese error messages; `%*` is passed through (`start.bat --lan`). `start.sh` mirrors this. |
| M2 | MIME | **修正** | New `serve.py` (stdlib only). It forces .js/.mjs → text/javascript, .wasm, .glb, .task and .json types; binds 127.0.0.1 by default, or 0.0.0.0 with `--lan` (prints the LAN URL and the HTTPS caveat); auto-selects a free port; and opens the browser when ready. |
| M3 | 膝蓋骨 0.9× | **修正** | Patellar flexion = 0.7 × knee flexion (66° at 90°). The centroid glides on the trochlear circle; the slack/taut tendon is held constant once taut (28–29 mm). A tabulated outward lift keeps contact: 0 vertices more than 1.5 mm inside the femur, and the tibia is checked too. Test range tightened to 50–75°. |
| M4 | 実データ検証 | **未対応（要実データ）** | Documented in README 「既知の制限」 and in the in-app 推定の限界 (profile views, limbs pointing at the camera, not yet validated on real people). |
| M5 | 手関節の分担 | **修正** | `wristSplit` is now sign-dependent: palmar flexion 40:60, dorsiflexion 66.5:33.5 (Sarrafian 1977), deviation 60:40. Preset text updated. |
| m1 | 肩の上昇 | **修正** | Clavicle elevation is 0.6°/10° (max 6°). The GH centre rises about 20 mm at 90° abduction (test: 10–25 mm). |
| m2 | 鎖骨の前方突出 | **修正** | Replaced by plane-independent retraction of up to 18° (Ludewig 2009). |
| m3 | 肩甲骨の胸郭へのめり込み | **修正** | Rib-cage outline: a star-shaped profile per 1-cm slice of the thorax segment. A minimal scapular internal-rotation / tilt correction (coordinate descent; upward rotation is never changed) keeps penetration beyond the rest clearance at 0 mm (tests at 90° and 150°). |
| m4 | 伸展面 | **修正** | Elevation-driven girdle terms fade out for planes −15 → −75°. Extension adds slight downward rotation and anterior tilt. |
| m5 | 面依存のROM | 未対応 | `conventions.js` is frozen. |
| m6 | 外転プリセットの外旋 | **修正** | The preset adds up to 35° external rotation above 80°, and the text explains why. |
| m7 | 脊柱の分配 | **修正** | Thoracic share: flexion 30 %, lateral bend 50 %, rotation 75 %. |
| m8 | スクリューホーム 20/25° | **修正** | The code now uses 20° (`SCREW_HOME.range`), consistent with the text and README. |
| m9, m10 | 膝蓋腱・グーの文言 | **修正** | Texts reworded. |
| m11, m12 | 母指ROM・尾骨 | 未対応 | Needs `conventions.js` / assets. |
| m15 | 画面外の関節 | **修正** | `pipeline.js`: landmarks outside the image ([−0.05, 1.05]) get visibility 0. `filters.js`: distal joints (elbow, forearm, wrist, knee, ankle, fingers) hold below a confidence of 0.3 (`minConfidenceDistal`). Tracking tests pass. |
| m16 | 足のつま先外向き | 未対応 | |
| m17 | スマホ性能 | 未対応 | Worker / decimated GLB deferred. The lite model and hand toggle remain. |
| m18 | Safari < 16.4 | 記載 | Stated in README 既知の制限. |
| m19 | テレメトリ | **修正** | New `src/tracking/noTelemetry.js`, imported first by `tracker.js`, answers the MediaPipe usage-log POST to odml.pa.googleapis.com locally with HTTP 204, after which the logger stops. The in-app wording is now truthful: video and pose data are not sent, and the library's statistics upload is blocked. |
| m20, m24 | スクリーンショット | **修正** | Screenshots moved to `docs/screenshots/`; the smoke test writes there. |
| m21–23, m25 | その他 | 未対応 / 確認済み | |
