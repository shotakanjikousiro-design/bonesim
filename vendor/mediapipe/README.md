# vendor/mediapipe — MediaPipe Tasks Vision (vendored)

Everything needed to run pose + hand tracking from `http://localhost` with **no
network access**.

| file | origin | licence |
|---|---|---|
| `vision_bundle.mjs`, `vision.d.ts`, `README.tasks-vision.md` | npm `@mediapipe/tasks-vision@1.2.2` (unchanged) | Apache-2.0 (`LICENSE`) |
| `wasm/vision_wasm_internal.js`, `wasm/vision_wasm_internal.wasm` | npm `@mediapipe/tasks-vision@1.2.2` `wasm/` (unchanged; the `_module_` variant is not needed) | Apache-2.0 |
| `models/pose_landmarker_full.task` | Google MediaPipe Pose Landmarker (BlazePose GHUM 3D, full), float16 | Apache-2.0 |
| `models/pose_landmarker_lite.task` | Google MediaPipe Pose Landmarker (lite), float16 — for phones | Apache-2.0 |
| `models/hand_landmarker.task` | Google MediaPipe Hand Landmarker, float16 | Apache-2.0 |
| `models/face_landmarker.task` | Google MediaPipe Face Landmarker (face mesh 478 points + blendshapes), float16 — head fit and mouth opening | Apache-2.0 |
| `LICENSE` | Apache License 2.0 text from github.com/google-ai-edge/mediapipe | — |

Copyright Google LLC. The models are Google's MediaPipe models, published under
the Apache License 2.0 (see the MediaPipe model cards / solution pages at
https://ai.google.dev/edge/mediapipe/solutions/vision/pose_landmarker and
…/hand_landmarker).

## How the model files were obtained

`storage.googleapis.com` (the official host) was not reachable from the build
workspace, so byte-identical copies were taken from npm packages that
redistribute the official files unchanged:

| file | taken from | sha256 |
|---|---|---|
| `pose_landmarker_full.task` | `expo-mediapipe-pose@0.3.0` `assets/` | `5134a3aa…a839011b1` |
| `pose_landmarker_lite.task` | `pose-landmarker-react-native@0.1.0` `android/src/main/assets/models/` | `59929e1d…a690d574a` |
| `hand_landmarker.task` | `@beautinique/frontend-tryon-assets@1.0.2` `models/` (NOTICE: "copied as downloaded, bytes not changed", Apache-2.0) | `fbc2a300…bcf3607cde1` |
| `face_landmarker.task` | GitHub `torinmb/mediapipe-touchdesigner` (`src/mediapipe/models/face_landmark_detection/`, Git LFS) | `64184e22…6544e0bc9ff` (3,758,596 bytes) |

The face model's sha256 is the same in the Git LFS pointers / checksum files of several unrelated
repositories that redistribute the official file (`torinmb/mediapipe-touchdesigner`,
`mangelajo/flossy`, `njmill/tobii-linux`, `yousefelkilany/Word-level-Arabic-Sign-language`, …).
Its canonical face mesh (`geometry_pipeline_metadata_landmarks.binarypb`) is what
`tools/build_face_model.py` registers to the skull to produce `src/tracking/faceModel.js`.

Verification: a `.task` file is a zip of TFLite models. The inner
`pose_detector.tflite` / `pose_landmarks_detector.tflite` are byte-identical
across three independent npm packages (`expo-mediapipe-pose`,
`vision-camera-pose-landmarks-plugin`, `pose-landmarker-react-native`; zip
timestamps 2023-04-28 differ only by seconds), and their embedded metadata names
the original Google training artefacts
(`blazepose_ghum_39kp_full_oss_2021_07_02`, `blazepose_detector_…_2021_10_18`,
`handskeleton_handflag_handedness_full_2021_10_07_v0`,
`palmdetector_full_192x192_2021_09_30_v0`). They load and run with the vendored
tasks-vision 1.2.2 (checked in headless Chrome on real photos).

If you prefer the files straight from Google, or want the optional
`pose_landmarker_heavy.task` (~30 MB, most accurate, slow on phones), run on
Windows:

```
tools\fetch_models.bat            (or: tools\fetch_models.bat heavy)
powershell -ExecutionPolicy Bypass -File tools\fetch_models.ps1 -Heavy -Force
```

## Runtime loading order (src/tracking/tracker.js)

1. **Models** – `vendor/mediapipe/models/<name>.task` (local) → official URL
   `https://storage.googleapis.com/mediapipe-models/...<name>/float16/latest/<name>.task`.
   The file is fetched once into memory and passed as `modelAssetBuffer`.
   Missing heavy model → falls back to the Google URL automatically.
2. **WASM** – `vendor/mediapipe/wasm/` → `https://cdn.jsdelivr.net/npm/@mediapipe/tasks-vision@1.2.2/wasm`.
3. **Delegate** – GPU, falling back to CPU if GPU creation fails or if GPU
   inference is pathologically slow (> 120 ms/frame median, e.g. software WebGL).

## Updating

```
npm pack @mediapipe/tasks-vision@<version>
# copy package/vision_bundle.mjs, package/vision.d.ts, package/wasm/vision_wasm_internal.{js,wasm}
```
and update the jsDelivr fallback version in `src/tracking/tracker.js`.
