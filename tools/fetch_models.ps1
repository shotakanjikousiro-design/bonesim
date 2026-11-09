# BoneSim - download MediaPipe model files for offline use.
# The light/full pose models and the hand model are already vendored in
# vendor/mediapipe/models/. Run this to (re)download them or to add the
# optional "heavy" pose model (~30 MB).
#   powershell -ExecutionPolicy Bypass -File tools\fetch_models.ps1 [-Heavy] [-Force]
param([switch]$Heavy, [switch]$Force)
$ErrorActionPreference = 'Stop'
$dest = Join-Path (Split-Path -Parent $PSScriptRoot) 'vendor\mediapipe\models'
New-Item -ItemType Directory -Force -Path $dest | Out-Null
$base = 'https://storage.googleapis.com/mediapipe-models'
$models = [ordered]@{
  'pose_landmarker_lite.task' = "$base/pose_landmarker/pose_landmarker_lite/float16/latest/pose_landmarker_lite.task"
  'pose_landmarker_full.task' = "$base/pose_landmarker/pose_landmarker_full/float16/latest/pose_landmarker_full.task"
  'hand_landmarker.task'      = "$base/hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task"
}
if ($Heavy) { $models['pose_landmarker_heavy.task'] = "$base/pose_landmarker/pose_landmarker_heavy/float16/latest/pose_landmarker_heavy.task" }
$ProgressPreference = 'SilentlyContinue'
foreach ($name in $models.Keys) {
  $out = Join-Path $dest $name
  if ((Test-Path $out) -and -not $Force) { Write-Host "skip  $name (exists; use -Force to re-download)"; continue }
  Write-Host "fetch $name"
  Invoke-WebRequest -Uri $models[$name] -OutFile "$out.part" -UseBasicParsing
  Move-Item -Force "$out.part" $out
}
Write-Host "done -> $dest"
