@echo off
rem BoneSim - download MediaPipe model files into vendor\mediapipe\models for offline use.
rem   tools\fetch_models.bat          (lite + full pose, hand; skips existing files)
rem   tools\fetch_models.bat heavy    (also the ~30 MB heavy pose model)
setlocal
set "DEST=%~dp0..\vendor\mediapipe\models"
if not exist "%DEST%" mkdir "%DEST%"
set "BASE=https://storage.googleapis.com/mediapipe-models"
where curl >nul 2>nul
if errorlevel 1 goto :ps
call :get pose_landmarker_lite.task "%BASE%/pose_landmarker/pose_landmarker_lite/float16/latest/pose_landmarker_lite.task" || goto :fail
call :get pose_landmarker_full.task "%BASE%/pose_landmarker/pose_landmarker_full/float16/latest/pose_landmarker_full.task" || goto :fail
call :get hand_landmarker.task "%BASE%/hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task" || goto :fail
if /i "%~1"=="heavy" call :get pose_landmarker_heavy.task "%BASE%/pose_landmarker/pose_landmarker_heavy/float16/latest/pose_landmarker_heavy.task" || goto :fail
echo done -^> %DEST%
exit /b 0

:get
if exist "%DEST%\%~1" ( echo skip  %~1 ^(exists^) & exit /b 0 )
echo fetch %~1
curl -L --fail -o "%DEST%\%~1.part" "%~2" || exit /b 1
move /y "%DEST%\%~1.part" "%DEST%\%~1" >nul
exit /b 0

:ps
echo curl not found, using PowerShell
if /i "%~1"=="heavy" ( powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0fetch_models.ps1" -Heavy ) else ( powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0fetch_models.ps1" )
exit /b %errorlevel%

:fail
echo download failed
exit /b 1
