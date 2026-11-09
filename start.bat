@echo off
chcp 65001 >nul
setlocal
cd /d "%~dp0"
rem ---- BoneSim launcher: finds Python, starts serve.py (opens the browser when ready) ----

set "PY="
py -3 --version >nul 2>&1
if not errorlevel 1 set "PY=py -3"
if defined PY goto :found
python --version >nul 2>&1
if not errorlevel 1 set "PY=python"
if defined PY goto :found
python3 --version >nul 2>&1
if not errorlevel 1 set "PY=python3"
if defined PY goto :found
goto :nopython

:found
echo BoneSim を起動します（Python: %PY%）...
%PY% serve.py %*
if errorlevel 2 goto :noport
if errorlevel 1 goto :failed
goto :eof

:nopython
echo.
echo Python 3 が見つかりませんでした。
echo   1. https://www.python.org/downloads/ から Python 3 をインストールしてください。
echo      （インストール画面で「Add python.exe to PATH」にチェックを入れると確実です）
echo   2. インストール後、もう一度 start.bat をダブルクリックしてください。
echo   ※ Microsoft Store の「python」案内が出る場合も、上記からインストールしてください。
echo.
pause
exit /b 1

:noport
echo.
echo 使えるポートが見つかりませんでした（8000〜8019 がすべて使用中）。
echo 他のアプリやウィンドウを閉じてから、もう一度お試しください。
pause
exit /b 2

:failed
echo.
echo サーバーが異常終了しました。上のメッセージを確認してください。
pause
exit /b 1
