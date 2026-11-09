#!/bin/sh
# BoneSim launcher (macOS / Linux): starts serve.py, which picks a free port
# (8000, 8001, ...) and opens the browser once the server is listening.
#   ./start.sh            local only
#   ./start.sh --lan      reachable from phones on the same Wi-Fi (camera needs HTTPS)
cd "$(dirname "$0")" || exit 1
for c in python3 python; do
  if command -v "$c" >/dev/null 2>&1 && "$c" -c 'import sys; sys.exit(0 if sys.version_info[0] == 3 else 1)' 2>/dev/null; then
    exec "$c" serve.py "$@"
  fi
done
echo "Python 3 が見つかりません。python3 をインストールしてください。"
exit 1
