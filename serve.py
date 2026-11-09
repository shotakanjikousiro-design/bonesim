#!/usr/bin/env python3
"""BoneSim local web server (Python 3 standard library only).

    python serve.py              # http://localhost:8000/ (or the next free port), opens the browser
    python serve.py --lan        # also reachable from phones on the same Wi-Fi
    python serve.py --port 8080 --no-browser

* Forces correct MIME types (Windows can map .js to text/plain via the registry,
  which makes browsers refuse ES modules).
* Binds 127.0.0.1 by default (no firewall prompt); --lan binds 0.0.0.0.
* Picks the first free port starting at --port (default 8000).
* Opens the browser only after the server is listening.
"""
import argparse
import http.server
import os
import socket
import socketserver
import sys
import threading
import webbrowser

ROOT = os.path.dirname(os.path.abspath(__file__))

MIME = {
    '.html': 'text/html; charset=utf-8',
    '.js': 'text/javascript; charset=utf-8',
    '.mjs': 'text/javascript; charset=utf-8',
    '.css': 'text/css; charset=utf-8',
    '.json': 'application/json',
    '.wasm': 'application/wasm',
    '.glb': 'model/gltf-binary',
    '.gltf': 'model/gltf+json',
    '.task': 'application/octet-stream',
    '.png': 'image/png',
    '.svg': 'image/svg+xml',
    '.md': 'text/plain; charset=utf-8',
}


class Handler(http.server.SimpleHTTPRequestHandler):
    extensions_map = {**http.server.SimpleHTTPRequestHandler.extensions_map, **MIME}

    def __init__(self, *a, **kw):
        super().__init__(*a, directory=ROOT, **kw)

    def end_headers(self):
        self.send_header('Cache-Control', 'no-cache')
        super().end_headers()

    def log_message(self, fmt, *args):  # quiet: only errors
        if len(args) > 1 and str(args[1]).startswith(('4', '5')) and not self.path.endswith('favicon.ico'):
            sys.stderr.write('[%s] %s\n' % (self.log_date_time_string(), fmt % args))

    def handle(self):
        try:
            super().handle()
        except (ConnectionResetError, BrokenPipeError):
            pass


class Server(socketserver.ThreadingMixIn, http.server.HTTPServer):
    daemon_threads = True
    allow_reuse_address = False


def lan_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('192.0.2.1', 80))  # no packet is sent; picks the outgoing interface
        return s.getsockname()[0]
    except OSError:
        return None
    finally:
        s.close()


def main():
    for st in (sys.stdout, sys.stderr):
        try:
            st.reconfigure(errors='replace')
        except Exception:
            pass
    ap = argparse.ArgumentParser(description='BoneSim local server')
    ap.add_argument('--port', type=int, default=8000)
    ap.add_argument('--lan', action='store_true', help='bind 0.0.0.0 (phones on the same Wi-Fi)')
    ap.add_argument('--no-browser', action='store_true')
    a = ap.parse_args()
    host = '0.0.0.0' if a.lan else '127.0.0.1'
    httpd = None
    for port in range(a.port, a.port + 20):
        try:
            httpd = Server((host, port), Handler)
            break
        except OSError:
            print(f'ポート {port} は使用中です。次のポートを試します…')
    if httpd is None:
        print(f'空いているポートが見つかりませんでした（{a.port}〜{a.port + 19}）。他のアプリを閉じてから再実行してください。')
        sys.exit(2)
    port = httpd.server_address[1]
    url = f'http://localhost:{port}/'
    print('=' * 60)
    print(' BoneSim 骨透かしシミュレーター')
    print(f' PC のブラウザで開く: {url}')
    if a.lan:
        ip = lan_ip()
        if ip:
            print(f' 同じWi-Fiのスマホから: http://{ip}:{port}/')
        print(' ※ スマホのカメラは HTTPS が必要なため、この方法ではシミュレーターのみ使えます。')
    print(' 終了するにはこのウィンドウを閉じるか Ctrl+C を押してください。')
    print('=' * 60)
    sys.stdout.flush()
    if not a.no_browser:
        threading.Timer(0.3, lambda: webbrowser.open(url)).start()
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print('\n終了しました。')
    finally:
        httpd.server_close()


if __name__ == '__main__':
    main()
