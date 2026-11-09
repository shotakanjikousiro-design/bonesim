"""Headless browser smoke test for index.html (Chrome + Playwright).

    python3 tests/smoke_browser.py            # screenshots -> docs/screenshots/screenshot_*.png
    python3 tests/smoke_browser.py NAME ...   # only the named shots

Serves the project folder, opens index.html, fails on console errors / page
errors, drives window.bonesim to several poses and saves screenshots.
"""
import http.server
import json
import os
import socketserver
import sys
import threading
import time
from functools import partial

from playwright.sync_api import sync_playwright

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
DOCS = os.path.join(ROOT, 'docs', 'screenshots')
CHROME = '/usr/bin/google-chrome'


class Quiet(http.server.SimpleHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def handle(self):
        try:
            super().handle()
        except (ConnectionResetError, BrokenPipeError):
            pass


def serve():
    handler = partial(Quiet, directory=ROOT)
    httpd = socketserver.ThreadingTCPServer(('127.0.0.1', 0), handler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    return httpd, httpd.server_address[1]


def anat():
    limb = lambda: {
        'shoulder': {'elevation': 0, 'plane': 0, 'axialRotation': 0}, 'elbow': {'flexion': 0},
        'forearm': {'pronation': -90}, 'wrist': {'flexion': 0, 'ulnarDeviation': 0},
        'hip': {'flexion': 0, 'abduction': 0, 'axialRotation': 0}, 'knee': {'flexion': 0},
        'ankle': {'dorsiflexion': 0, 'inversion': 0}}
    return {'spine': {'flexion': 0, 'lateralBend': 0, 'axialRotation': 0},
            'neck': {'flexion': 0, 'lateralBend': 0, 'axialRotation': 0}, 'left': limb(), 'right': limb()}


def main():
    os.makedirs(DOCS, exist_ok=True)
    httpd, port = serve()
    url = f'http://127.0.0.1:{port}/index.html'
    errors = []
    only = sys.argv[1:]
    with sync_playwright() as p:
        browser = p.chromium.launch(executable_path=CHROME, headless=True, args=[
            '--use-gl=angle', '--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist',
            '--use-fake-device-for-media-stream', '--use-fake-ui-for-media-stream'])

        def open_page(w, h, mobile=False):
            ctx = browser.new_context(viewport={'width': w, 'height': h}, device_scale_factor=2 if mobile else 1,
                                      is_mobile=mobile, has_touch=mobile)
            page = ctx.new_page()
            page.on('console', lambda m: errors.append(f'console.{m.type}: {m.text}') if m.type == 'error' else None)
            page.on('pageerror', lambda e: errors.append(f'pageerror: {e}'))
            page.goto(url)
            page.wait_for_function('window.bonesim && window.bonesim.rig', timeout=60000)
            page.wait_for_timeout(500)
            return page

        def shot(page, name, pose=None, view=None, style='xray', extra=''):
            if only and name not in only:
                return
            js = f"""() => {{ const b = window.bonesim; b.setStyle('{style}');
                {f"b.sim.setPose({json.dumps(pose)});" if pose else ''}
                {f"b.setView('{view}', true);" if view else ''} {extra} }}"""
            page.evaluate(js)
            page.wait_for_timeout(700)
            path = os.path.join(DOCS, f'screenshot_{name}.png')
            page.screenshot(path=path)
            print('saved', path)

        page = open_page(1280, 800)
        shot(page, 'sim_rest', anat(), 'front')
        arms = "b.camera.position.set(0,1.05,2.1); b.controls.target.set(0,1.0,0.08); b.controls.update();"
        shot(page, 'sim_elbows_anatomical', anat(), None, style='bone', extra=arms)
        pe = anat(); pe['left']['elbow']['flexion'] = 90; pe['right']['elbow']['flexion'] = 90
        shot(page, 'sim_elbows_flex90_top', pe, None, style='bone', extra="b.camera.position.set(0,2.2,0.5); b.controls.target.set(0,1.0,0.25); b.controls.update();")
        pe2 = anat(); pe2['left']['elbow']['flexion'] = 140; pe2['right']['elbow']['flexion'] = 140
        shot(page, 'sim_elbows_flex140', pe2, None, style='bone', extra=arms)
        ps = anat(); ps['right']['shoulder'] = {'elevation': 150, 'plane': 0, 'axialRotation': -30}
        shot(page, 'sim_scapula_abd150_back', ps, None, style='bone', extra="b.camera.position.set(-0.45,1.35,-1.1); b.controls.target.set(-0.1,1.25,0.05); b.controls.update();")
        p1 = anat(); p1['right']['elbow']['flexion'] = 90; p1['right']['forearm']['pronation'] = 90
        p1['left']['elbow']['flexion'] = 90
        shot(page, 'sim_elbow90_pron90', p1, 'front', extra="b.camera.position.set(-0.25,1.25,1.0); b.controls.target.set(-0.15,1.0,0.2); b.controls.update();")
        p1b = anat(); p1b['right']['forearm']['pronation'] = 90; p1b['right']['elbow']['flexion'] = 10
        shot(page, 'sim_pronation_arm', p1b, 'arm')
        p1c = anat(); p1c['right']['elbow']['flexion'] = 10
        shot(page, 'sim_supination_arm', p1c, 'arm')
        p2 = anat(); p2['right']['shoulder'] = {'elevation': 150, 'plane': 0, 'axialRotation': 0}
        shot(page, 'sim_abduction150_back', p2, 'back')
        shot(page, 'sim_abduction150_front_bone', p2, 'front', style='bone')
        p3 = anat(); p3['right']['knee']['flexion'] = 60
        shot(page, 'sim_knee60', p3, 'knee', style='bone')
        p3b = anat(); p3b['right']['knee']['flexion'] = 110
        shot(page, 'sim_knee110_side', p3b, 'knee', style='xray')
        p3c = anat()
        shot(page, 'sim_knee0_side', p3c, 'knee', style='xray')
        lat = "b.camera.position.set(-0.55,0.36,0.05); b.controls.target.set(-0.07,0.36,0.05); b.controls.update();"
        for f in (0, 45, 90, 120):
            pk = anat(); pk['right']['knee']['flexion'] = f
            shot(page, f'sim_knee_lat_{f}', pk, None, style='bone', extra=lat)
        shot(page, 'sim_xray_front', anat(), 'front', style='xray', extra="b.sim.startPreset('abduction'); ")
        page.evaluate("() => window.bonesim.sim.stopPreset()")
        shot(page, 'sim_skin', p2, 'front', style='skin')
        pf = anat(); pf['right']['elbow']['flexion'] = 30; pf['right']['forearm']['pronation'] = -60
        pf['right']['fingers'] = {'thumb': {'cmcFlexion': 35, 'cmcAbduction': 25, 'mcp': 45, 'ip': 50},
                                  'index': {'mcp': 85, 'pip': 100, 'dip': 60}, 'middle': {'mcp': 88, 'pip': 100, 'dip': 62},
                                  'ring': {'mcp': 90, 'pip': 100, 'dip': 62}, 'little': {'mcp': 92, 'pip': 100, 'dip': 60}}
        shot(page, 'sim_fist', pf, 'arm', style='bone')
        # preset playing with teaching panel
        if not only or 'sim_preset_knee' in only:
            page.evaluate("() => { const b = window.bonesim; b.setStyle('xray'); b.sim.startPreset('knee'); }")
            page.wait_for_timeout(2500)
            page.screenshot(path=os.path.join(DOCS, 'screenshot_sim_preset_knee.png'))
            page.evaluate("() => window.bonesim.sim.stopPreset()")
        # pick a bone
        if not only or 'sim_pick' in only:
            page.evaluate("() => { const b = window.bonesim; b.sim.setPose({}); b.setView('front', true); b.showPick('radius_R'); }")
            page.wait_for_timeout(500)
            page.screenshot(path=os.path.join(DOCS, 'screenshot_sim_pick.png'))
        # camera mode with the synthetic demo source (no camera / no MediaPipe needed)
        if not only or 'cam' in only:
            page.evaluate("() => { window.bonesim.setMode('cam'); window.bonesim.setStyle('xray'); }")
            page.select_option('#camSource', 'demo')
            page.click('#camStart')
            page.wait_for_timeout(2500)
            page.screenshot(path=os.path.join(DOCS, 'screenshot_camera_demo.png'))
            page.click('#camSide3D')
            page.wait_for_timeout(1500)
            page.screenshot(path=os.path.join(DOCS, 'screenshot_camera_demo_side3d.png'))
            page.click('#camSide3D')
            page.click('#camMirror')
            page.evaluate("() => window.bonesim.setStyle('bone')")
            page.wait_for_timeout(1200)
            page.screenshot(path=os.path.join(DOCS, 'screenshot_camera_demo_mirror_bone.png'))
            page.click('#camMirror')
            print('camera status:', page.inner_text('#camStatus'), page.inner_text('#camFps'))
            # real webcam path with chrome's fake device (MediaPipe on a test pattern: no person expected)
            page.select_option('#camSource', 'camera')
            page.click('#camStart')
            page.wait_for_timeout(20000)
            page.screenshot(path=os.path.join(DOCS, 'screenshot_camera_fakecam.png'))
            print('fake camera status:', page.inner_text('#camStatus'), page.inner_text('#camFps'))
            page.evaluate("() => window.bonesim.setMode('sim')")
        page.context.close()

        # mobile
        if not only or 'mobile' in only:
            m = open_page(390, 844, mobile=True)
            m.evaluate("() => window.bonesim.sim.startPreset('pronation')")
            m.wait_for_timeout(1800)
            m.screenshot(path=os.path.join(DOCS, 'screenshot_mobile_sim.png'))
            m.evaluate("() => window.bonesim.setMode('cam')")
            m.select_option('#camSource', 'demo')
            m.click('#camStart')
            m.wait_for_timeout(1500)
            m.screenshot(path=os.path.join(DOCS, 'screenshot_mobile_cam.png'))
            print('saved mobile screenshots')
            m.context.close()
        browser.close()
    httpd.shutdown()
    # MediaPipe (fake-camera path) prints TFLite info via console.error and tries a telemetry POST
    # to odml.pa.googleapis.com that CORS blocks; both are outside this app's code.
    known = ('favicon', 'XNNPACK', 'odml.pa.googleapis.com', 'net::ERR_FAILED')
    benign = [e for e in errors if any(k in e for k in known)]
    for e in benign:
        print('benign', e[:140])
    real = [e for e in errors if e not in benign]
    for e in real:
        print('ERROR', e)
    print('SMOKE', 'FAIL' if real else 'PASS', f'({len(real)} console/page errors)')
    sys.exit(1 if real else 0)


if __name__ == '__main__':
    main()
