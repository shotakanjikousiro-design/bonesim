"""Run the REAL MediaPipe models (headless Chrome, fake webcam) on a still photo and
screenshot the camera-mode overlay.

    python3 tests/closeup_photo.py PHOTO.jpg [OUT.png] [--bone] [--undim]

--undim: the photo is an app screenshot taken with 「映像を暗くする」 (CSS brightness 0.45):
brighten it back ≈ ×2.2 so the detector sees roughly what the camera delivered.

The photo is converted to a looping Y4M "webcam" (ffmpeg) and fed to Chrome with
--use-file-for-fake-video-capture; the app's camera mode then runs end to end
(tracker.js → pipeline.js → rig). Mirror display is switched off so the result can be
compared with the photo directly.
"""
import os
import subprocess
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import smoke_browser as sb  # noqa: E402
from playwright.sync_api import sync_playwright  # noqa: E402


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    if not args:
        print(__doc__)
        sys.exit(2)
    photo = os.path.abspath(args[0])
    out = os.path.abspath(args[1]) if len(args) > 1 else os.path.join(sb.DOCS, 'screenshot_closeup_fix.png')
    style = 'bone' if '--bone' in sys.argv else 'xray'
    tmp = tempfile.mkdtemp()
    y4m = os.path.join(tmp, 'cam.y4m')
    if '--undim' in sys.argv:
        from PIL import Image, ImageEnhance
        src = os.path.join(tmp, 'undim.png')
        ImageEnhance.Brightness(Image.open(photo).convert('RGB')).enhance(2.2).save(src)
        photo = src
    subprocess.run(['ffmpeg', '-loglevel', 'error', '-y', '-loop', '1', '-i', photo, '-t', '4', '-r', '15',
                    '-vf', 'scale=trunc(iw/2)*2:trunc(ih/2)*2', '-pix_fmt', 'yuv420p', y4m], check=True)
    httpd, port = sb.serve()
    errors = []
    with sync_playwright() as p:
        browser = p.chromium.launch(executable_path=sb.CHROME, headless=True, args=[
            '--use-gl=angle', '--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist',
            '--use-fake-device-for-media-stream', '--use-fake-ui-for-media-stream', f'--use-file-for-fake-video-capture={y4m}'])
        page = browser.new_page(viewport={'width': 1100, 'height': 900})
        page.on('pageerror', lambda e: errors.append(str(e)))
        page.goto(f'http://127.0.0.1:{port}/index.html')
        page.wait_for_function('window.bonesim && window.bonesim.rig', timeout=60000)
        page.evaluate(f"() => {{ window.bonesim.setMode('cam'); window.bonesim.setStyle('{style}'); }}")
        page.select_option('#camSource', 'camera')
        if page.is_checked('#camMirror'):
            page.click('#camMirror')
        page.select_option('#camModel', 'full')
        page.click('#camStart')
        page.wait_for_timeout(40000)
        status = page.inner_text('#camStatus')
        info = page.evaluate("""() => { const r = window.bonesim.cam.last; if (!r) return null;
            return { framing: r.framing, framingRaw: r.framingRaw, head: r.head && { ao: r.head.ao, rmsPx: r.head.rmsPx, n: r.head.n },
                     nFit: r.alignment && r.alignment.nFit, fovY: r.camera && r.camera.fovY, dims: r.dimsSource,
                     size: r.landmarks2D && [r.landmarks2D.width, r.landmarks2D.height] }; }""")
        page.screenshot(path=out)
        browser.close()
    httpd.shutdown()
    print('status:', status)
    print('result:', info)
    print('saved', out)
    for e in errors:
        print('pageerror', e)


if __name__ == '__main__':
    main()
