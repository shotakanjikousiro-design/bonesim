"""Build the hosted-page bundle (dist_artifact/) for publishing as a Claude page.

The hosting service only serves text files, so every binary asset
(.glb, .task, .wasm) is written as `<name>.b64.js` (a JS module exporting the
file as base64). src/shared/binAsset.js falls back to these automatically.

    python tools/build_artifact.py
"""
import base64, os, shutil

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, 'dist_artifact')
INCLUDE = ['index.html', 'src', 'vendor', 'assets']
BINARY = ('.glb', '.task', '.wasm')
SKIP = ('.d.ts',)

shutil.rmtree(OUT, ignore_errors=True)
total = 0
for item in INCLUDE:
    src = os.path.join(ROOT, item)
    paths = [src] if os.path.isfile(src) else [os.path.join(r, f) for r, _, fs in os.walk(src) for f in fs]
    for p in sorted(paths):
        rel = os.path.relpath(p, ROOT)
        if rel.endswith(SKIP) or '__pycache__' in rel:
            continue
        dst = os.path.join(OUT, rel)
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        if rel.endswith(BINARY):
            dst += '.b64.js'
            with open(p, 'rb') as f:
                data = base64.b64encode(f.read()).decode('ascii')
            LIMIT = 12_000_000  # hosted text files must be < 16 MiB; split big ones
            if len(data) <= LIMIT:
                with open(dst, 'w') as f:
                    f.write('export default "' + data + '";\n')
            else:
                base = os.path.basename(dst)[:-3]  # strip '.js'
                parts = [data[i:i + LIMIT] for i in range(0, len(data), LIMIT)]  # LIMIT % 4 == 0
                names = []
                for k, chunk in enumerate(parts):
                    name = f'{base}.{k}.js'; names.append(name)
                    with open(os.path.join(os.path.dirname(dst), name), 'w') as f:
                        f.write('export default "' + chunk + '";\n')
                    print(f'{len(chunk)/1e6:8.2f} MB  {os.path.relpath(os.path.join(os.path.dirname(dst), name), OUT)}')
                    total += len(chunk)
                with open(dst, 'w') as f:
                    f.write(''.join(f'import p{k} from "./{n}";\n' for k, n in enumerate(names)))
                    f.write('export default ' + ' + '.join(f'p{k}' for k in range(len(names))) + ';\n')
        else:
            shutil.copy2(p, dst)
        size = os.path.getsize(dst)
        total += size
        print(f'{size/1e6:8.2f} MB  {os.path.relpath(dst, OUT)}')
print(f'total {total/1e6:.1f} MB')
