"""Generate src/tracking/faceModel.js and add the temporomandibular joint to assets/rig.json.

    python3 tools/build_face_model.py

* Face anchors: the canonical face mesh that ships inside MediaPipe's
  face_landmarker.task (geometry_pipeline_metadata_landmarks.binarypb, 468
  vertices, cm) is registered to this skeleton's skull by a similarity fit
  (soft-tissue nasion, chin, eye centres, nose tip).  The result gives, for
  a set of expression-stable FaceLandmarker indices, their position relative
  to the atlanto-occipital joint (the rig's skull pivot).
* TMJ: the mandibular condyles are located on the mandible mesh
  (top of the posterior ramus process on each side).
"""
import json
import os
import struct
import zipfile

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
TASK = os.path.join(ROOT, 'vendor/mediapipe/models/face_landmarker.task')
RIG = os.path.join(ROOT, 'assets/rig.json')
GLB = os.path.join(ROOT, 'assets/skeleton.glb')
OUT = os.path.join(ROOT, 'src/tracking/faceModel.js')

# expression-stable landmarks (forehead, temples, eye corners, nose bridge / tip / base, cheekbones)
STABLE = [10, 67, 297, 109, 338, 151, 9, 8, 168, 6, 197, 195, 5, 4, 1, 2, 98, 327,
          33, 133, 362, 263, 159, 386, 145, 374, 116, 345, 50, 280, 127, 356, 234, 454, 21, 251]


def varint(b, i):
    r = s = 0
    while True:
        c = b[i]; i += 1; r |= (c & 127) << s; s += 7
        if c < 128:
            return r, i


def fields(b):
    i = 0
    while i < len(b):
        k, i = varint(b, i); f, w = k >> 3, k & 7
        if w == 0: v, i = varint(b, i)
        elif w == 2: l, i = varint(b, i); v = b[i:i + l]; i += l
        elif w == 5: v = b[i:i + 4]; i += 4
        elif w == 1: v = b[i:i + 8]; i += 8
        else: raise ValueError(w)
        yield f, w, v


def canonical_mesh():
    z = zipfile.ZipFile(TASK)
    b = z.read('geometry_pipeline_metadata_landmarks.binarypb')
    mesh = [v for f, w, v in fields(b) if f == 1 and w == 2][0]
    vals = [struct.unpack('<f', v)[0] for f, w, v in fields(mesh) if f == 3 and w == 5]
    return np.array(vals).reshape(-1, 5)[:, :3] / 100.0  # metres; +X subject's left, +Y up, +Z out of the face


def umeyama(P, Q):
    cp, cq = P.mean(0), Q.mean(0); A, B = P - cp, Q - cq
    U, S, Vt = np.linalg.svd(B.T @ A); D = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))])
    R = U @ D @ Vt; s = (S * np.diag(D)).sum() / (A ** 2).sum()
    return s, R, cq - s * R @ cp


def glb_mesh(name):
    g = open(GLB, 'rb').read()
    jl = struct.unpack('<I', g[12:16])[0]; J = json.loads(g[20:20 + jl]); bin0 = g[20 + jl + 8:]
    nd = [n for n in J['nodes'] if n['name'] == name][0]
    out = []
    for p in J['meshes'][nd['mesh']]['primitives']:
        a = J['accessors'][p['attributes']['POSITION']]; bv = J['bufferViews'][a['bufferView']]
        off = bv.get('byteOffset', 0) + a.get('byteOffset', 0); st = bv.get('byteStride', 12)
        out.append(np.array([struct.unpack_from('<3f', bin0, off + k * st) for k in range(a['count'])]))
    return np.vstack(out)


def main():
    R = json.load(open(RIG, encoding='utf-8'))
    AO = np.array(R['joints']['atlantooccipital']['center'])
    L = {k: np.array(v) - AO for k, v in R['landmarks'].items()}
    V = canonical_mesh()
    # soft-tissue targets on this skull (relative to AO): nasion +6 mm, chin = menton 7 mm down / 9 mm forward,
    # eye centres and nose tip as in closeup.js HEAD_MODEL
    eyeL, eyeR, nose = [0.0390, 0.0404, 0.0915], [-0.0250, 0.0404, 0.0915], [0.0068, 0.0067, 0.1294]
    src = np.array([V[168], V[152], (V[362] + V[263]) / 2, (V[133] + V[33]) / 2, V[1]])
    dst = np.array([L['nasion'] + [0, 0, 0.006], L['menton'] + [0, -0.007, 0.009], eyeL, eyeR, nose])
    s, Rm, t = umeyama(src, dst)
    res = np.linalg.norm((s * (Rm @ src.T).T + t) - dst, axis=1) * 1000
    print(f'canonical face -> skull: scale {s:.3f}, residuals {np.round(res, 1)} mm')
    anchors = {i: (s * Rm @ V[i] + t) for i in STABLE}

    M = glb_mesh('mandible'); cx = M[:, 0].mean(); cond = {}
    for side, sel in (('L', M[:, 0] > cx + 0.03), ('R', M[:, 0] < cx - 0.03)):
        P = M[sel]; top = P[P[:, 1] > P[:, 1].max() - 0.03]
        zs = np.sort(top[:, 2]); k = np.diff(zs).argmax(); zsplit = (zs[k] + zs[k + 1]) / 2  # coronoid | condyle
        post = P[P[:, 2] < zsplit]; head = post[post[:, 1] > post[:, 1].max() - 0.008]
        c = head.mean(0); c[1] = post[:, 1].max() - 0.005
        cond[side] = c
    centre = (cond['L'] + cond['R']) / 2; axis = cond['L'] - cond['R']; axis /= np.linalg.norm(axis)
    R['joints']['temporomandibular'] = {
        'center': [round(float(x), 5) for x in centre], 'axes': {'opening': [round(float(x), 5) for x in axis]},
        'condyle_L': [round(float(x), 5) for x in cond['L']], 'condyle_R': [round(float(x), 5) for x in cond['R']],
        'method': 'condylar heads = top of the posterior ramus process of the mandible mesh; axis R→L condyle',
    }
    json.dump(R, open(RIG, 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
    print('TMJ', R['joints']['temporomandibular'])

    rows = ',\n'.join(f'  {i}: [{a[0]:.4f}, {a[1]:.4f}, {a[2]:.4f}]' for i, a in anchors.items())
    rm = ', '.join(f'{x:.5f}' for x in Rm.flatten())
    js = f'''// GENERATED by tools/build_face_model.py — do not edit by hand.
// MediaPipe FaceLandmarker landmark index → position relative to the skull's
// atlanto-occipital joint (metres, BoneSim axes: +X subject's left, +Y up, +Z forward).
// Source: canonical face mesh inside face_landmarker.task, registered to this skeleton's
// skull (similarity fit, scale {s:.3f}: the BodyParts3D skull is smaller than the average face
// mesh, so the anchors are the face "as worn by this skull" — the skull then fills the face
// on screen and the estimated depth is FACE_DEPTH_SCALE × the true distance).
export const FACE_DEPTH_SCALE = {s:.4f};
/** rotation canonical-face frame → skull frame (row-major 3×3) */
export const FACE_TO_SKULL = [{rm}];
export const FACE_ANCHORS = {{
{rows},
}};
'''
    open(OUT, 'w', encoding='utf-8').write(js)
    print('wrote', OUT, len(anchors), 'anchors')


if __name__ == '__main__':
    main()
