#!/usr/bin/env python3
"""Render verification images of assets/skeleton.glb + rig.json (matplotlib, painter's algorithm).

python3 tools/render_check.py  ->  docs/asset_check_{front,side,elbow,knee,hand,shoulder,ankle}.png
Joint centres = coloured dots; each axis = line from the centre in its POSITIVE direction.
"""
import json
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.collections import PolyCollection  # noqa: E402

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")


def load_glb(path):
    import pygltflib as g
    gl = g.GLTF2().load_binary(path)
    blob = gl.binary_blob()
    out = {}
    for node in gl.nodes:
        prim = gl.meshes[node.mesh].primitives[0]

        def acc(i):
            a = gl.accessors[i]
            bv = gl.bufferViews[a.bufferView]
            dt = {5126: "<f4", 5123: "<u2", 5125: "<u4"}[a.componentType]
            n = {"VEC3": 3, "SCALAR": 1}[a.type]
            arr = np.frombuffer(blob, dt, a.count * n, bv.byteOffset + (a.byteOffset or 0))
            return arr.reshape(-1, n) if n > 1 else arr
        out[node.name] = (acc(prim.attributes.POSITION).astype(float),
                          acc(prim.indices).reshape(-1, 3).astype(int),
                          node.extras)
    return out


AX_COL = {"flexion": "tab:red", "pronation": "tab:green", "dorsiflexion": "tab:red", "inversion": "tab:purple",
          "abduction": "tab:blue", "internalRotation": "tab:orange", "axialRotation": "tab:orange",
          "ulnarDeviation": "tab:blue", "lateralBend": "tab:blue", "elevation": "tab:blue",
          "protraction": "tab:cyan", "posteriorRotation": "tab:olive", "upwardRotation": "tab:blue",
          "posteriorTilt": "tab:olive"}


def render(meshes, rig, view, fname, box=None, axlen=0.05, title="", skin=False, landmarks=True, dpi=110):
    """view: 'front' (from +Z), 'right' (from subject's right, -X), 'left' (from +X), 'top'."""
    if view == "front":
        R = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])   # screen x=X, y=Y, depth=Z
    elif view == "right":
        R = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])  # screen x=Z (anterior right), depth=-X
    elif view == "left":
        R = np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0]])  # screen x=-Z, depth=+X
    elif view == "back":
        R = np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]])
    light = np.array([0.3, 0.5, 1.0]) / np.linalg.norm([0.3, 0.5, 1.0])
    polys, cols, depth = [], [], []
    for name, (V, F, ex) in meshes.items():
        if name == "skin" and not skin:
            continue
        P = V @ R.T
        if box is not None:
            lo, hi = box
            inb = np.all((V >= lo) & (V <= hi), axis=1)
            Fm = F[inb[F].any(1)]
        else:
            Fm = F
        if len(Fm) == 0:
            continue
        T = P[Fm]
        n = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
        nn = np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
        n = n / nn
        front = n[:, 2] > 0
        T, n = T[front], n[front]
        shade = 0.35 + 0.65 * np.clip(n @ light, 0, 1)
        base = np.array([0.82, 0.82, 0.80]) if name != "skin" else np.array([0.9, 0.75, 0.7])
        polys.append(T[:, :, :2])
        cols.append(np.c_[np.outer(shade, base), np.full(len(shade), 1.0 if name != "skin" else 0.15)])
        depth.append(T[:, :, 2].mean(1))
    polys = np.concatenate(polys)
    cols = np.concatenate(cols)
    depth = np.concatenate(depth)
    o = np.argsort(depth)
    fig, ax = plt.subplots(figsize=(9, 11) if box is None else (9, 9))
    ax.add_collection(PolyCollection(polys[o], facecolors=cols[o], edgecolors="none"))
    pts = polys.reshape(-1, 2)
    ax.set_xlim(pts[:, 0].min() - 0.02, pts[:, 0].max() + 0.02)
    ax.set_ylim(pts[:, 1].min() - 0.02, pts[:, 1].max() + 0.02)
    ax.set_aspect("equal")
    lo, hi = (box if box is not None else (np.full(3, -9), np.full(3, 9)))
    used = set()
    for jn, j in rig["joints"].items():
        c = np.array(j["center"])
        if np.any(c < lo) or np.any(c > hi):
            continue
        cs = c @ R.T
        ax.plot(cs[0], cs[1], "o", color="k", ms=4, zorder=5)
        if box is not None:
            ax.annotate(jn, cs[:2], fontsize=7, color="k", xytext=(3, 3), textcoords="offset points", zorder=6)
        for an, a in j["axes"].items():
            if box is None and an not in ("flexion", "pronation", "dorsiflexion", "inversion"):
                continue
            e = (c + axlen * np.array(a)) @ R.T
            col = AX_COL.get(an, "m")
            ax.plot([cs[0], e[0]], [cs[1], e[1]], "-", color=col, lw=2, zorder=5,
                    label=an if an not in used else None)
            ax.plot(e[0], e[1], ">", color=col, ms=3, zorder=5)
            used.add(an)
        for extra in ("proximal", "distal", "medialCondyleCenter", "lateralCondyleCenter",
                      "medialMalleolusTip", "lateralMalleolusTip", "patellaApex", "tibialTuberosity"):
            if extra in j:
                p = np.array(j[extra]) @ R.T
                ax.plot(p[0], p[1], "s", color="tab:green", ms=3, zorder=5)
    if landmarks:
        for ln, p in rig["landmarks"].items():
            p = np.array(p)
            if np.any(p < lo) or np.any(p > hi):
                continue
            q = p @ R.T
            ax.plot(q[0], q[1], "x", color="tab:blue", ms=4, zorder=5)
            if box is not None:
                ax.annotate(ln, q[:2], fontsize=6, color="tab:blue", xytext=(3, -8), textcoords="offset points")
    ax.legend(loc="lower right", fontsize=7)
    ax.set_title(title, fontsize=10)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(fname, dpi=dpi)
    plt.close(fig)


def main():
    meshes = load_glb(os.path.join(ROOT, "assets", "skeleton.glb"))
    rig = json.load(open(os.path.join(ROOT, "assets", "rig.json"), encoding="utf-8"))
    D = os.path.join(ROOT, "docs")
    os.makedirs(D, exist_ok=True)
    only = sys.argv[1:]

    def want(n):
        return not only or n in only
    if want("front"):
        render(meshes, rig, "front", f"{D}/asset_check_front.png", skin=True,
               title="Front (viewer facing subject; subject's LEFT = +X = screen right). "
                     "Dots = joint centres, lines = +axis")
    if want("side"):
        render(meshes, rig, "right", f"{D}/asset_check_side.png", skin=True,
               title="Side view from subject's RIGHT (anterior +Z = screen right)")
    L = lambda c, r: (np.array(c) - r, np.array(c) + r)  # noqa: E731
    j = rig["joints"]
    if want("elbow"):
        c = np.array(j["humeroulnar_L"]["center"])
        render(meshes, rig, "front", f"{D}/asset_check_elbow_front.png", box=L(c + [0, -0.1, 0], [0.09, 0.2, 0.12]),
               axlen=0.04, title="LEFT elbow/forearm, front view")
        render(meshes, rig, "left", f"{D}/asset_check_elbow_side.png", box=L(c + [0, -0.1, 0], [0.09, 0.2, 0.12]),
               axlen=0.04, title="LEFT elbow/forearm from subject's left (anterior = screen LEFT)")
    if want("knee"):
        c = np.array(j["knee_L"]["center"])
        render(meshes, rig, "front", f"{D}/asset_check_knee_front.png", box=L(c, [0.08, 0.1, 0.1]), axlen=0.04,
               title="LEFT knee, front view")
        render(meshes, rig, "left", f"{D}/asset_check_knee_side.png", box=L(c, [0.08, 0.1, 0.1]), axlen=0.04,
               title="LEFT knee from subject's left (anterior = screen LEFT)")
    if want("hand"):
        c = np.array(j["midcarpal_R"]["center"])
        render(meshes, rig, "front", f"{D}/asset_check_hand.png", box=L(c + [0, -0.06, 0], [0.08, 0.13, 0.1]),
               axlen=0.02, title="RIGHT wrist/hand, front view")
    if want("shoulder"):
        c = np.array(j["glenohumeral_L"]["center"])
        render(meshes, rig, "front", f"{D}/asset_check_shoulder.png", box=L(c + [-0.06, 0, 0], [0.12, 0.12, 0.15]),
               axlen=0.04, title="LEFT shoulder girdle, front view")
    if want("ankle"):
        c = np.array(j["talocrural_L"]["center"])
        render(meshes, rig, "left", f"{D}/asset_check_ankle.png", box=L(c + [0, -0.02, 0.05], [0.07, 0.08, 0.15]),
               axlen=0.04, title="LEFT ankle/foot from subject's left (anterior = screen LEFT)")
    if want("back"):
        render(meshes, rig, "back", f"{D}/asset_check_back.png", box=(np.array([-0.25, 1.1, -0.1]), np.array([0.25, 1.52, 0.2])),
               axlen=0.03, title="Shoulder girdle / neck from BEHIND (subject's left = screen left)")
        render(meshes, rig, "front", f"{D}/asset_check_pelvis.png", box=(np.array([-0.2, 0.7, -0.1]), np.array([0.2, 1.0, 0.25])),
               axlen=0.03, title="Pelvis, front view")
    if want("spine"):
        render(meshes, rig, "right", f"{D}/asset_check_spine.png", box=(np.array([-0.1, 0.8, -0.1]), np.array([0.1, 1.65, 0.25])),
               axlen=0.03, title="Spine from subject's right (anterior = screen right)")


if __name__ == "__main__":
    main()
