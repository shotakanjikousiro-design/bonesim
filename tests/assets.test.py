#!/usr/bin/env python3
"""Checks for assets/skeleton.glb + assets/rig.json  (python3 tests/assets.test.py; needs numpy only).

- every rig node / segment node exists in the GLB, meshes are valid, GLB ≤ 8 MB
- segment tree rooted at pelvis, all PLAN joint IDs present, unit axes
- joint centres lie near the expected bones
- left/right symmetry
- frame orientation (+X left, +Y up, +Z anterior)
- positive rotation about each axis gives the named anatomical motion
"""
import json
import os
import re
import struct
import sys

import numpy as np

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
GLB = os.path.join(ROOT, "assets", "skeleton.glb")
RIG = os.path.join(ROOT, "assets", "rig.json")
CONV = os.path.join(ROOT, "src", "shared", "conventions.js")

FAILS, PASSES = [], 0


def check(cond, msg):
    global PASSES
    if cond:
        PASSES += 1
    else:
        FAILS.append(msg)
        print("FAIL:", msg)


def load_glb(path):
    data = open(path, "rb").read()
    magic, ver, length = struct.unpack_from("<4sII", data, 0)
    assert magic == b"glTF" and ver == 2 and length == len(data)
    off, js, bin_ = 12, None, None
    while off < length:
        clen, ctype = struct.unpack_from("<II", data, off)
        chunk = data[off + 8: off + 8 + clen]
        if ctype == 0x4E4F534A:
            js = json.loads(chunk)
        elif ctype == 0x004E4942:
            bin_ = chunk
        off += 8 + clen
    out = {}
    for node in js["nodes"]:
        prim = js["meshes"][node["mesh"]]["primitives"][0]

        def acc(i):
            a = js["accessors"][i]
            bv = js["bufferViews"][a["bufferView"]]
            dt = {5126: "<f4", 5123: "<u2", 5125: "<u4"}[a["componentType"]]
            n = {"VEC3": 3, "SCALAR": 1}[a["type"]]
            arr = np.frombuffer(bin_, dt, a["count"] * n, bv.get("byteOffset", 0) + a.get("byteOffset", 0))
            return arr.reshape(-1, n) if n > 1 else arr
        out[node["name"]] = dict(V=acc(prim["attributes"]["POSITION"]).astype(float),
                                 N=acc(prim["attributes"]["NORMAL"]).astype(float),
                                 F=acc(prim["indices"]).reshape(-1, 3).astype(np.int64))
    return out


def rot(P, c, axis, deg):
    """Rodrigues rotation (right-hand rule) of point(s) P about the line (c, axis)."""
    k = np.asarray(axis, float)
    k = k / np.linalg.norm(k)
    t = np.radians(deg)
    P = np.asarray(P, float)
    Q = np.atleast_2d(P) - c
    R = Q * np.cos(t) + np.cross(k, Q) * np.sin(t) + np.outer(Q @ k, k) * (1 - np.cos(t)) + c
    return R.reshape(P.shape)


def main():
    rig = json.load(open(RIG, encoding="utf-8"))
    G = load_glb(GLB)
    J, LM, SEG, NODES = rig["joints"], rig["landmarks"], rig["segments"], rig["nodes"]
    a = lambda v: np.array(v, float)  # noqa: E731

    # ---------- structure ----------
    size = os.path.getsize(GLB)
    check(size <= 8.5e6, f"GLB size {size/1e6:.2f} MB > 8.5 MB")
    check(rig["units"] == "m" and rig["frame"] == {"left": "+X", "up": "+Y", "forward": "+Z"}, "frame header")
    for n in NODES:
        check(n in G, f"rig node {n} missing from GLB")
    for n, m in G.items():
        check(n in NODES, f"GLB node {n} not in rig.nodes")
        check(len(m["V"]) == len(m["N"]) and m["F"].max() < len(m["V"]) and len(m["F"]) > 10, f"mesh {n} invalid")
        nl = np.linalg.norm(m["N"], axis=1)
        check(np.all(np.abs(nl - 1) < 1e-2), f"normals of {n} not unit")
        if n != "skin":
            check(NODES[n]["segment"] in SEG and n in SEG[NODES[n]["segment"]]["nodes"], f"node {n} segment link")
            check(bool(re.search(r"[぀-ヿ一-鿿]", NODES[n]["nameJa"])), f"nameJa of {n}")
            for f in ("nameEn", "fma", "group"):
                check(f in NODES[n], f"node {n} lacks {f}")
    check("skin" in G, "skin node present")
    for s, d in SEG.items():
        for n in d["nodes"]:
            check(n in G, f"segment {s} node {n} missing in GLB")
        if d["parent"] is None:
            check(s == "pelvis" and d["joint"] is None, "only pelvis is root")
        else:
            check(d["parent"] in SEG, f"parent of {s}")
            check(d["joint"] in J, f"joint of {s}")
        # reach root
        cur, hops = s, 0
        while SEG[cur]["parent"] is not None and hops < 40:
            cur, hops = SEG[cur]["parent"], hops + 1
        check(cur == "pelvis", f"segment {s} does not reach pelvis")
    expected_segments = ["pelvis", "L5", "L4", "L3", "L2", "L1", "thorax", "C7", "C6", "C5", "C4", "C3", "C2", "C1", "skull"]
    lim = ["clavicle", "scapula", "humerus", "ulna", "radius", "carpalsProximal", "hand", "thumbMeta", "thumbProx",
           "thumbDist"] + [f"{f}{p}" for f in ("index", "middle", "ring", "little") for p in ("Prox", "Mid", "Dist")] + \
          ["femur", "patella", "tibia", "talus", "foot"]
    expected_segments += [f"{x}_{k}" for x in lim for k in "LR"]
    check(set(expected_segments) == set(SEG), f"segment ids differ: {set(expected_segments) ^ set(SEG)}")
    expected_joints = ["lumbosacral", "L1L2", "L2L3", "L3L4", "L4L5", "T12L1", "C7T1", "C1C2", "C2C3", "C3C4",
                       "C4C5", "C5C6", "C6C7", "atlantooccipital"]
    per = ["sternoclavicular", "acromioclavicular", "glenohumeral", "humeroulnar", "radioulnar", "radiocarpal",
           "midcarpal", "cmc1", "mcp1", "mcp2", "mcp3", "mcp4", "mcp5", "pip2", "pip3", "pip4", "pip5", "dip2", "dip3",
           "dip4", "dip5", "ip1", "hip", "knee", "patellofemoral", "talocrural", "subtalar"]
    expected_joints += [f"{x}_{k}" for x in per for k in "LR"]
    for j in expected_joints:
        check(j in J, f"joint {j} missing")
    for jn, j in J.items():
        check(len(j["center"]) == 3 and "method" in j and len(j["axes"]) >= 1, f"joint {jn} fields")
        for an, v in j["axes"].items():
            check(abs(np.linalg.norm(v) - 1) < 1e-3, f"axis {jn}.{an} not unit")
    # dims keys == DEFAULT_DIMS keys
    src = open(CONV, encoding="utf-8").read()
    block = re.search(r"DEFAULT_DIMS\s*=\s*\{(.*?)\};", src, re.S).group(1)
    keys = set(re.findall(r"(\w+)\s*:", block))
    check(keys == set(rig["dims"]), f"dims keys differ: {keys ^ set(rig['dims'])}")
    for k, v in rig["dims"].items():
        check(0.02 < v < 0.8, f"dims {k}={v} implausible")

    # ---------- frame orientation ----------
    allV = np.vstack([m["V"] for n, m in G.items() if n != "skin"])
    check(np.argmax(allV[:, 1]) in range(0, len(G["skull"]["V"])) or G["skull"]["V"][:, 1].max() >= allV[:, 1].max() - 1e-6,
          "skull top is the highest point")
    check(G["toePhalanges_L"]["V"][:, 2].max() > G["calcaneus_L"]["V"][:, 2].max(), "toes anterior (+Z) of heel")
    check(G["humerus_L"]["V"][:, 0].mean() > 0.1 and G["humerus_R"]["V"][:, 0].mean() < -0.1, "left humerus at +X")
    check(G["sternumBody"]["V"][:, 2].mean() > G["T6"]["V"][:, 2].mean() + 0.08, "sternum anterior to spine")
    check(abs(allV[:, 1].min()) < 0.02, "feet on floor y≈0")
    for n in NODES:
        if n.endswith("_L"):
            check(G[n]["V"][:, 0].mean() > 0, f"{n} on +X")
        if n.endswith("_R"):
            check(G[n]["V"][:, 0].mean() < 0, f"{n} on -X")
    # palms forward (anatomical): radius lateral to ulna distally
    for k, s in (("L", 1), ("R", -1)):
        check(s * (a(LM[f"radialStyloid_{k}"])[0] - a(LM[f"ulnarStyloid_{k}"])[0]) > 0.02, f"forearm supinated {k}")

    # ---------- joint centres near bones ----------
    def dist_to(nodes, p):
        V = np.vstack([G[n]["V"] for n in nodes])
        return np.linalg.norm(V - p, axis=1).min()

    for k in "LR":
        gh = a(J[f"glenohumeral_{k}"]["center"])
        d = dist_to([f"humerus_{k}"], gh)
        check(0.012 < d < 0.03, f"GH {k}: {d*1000:.1f} mm from humeral head surface (expect ≈ radius)")
        hp = a(J[f"hip_{k}"]["center"])
        d = dist_to([f"femur_{k}"], hp)
        check(0.015 < d < 0.03, f"hip {k}: {d*1000:.1f} mm from femoral head surface")
        d = dist_to([f"hipBone_{k}"], hp)
        check(d < 0.035, f"hip {k} centre {d*1000:.1f} mm from acetabulum")
        check(dist_to([f"humerus_{k}"], a(J[f"humeroulnar_{k}"]["center"])) < 0.015, f"elbow centre inside distal humerus {k}")
        check(dist_to([f"radius_{k}"], a(J[f"radioulnar_{k}"]["proximal"])) < 0.015, f"radial head centre {k}")
        check(dist_to([f"ulna_{k}"], a(J[f"radioulnar_{k}"]["distal"])) < 0.012, f"ulnar head centre {k}")
        check(dist_to([f"femur_{k}"], a(J[f"knee_{k}"]["center"])) < 0.025, f"knee centre in distal femur {k}")
        check(dist_to([f"talus_{k}"], a(J[f"talocrural_{k}"]["center"])) < 0.015, f"ankle centre in talus {k}")
    seg_nodes = lambda s: SEG[s]["nodes"]  # noqa: E731
    for s, d in SEG.items():
        if d["joint"] is None:
            continue
        c = a(J[d["joint"]]["center"])
        dd = min(dist_to(seg_nodes(s), c), dist_to(seg_nodes(d["parent"]), c))
        check(dd < 0.03, f"joint {d['joint']} centre {dd*1000:.1f} mm from its bones")

    # ---------- symmetry ----------
    M = np.diag([-1, 1, 1])
    for jn, j in J.items():
        if jn.endswith("_L"):
            r = J[jn[:-2] + "_R"]
            dd = np.linalg.norm(M @ a(j["center"]) - a(r["center"]))
            check(dd < 0.02, f"symmetry {jn}: {dd*1000:.1f} mm")
            for an, v in j["axes"].items():
                # mirrored axis of a proper rotation axis is -M·v (pseudo-vector) for side-dependent signs;
                # compare up to sign
                c = abs(float((M @ a(v)) @ a(r["axes"][an])))
                check(c > 0.95, f"axis symmetry {jn}.{an}: |cos|={c:.3f}")
        elif not jn.endswith("_R"):
            check(abs(j["center"][0]) < 0.01, f"midline joint {jn} x={j['center'][0]:.3f}")
    for ln, p in LM.items():
        if ln.endswith("_L"):
            dd = np.linalg.norm(M @ a(p) - a(LM[ln[:-2] + "_R"]))
            check(dd < 0.02, f"landmark symmetry {ln}: {dd*1000:.1f} mm")

    # ---------- rotation directions ----------
    def moved(p, jn, an, deg):
        j = J[jn]
        return rot(a(p), a(j["center"]), j["axes"][an], deg) - a(p)

    for k, s in (("L", 1), ("R", -1)):
        lat = np.array([s, 0, 0])
        d = moved(LM[f"ulnarStyloid_{k}"], f"humeroulnar_{k}", "flexion", 90)
        check(d[2] > 0.15 and d[1] > 0.1, f"elbow flexion → forearm anterior/up {k} {d}")
        # pronation: thumb side (radial styloid) lateral → anterior
        d = moved(LM[f"radialStyloid_{k}"], f"radioulnar_{k}", "pronation", 45)
        check(d[2] > 0.005 and s * d[0] < 0, f"pronation moves radial styloid lateral→anterior {k} {d}")
        d = moved(LM[f"ankleCenter_{k}"], f"knee_{k}", "flexion", 90)
        check(d[2] < -0.2, f"knee flexion → shank posterior {k}")
        d = moved(LM[f"toeTip_{k}"], f"talocrural_{k}", "dorsiflexion", 20)
        check(d[1] > 0.02, f"dorsiflexion → toes up {k}")
        d = moved(LM[f"kneeCenter_{k}"], f"hip_{k}", "flexion", 90)
        check(d[2] > 0.3, f"hip flexion → thigh anterior {k}")
        d = moved(LM[f"kneeCenter_{k}"], f"hip_{k}", "abduction", 30)
        check(s * d[0] > 0.1, f"hip abduction → lateral {k}")
        d = moved(LM[f"tibialTuberosity_{k}"], f"knee_{k}", "axialRotation", 20)
        check(-s * d[0] > 0.005, f"tibial internal rotation → tuberosity medial {k}")
        sole = rot(np.array([[0, -1, 0]]), np.zeros(3), J[f"subtalar_{k}"]["axes"]["inversion"], 20)[0]
        check(-s * sole[0] > 0.1, f"subtalar inversion → sole faces medially {k}")
        d = moved(LM[f"middleTip_{k}"], f"radiocarpal_{k}", "flexion", 30)
        check(d[2] > 0.03, f"wrist palmar flexion → fingers anterior (palm side) {k}")
        d = moved(LM[f"middleTip_{k}"], f"radiocarpal_{k}", "ulnarDeviation", 20)
        check(-s * d[0] > 0.02, f"ulnar deviation → fingers medial {k}")
        for f, i in (("index", 2), ("middle", 3), ("ring", 4), ("little", 5)):
            d = moved(LM[f"{f}Tip_{k}"], f"mcp{i}_{k}", "flexion", 45)
            check(d[2] > 0.02, f"mcp{i} flexion → fingertip palmar {k}")
            d = moved(LM[f"{f}Tip_{k}"], f"pip{i}_{k}", "flexion", 45)
            check(d[2] > 0.01, f"pip{i} flexion → palmar {k}")
        d = moved(LM[f"indexTip_{k}"], f"mcp2_{k}", "abduction", 15)
        check(s * d[0] > 0.005, f"index abduction → radial {k}")
        d = moved(LM[f"littleTip_{k}"], f"mcp5_{k}", "abduction", 15)
        check(-s * d[0] > 0.005, f"little finger abduction → ulnar {k}")
        d = moved(LM[f"thumbTip_{k}"], f"cmc1_{k}", "flexion", 30)
        check(-s * d[0] > 0.01, f"thumb CMC flexion → ulnar {k}")
        d = moved(LM[f"thumbTip_{k}"], f"cmc1_{k}", "abduction", 30)
        check(d[2] > 0.01, f"thumb palmar abduction → anterior {k}")
        d = moved(LM[f"lateralEpicondyleHumerus_{k}"], f"glenohumeral_{k}", "internalRotation", 30)
        check(d[2] > 0.01, f"GH internal rotation → lateral epicondyle anterior {k}")
        d = moved(LM[f"elbowCenter_{k}"], f"glenohumeral_{k}", "flexion", 90)
        check(d[2] > 0.2, f"GH flexion → arm anterior {k}")
        d = moved(LM[f"elbowCenter_{k}"], f"glenohumeral_{k}", "abduction", 90)
        check(s * d[0] > 0.2, f"GH abduction → arm lateral {k}")
        d = moved(LM[f"inferiorAngleScapula_{k}"], f"acromioclavicular_{k}", "upwardRotation", 20)
        check(s * d[0] > 0.02, f"scapular upward rotation → inferior angle lateral {k}")
        d = moved(J[f"acromioclavicular_{k}"]["center"], f"sternoclavicular_{k}", "elevation", 20)
        check(d[1] > 0.03, f"SC elevation → acromial end up {k}")
        d = moved(J[f"acromioclavicular_{k}"]["center"], f"sternoclavicular_{k}", "protraction", 20)
        check(d[2] > 0.03, f"SC protraction → acromial end anterior {k}")
        pf = J[f"patellofemoral_{k}"]
        d = moved(pf["patellaApex"], f"patellofemoral_{k}", "flexion", 30)
        check(d[2] < 0 and d[1] < 0.01, f"patella glides posteriorly/inferiorly with knee flexion {k}")
    d = moved(LM["vertex"], "lumbosacral", "flexion", 20)
    check(d[2] > 0.1, "spine flexion → forward")
    d = moved(LM["vertex"], "lumbosacral", "lateralBend", 20)
    check(d[0] < -0.1, "spine lateralBend + → toward subject's right (-X)")
    d = moved(LM["noseTip"], "C1C2", "axialRotation", 30)
    check(d[0] > 0.02, "axial rotation + → face turns to subject's left (+X)")
    d = moved(LM["noseTip"], "atlantooccipital", "flexion", 20)
    check(d[1] < 0, "head flexion → nose down")

    # ---------- rest pose sanity ----------
    rp = rig["restPose"]
    for k in "LR":
        check(abs(rp[k]["elbow"]["flexion"]) < 30, "rest elbow nearly straight")
        check(-100 < rp[k]["forearm"]["pronation"] < -60, "rest forearm supinated")

    print(f"\n{PASSES} checks passed, {len(FAILS)} failed")
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(main())
