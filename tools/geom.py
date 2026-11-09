"""Geometry helpers for tools/build_assets.py (BodyParts3D -> BoneSim frame, fits)."""
import os
import numpy as np
import trimesh
from scipy.spatial import cKDTree

# BodyParts3D: mm, +X = subject left, +Y = posterior, +Z = superior.
# BoneSim:     m,  +X = subject left, +Y = up,        +Z = anterior.
BP3D_TO_BONESIM = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], float) / 1000.0

STL_DIR = os.environ.get(
    "BP3D_STL", "/home/claude/data/BodyParts3D/assets/BodyParts3D_data/stl")
CACHE_DIR = os.environ.get("BP3D_CACHE", "/home/claude/data/cache")


class Bone:
    """Triangle mesh in BoneSim frame with per-vertex normals and a KD-tree."""

    def __init__(self, V, F):
        self.V = np.asarray(V, float)
        self.F = np.asarray(F, np.int64)
        m = trimesh.Trimesh(self.V, self.F, process=False)
        self.N = np.asarray(m.vertex_normals, float)
        self._tree = None
        self._mesh = m

    @property
    def tree(self):
        if self._tree is None:
            self._tree = cKDTree(self.V)
        return self._tree

    @property
    def mesh(self):
        return self._mesh

    def centroid(self):
        return self._mesh.center_mass if self._mesh.is_watertight else self.V.mean(0)


def load_fma(fid):
    os.makedirs(CACHE_DIR, exist_ok=True)
    cp = os.path.join(CACHE_DIR, fid + ".npz")
    if os.path.exists(cp):
        d = np.load(cp)
        return d["V"], d["F"]
    m = trimesh.load(os.path.join(STL_DIR, fid + ".stl"), process=True)
    V = np.asarray(m.vertices) @ BP3D_TO_BONESIM.T
    F = np.asarray(m.faces)
    # rotation (det=+1) keeps winding: outward normals stay outward
    np.savez(cp, V=V, F=F)
    return V, F


def load_bone(ids):
    Vs, Fs, off = [], [], 0
    for fid in ids:
        V, F = load_fma(fid)
        Vs.append(V)
        Fs.append(F + off)
        off += len(V)
    return Bone(np.vstack(Vs), np.vstack(Fs))


def unit(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def sphere_fit(P):
    """Algebraic least-squares sphere: returns centre, radius, rms residual."""
    P = np.asarray(P, float)
    A = np.c_[2 * P, np.ones(len(P))]
    b = (P ** 2).sum(1)
    x, *_ = np.linalg.lstsq(A, b, rcond=None)
    c = x[:3]
    r = np.sqrt(max(x[3] + c @ c, 1e-12))
    res = np.linalg.norm(P - c, axis=1) - r
    return c, r, float(np.sqrt((res ** 2).mean()))


def robust_sphere(P, N=None, iters=8, tol=0.15, normal_min=0.7, c0=None, r0=None,
                  rmin=None, rmax=None):
    """Iteratively-reweighted sphere fit on surface points P (normals N, outward).

    Keeps points whose distance to the centre is within (1±tol)·r and whose outward
    normal points away from the centre (convex articular cap)."""
    P = np.asarray(P, float)
    mask = np.ones(len(P), bool)
    if c0 is None:
        c, r, _ = sphere_fit(P)
    else:
        c, r = np.asarray(c0, float), float(r0)
    for _ in range(iters):
        d = P - c
        dist = np.linalg.norm(d, axis=1)
        m = np.abs(dist - r) < tol * r
        if N is not None:
            m &= (np.einsum("ij,ij->i", d, N) / np.maximum(dist, 1e-9)) > normal_min
        if m.sum() < 20:
            break
        mask = m
        c, r, _ = sphere_fit(P[mask])
        if rmin is not None and r < rmin:
            break
        if rmax is not None and r > rmax:
            break
    _, _, rms = sphere_fit(P[mask])
    return c, r, rms, mask


def circle_fit_plane(P, axis):
    """Fit circle to points projected on the plane ⟂ axis. Returns centre (3D, on mean plane), r."""
    axis = unit(axis)
    a = np.cross(axis, [1, 0, 0])
    if np.linalg.norm(a) < 0.1:
        a = np.cross(axis, [0, 1, 0])
    e1 = unit(a)
    e2 = np.cross(axis, e1)
    m = P.mean(0)
    Q = np.c_[(P - m) @ e1, (P - m) @ e2]
    A = np.c_[2 * Q, np.ones(len(Q))]
    b = (Q ** 2).sum(1)
    x, *_ = np.linalg.lstsq(A, b, rcond=None)
    c2 = x[:2]
    r = np.sqrt(x[2] + c2 @ c2)
    return m + c2[0] * e1 + c2[1] * e2, r


def pca_axis(P, toward=None):
    P = np.asarray(P, float)
    m = P.mean(0)
    _, _, vt = np.linalg.svd(P - m, full_matrices=False)
    a = vt[0]
    if toward is not None and a @ toward < 0:
        a = -a
    return m, a


def contact(A: Bone, B: Bone, band=0.003, min_pts=30):
    """Articular contact region between two (non-touching) bones.

    Returns (centroid of the facing vertices of both bones, A-vertex mask, B-vertex mask, gap)."""
    dA, _ = B.tree.query(A.V)
    dB, _ = A.tree.query(B.V)
    gap = min(dA.min(), dB.min())
    for k in range(6):
        thr = gap + band * (1.5 ** k)
        mA, mB = dA < thr, dB < thr
        if mA.sum() >= min_pts and mB.sum() >= min_pts:
            break
    c = np.vstack([A.V[mA], B.V[mB]]).mean(0)
    return c, mA, mB, gap


def closest_point_on_line(p, a, d):
    d = unit(d)
    return a + ((p - a) @ d) * d


def orient(v, ref):
    """Flip v so that v·ref > 0."""
    v = unit(v)
    return v if v @ ref >= 0 else -v


def rotate(P, center, axis, deg):
    """Rodrigues rotation of points P about the line (center, axis) by deg (right-hand rule)."""
    k = unit(axis)
    t = np.radians(deg)
    P = np.atleast_2d(P) - center
    Pr = P * np.cos(t) + np.cross(k, P) * np.sin(t) + np.outer(P @ k, k) * (1 - np.cos(t))
    return Pr + center


def rotational_axis_fit(P, N):
    """Axis of a surface of revolution (Pottmann & Randrup): all surface normal lines
    intersect the axis.  Minimise sum (a·(p×n) + ā·n)^2 with |a| = 1.
    Returns (point on axis closest to the centroid of P, unit direction)."""
    P = np.asarray(P, float)
    N = np.asarray(N, float)
    m0 = P.mean(0)
    Pc = P - m0
    L = np.c_[np.cross(Pc, N), N]          # rows: [p×n, n]
    M = L.T @ L
    # minimise x^T M x  s.t. a^T a = 1 where x = [a, abar]
    A, Bm, D = M[:3, :3], M[:3, 3:], M[3:, 3:]
    Dinv = np.linalg.pinv(D)
    S = A - Bm @ Dinv @ Bm.T                # eliminate abar = -Dinv Bm^T a
    w, U = np.linalg.eigh(S)
    a = U[:, 0]
    abar = -Dinv @ Bm.T @ a
    # point on line closest to origin (centroid frame): a × abar  (|a| = 1)
    p0 = np.cross(a, abar)
    p = p0 + ((-p0) @ a) * a + m0
    return p, unit(a)


def line_dist(P, p, a):
    a = unit(a)
    D = np.atleast_2d(P) - p
    return np.linalg.norm(D - np.outer(D @ a, a), axis=1)
