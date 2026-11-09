#!/usr/bin/env python3
"""BoneSim asset builder: BodyParts3D STL -> assets/skeleton.glb + assets/rig.json

Usage:
    git clone --depth 1 https://github.com/Kevin-Mattheus-Moerman/BodyParts3D /home/claude/data/BodyParts3D
    pip install --break-system-packages trimesh numpy scipy fast-simplification pygltflib rtree matplotlib
    python3 tools/build_assets.py [--stl DIR] [--out assets]

Coordinate transform (BodyParts3D mm, +X left, +Y posterior, +Z superior) ->
BoneSim (m, +X subject-left, +Y up, +Z anterior):  p_bs = (x, z, -y) / 1000.
Pure rotation (det +1) + scale; no translation (floor ≈ y=0, midline ≈ x=0).

Every joint centre/axis is computed from the full-resolution geometry *before*
decimation.  AXIS CONVENTION: rotating the child segment by +angle (right-hand rule)
about axes[k] produces the positive anatomical motion named k (see AXIS_CONVENTION).
"""
import argparse
import json
import os
import struct
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import geom  # noqa: E402
from geom import (Bone, load_bone, unit, robust_sphere, sphere_fit, pca_axis, contact,  # noqa: E402
                  orient, rotational_axis_fit, line_dist, closest_point_on_line, circle_fit_plane)
from bp3d_nodes import node_table, DISCS, NASAL_CARTILAGE, OCCIPITAL  # noqa: E402

X, Y, Z = np.eye(3)
SIDE_S = {"L": 1.0, "R": -1.0}
AXIS_CONVENTION = (
    "World/rest-pose unit vectors. Rotating the CHILD segment about the joint 'center' by +angle "
    "(right-hand rule, i.e. THREE.Quaternion.setFromAxisAngle(axis, +angle)) produces the POSITIVE "
    "anatomical motion named by the key (flexion, pronation, dorsiflexion, inversion, abduction, "
    "internalRotation, ulnarDeviation, ...). This matches src/shared/conventions.js signs, e.g. "
    "elbow flexion axis ≈ -X (= Rx(-flex)), knee flexion axis ≈ +X (= Rx(+flex)), forearm pronation axis "
    "≈ -s·Y (= Ry(-s·pron)), ankle dorsiflexion ≈ -X, hip abduction ≈ +s·Z, s = +1 left / -1 right. "
    "Spine/neck joints: flexion +X, lateralBend +Z (toward subject's right), axialRotation +Y (toward left).")


def J(v):
    return [round(float(x), 5) for x in v]


# ----------------------------------------------------------------- helpers
def extreme(b: Bone, direction, mask=None):
    V = b.V if mask is None else b.V[mask]
    return V[np.argmax(V @ direction)]


def long_axis(b: Bone, toward):
    return pca_axis(b.V, toward=toward)[1]


def perp(v, axis):
    axis = unit(axis)
    return unit(v - (v @ axis) * axis)


def head_center(prox: Bone, dist: Bone, rmin, rmax, d_long):
    """Centre of curvature of the articular head of `prox` facing `dist`.
    Robust sphere on prox's contact patch; fallback = contact centroid moved proximally."""
    c, mA, mB, gap = contact(prox, dist, band=0.003, min_pts=40)
    # widen the patch around the contact to cover the head
    dA, _ = dist.tree.query(prox.V)
    patch = dA < gap + 0.006
    try:
        cc, r, rms, m = robust_sphere(prox.V[patch], prox.N[patch], normal_min=0.5, tol=0.25)
        ok = rmin <= r <= rmax and np.linalg.norm(cc - c) < 2.0 * r + 0.004 and m.sum() > 30
    except Exception:
        ok = False
    if ok:
        return cc, f"sphere fit to articular head of proximal bone (r={r*1000:.1f} mm, rms={rms*1000:.2f} mm)"
    # fallback: contact centroid moved proximally by half the local half-width
    w = np.ptp(prox.V[mA] @ perp(X, d_long)) if mA.sum() else 0.01
    return c - unit(d_long) * 0.35 * max(w, 0.006), "fallback: contact centroid shifted proximally"


# ----------------------------------------------------------------- rig computation
def compute_rig(B):
    joints, lm, notes, rest = {}, {}, {}, {}

    # ---------------- lower limb ----------------
    for k, s in SIDE_S.items():
        lat = s * X
        fem, tib, fib, pat, tal, cal = (B[f"{n}_{k}"] for n in
                                        ("femur", "tibia", "fibula", "patella", "talus", "calcaneus"))
        # hip: femoral head sphere
        V, N = fem.V, fem.N
        ytop = V[:, 1].max()
        L = np.ptp(V[:, 1])
        prox = V[:, 1] > ytop - 0.12 * L
        xm = np.median(V[prox, 0])
        seed = prox & (s * (V[:, 0] - xm) < 0)
        hc, hr, hrms, _ = robust_sphere(V[seed], N[seed], normal_min=0.6)
        joints[f"hip_{k}"] = dict(
            center=J(hc), radius=round(hr, 4),
            axes=dict(flexion=J(-X), abduction=J(s * Z), internalRotation=J(-s * Y)),
            method=f"robust sphere fit to femoral head (r={hr*1000:.1f} mm, rms={hrms*1000:.2f} mm); axes = pelvis frame (conventions hipQ)")
        # knee: posterior condyle spheres
        ybot = V[:, 1].min()
        dist = V[:, 1] < ybot + 0.045
        xn = np.median(V[dist, 0])
        post = dist & (N[:, 2] < -0.3)
        cen = {}
        for nm, sel in (("medial", post & (s * (V[:, 0] - xn) < -0.004)),
                        ("lateral", post & (s * (V[:, 0] - xn) > 0.004))):
            c, r, rms, m = robust_sphere(V[sel], N[sel], normal_min=0.6, tol=0.2)
            cen[nm] = (c, r, rms)
        kc = 0.5 * (cen["medial"][0] + cen["lateral"][0])
        kax = orient(cen["lateral"][0] - cen["medial"][0], X)    # knee flexion = Rx(+)
        epiM = extreme(fem, -lat, V[:, 1] < ybot + 0.07)
        epiL = extreme(fem, lat, V[:, 1] < ybot + 0.07)
        lm[f"medialEpicondyleFemur_{k}"] = J(epiM)
        lm[f"lateralEpicondyleFemur_{k}"] = J(epiL)
        # tibia mechanical axis: plateau centre -> ankle centre
        tV = tib.V
        ttop = tV[:, 1].max()
        plateau = tV[tV[:, 1] > ttop - 0.012]
        plateau_c = plateau.mean(0)
        medMal_tip = tV[np.argmin(tV[:, 1])]
        latMal_tip = fib.V[np.argmin(fib.V[:, 1])]
        # talar dome
        cT, mTt, mTb, _ = contact(tal, tib, band=0.003)
        dome_c, dome_r, dome_rms, _ = robust_sphere(tal.V[mTt], tal.N[mTt], normal_min=0.3, tol=0.3)
        tc_ax = orient(medMal_tip - latMal_tip, -X)   # dorsiflexion = Rx(-)
        ankle_c = closest_point_on_line(dome_c, 0.5 * (medMal_tip + latMal_tip), tc_ax)
        tib_up = unit(plateau_c - ankle_c)
        joints[f"knee_{k}"] = dict(
            center=J(kc),
            axes=dict(flexion=J(kax), axialRotation=J(-s * tib_up)),
            medialCondyleCenter=J(cen["medial"][0]), lateralCondyleCenter=J(cen["lateral"][0]),
            method=("flexion axis through centres of spheres fitted to the posterior femoral condyles "
                    f"(r med {cen['medial'][1]*1000:.1f} / lat {cen['lateral'][1]*1000:.1f} mm); centre = midpoint. "
                    "axialRotation = tibial mechanical axis (plateau centre→ankle centre), + = internal rotation of tibia "
                    "(screw-home = external = negative)"))
        # patellofemoral
        ant = dist & (N[:, 2] > 0.3) & (V[:, 1] > ybot + 0.008)
        Pa = V[ant]
        groove = []
        for yy in np.arange(Pa[:, 1].min() + 0.004, Pa[:, 1].max(), 0.002):
            sl = Pa[np.abs(Pa[:, 1] - yy) < 0.001]
            sl = sl[np.abs(sl[:, 0] - xn) < 0.015]
            if len(sl) < 5:
                continue
            groove.append(sl[np.argmin(sl[:, 2])])  # deepest (most posterior) point of trochlea
        groove = np.array(groove)
        gc, gr = circle_fit_plane(groove, kax)
        pV = pat.V
        apex = pV[np.argmin(pV[:, 1])]
        base = pV[np.argmax(pV[:, 1])]
        tt_mask = (tV[:, 1] < ttop - 0.015) & (tV[:, 1] > ttop - 0.07)
        tub = extreme(tib, Z, tt_mask)
        joints[f"patellofemoral_{k}"] = dict(
            center=J(gc), grooveRadius=round(float(gr), 4),
            axes=dict(flexion=J(kax)),
            patellaRestCentroid=J(pat.centroid()),
            patellaApex=J(apex), patellaBase=J(base), tibialTuberosity=J(tub),
            patellarTendonLength=round(float(np.linalg.norm(apex - tub)), 4),
            method=("centre = circle fit (in plane ⟂ knee axis) to the deepest points of the femoral trochlear groove; "
                    "patella rotates about axes.flexion through this centre (≈0.7×knee flexion) keeping "
                    "|patellaApex−tibialTuberosity| = patellarTendonLength"))
        lm[f"patellaApex_{k}"] = J(apex)
        lm[f"patellaBase_{k}"] = J(base)
        lm[f"tibialTuberosity_{k}"] = J(tub)
        # talocrural
        joints[f"talocrural_{k}"] = dict(
            center=J(ankle_c), axes=dict(dorsiflexion=J(tc_ax)),
            medialMalleolusTip=J(medMal_tip), lateralMalleolusTip=J(latMal_tip),
            method=("axis through tips of medial (tibia) and lateral (fibula) malleoli (Inman); "
                    f"centre = point on that axis closest to the talar dome sphere centre (r={dome_r*1000:.1f} mm)"))
        lm[f"medialMalleolus_{k}"] = J(extreme(tib, -lat, tV[:, 1] < tV[:, 1].min() + 0.03))
        lm[f"lateralMalleolus_{k}"] = J(extreme(fib, lat, fib.V[:, 1] < fib.V[:, 1].min() + 0.03))
        # subtalar (Inman 1976: 42° up from horizontal, 23° medial to the foot long axis)
        cST, _, _, _ = contact(tal, cal, band=0.002)
        heel = cal.V[np.argmin(cal.V[:, 2])]
        mt2 = B[f"metatarsal2_{k}"]
        mt2_head = extreme(mt2, Z)
        foot_f = unit(np.r_[(mt2_head - heel)[0], 0, (mt2_head - heel)[2]])
        med = perp(-lat, foot_f)
        a42, a23 = np.radians(42), np.radians(23)
        inman = unit(np.cos(a42) * (np.cos(a23) * foot_f + np.sin(a23) * med) + np.sin(a42) * Y)
        joints[f"subtalar_{k}"] = dict(
            center=J(cST), axes=dict(inversion=J(-s * inman)),
            method=("Inman average axis: 42° above horizontal, 23° medial to foot long axis "
                    "(heel→2nd metatarsal head); anchored at centroid of talus–calcaneus articular contact. "
                    "+ = inversion/supination"))
        toes = B[f"toePhalanges_{k}"]
        toe_tip = extreme(toes, Z)
        lm[f"heel_{k}"] = J(heel)
        lm[f"toeTip_{k}"] = J(toe_tip)
        lm[f"metatarsal5Base_{k}"] = J(extreme(B[f"metatarsal5_{k}"], -Z))
        gt_mask = fem.V[:, 1] > hc[1] - 0.08
        lm[f"greaterTrochanter_{k}"] = J(extreme(fem, lat, gt_mask))
        lm[f"hipCenter_{k}"] = J(hc)
        lm[f"kneeCenter_{k}"] = J(kc)
        lm[f"ankleCenter_{k}"] = J(ankle_c)
        rest[f"_lower_{k}"] = dict(hip=hc, knee=kc, ankle=ankle_c, heel=heel, toe=toe_tip, mt2=mt2_head,
                                   sole=min(cal.V[:, 1].min(), toes.V[:, 1].min()))

    # ---------------- pelvis ----------------
    for k, s in SIDE_S.items():
        hb = B[f"hipBone_{k}"]
        hc = np.array(joints[f"hip_{k}"]["center"])
        lm[f"ASIS_{k}"] = J(extreme(hb, Z, hb.V[:, 1] > hc[1] + 0.03))
        lm[f"iliacCrestTop_{k}"] = J(extreme(hb, Y))
        lm[f"ischialTuberosity_{k}"] = J(extreme(hb, -Y))
    cPS, mR, mL, _ = contact(B["hipBone_R"], B["hipBone_L"], band=0.003)
    lm["pubicSymphysis"] = J(cPS)
    lm["pubicSymphysisSuperior"] = J(np.r_[cPS[0], max(B["hipBone_R"].V[mR, 1].max(), B["hipBone_L"].V[mL, 1].max()), cPS[2]])
    pelvis_c = 0.5 * (np.array(joints["hip_L"]["center"]) + np.array(joints["hip_R"]["center"]))
    lm["pelvisCenter"] = J(pelvis_c)
    sac = B["sacrum"]
    lm["sacralPromontory"] = J(extreme(sac, Z + 0.3 * Y, (sac.V[:, 1] > sac.V[:, 1].max() - 0.03) & (np.abs(sac.V[:, 0]) < 0.006)))
    lm["coccyxApproxTip"] = J(sac.V[np.argmin(sac.V[:, 1])])  # no coccyx in BP3D: sacral apex

    # ---------------- spine ----------------
    def disc_c(vert):
        return load_bone([DISCS[vert]]).centroid()

    spine_axes = dict(flexion=J(X), lateralBend=J(Z), axialRotation=J(Y))
    pairs = [("lumbosacral", "L5"), ("L4L5", "L4"), ("L3L4", "L3"), ("L2L3", "L2"), ("L1L2", "L1"),
             ("T12L1", "T12"), ("C7T1", "C7"), ("C6C7", "C6"), ("C5C6", "C5"), ("C4C5", "C4"),
             ("C3C4", "C3"), ("C2C3", "C2")]
    for jn, disc in pairs:
        joints[jn] = dict(center=J(disc_c(disc)), axes=spine_axes,
                          method=f"centroid of the {disc} intervertebral disc (below {disc}); axes = world (conventions spineQ)")
    ax_b = B["C2"]
    dens = ax_b.V[ax_b.V[:, 1] > ax_b.V[:, 1].max() - 0.012]
    joints["C1C2"] = dict(center=J(dens.mean(0)), axes=spine_axes,
                          method="centroid of the upper 12 mm of the dens (axis); axialRotation about the dens")
    occ = load_bone([OCCIPITAL])
    cAO, _, _, _ = contact(B["C1"], occ, band=0.003)
    joints["atlantooccipital"] = dict(center=J(cAO), axes=spine_axes,
                                      method="centroid of articular contact between atlas superior facets and occipital condyles")
    # thoracic joints between T vertebrae (inside the rigid thorax; for reference only)
    lm["C7spinous"] = J(extreme(B["C7"], -Z))
    lm["T1spinous"] = J(extreme(B["T1"], -Z))
    lm["L4spinous"] = J(extreme(B["L4"], -Z))
    for i in range(1, 6):
        lm[f"L{i}body"] = J(B[f"L{i}"].V[B[f"L{i}"].V[:, 2] > np.median(B[f"L{i}"].V[:, 2])].mean(0))

    # ---------------- thorax / sternum ----------------
    man, stb, xip = B["manubrium"], B["sternumBody"], B["xiphoid"]
    mx = np.median(man.V[:, 0])
    mid = np.abs(man.V[:, 0] - mx) < 0.004
    lm["jugularNotch"] = J(extreme(man, Y, mid))
    cSA, m1, m2, _ = contact(man, stb, band=0.002)
    pts = np.vstack([man.V[m1], stb.V[m2]])
    lm["sternalAngle"] = J(pts[np.argmax(pts[:, 2])])
    lm["xiphoid"] = J(xip.V[np.argmin(xip.V[:, 1])])

    # ---------------- skull ----------------
    sk = B["skull"]
    lm["vertex"] = J(extreme(sk, Y))
    occb = occ
    midO = np.abs(occb.V[:, 0]) < 0.01
    lm["EOM"] = J(extreme(occb, -Z, midO))  # external occipital protuberance
    nas = load_bone(["FMA53647", "FMA53648"])
    fr = load_bone(["FMA52734"])
    cN, mN, mF, _ = contact(nas, fr, band=0.002)
    lm["nasion"] = J(np.vstack([nas.V[mN], fr.V[mF]])[np.argmax(np.r_[nas.V[mN, 2], fr.V[mF, 2]])])
    lm["noseTip"] = J(extreme(load_bone([NASAL_CARTILAGE]), Z))
    mand = B["mandible"]
    lm["menton"] = J(mand.V[np.argmin(mand.V[:, 1])])
    for k, s in SIDE_S.items():
        sub = mand.V[s * mand.V[:, 0] > 0]
        lm[f"gonion_{k}"] = J(sub[np.argmax(-sub[:, 1] - sub[:, 2] + s * sub[:, 0])])  # infero-postero-lateral corner

    # ---------------- upper limb ----------------
    for k, s in SIDE_S.items():
        lat = s * X
        cla, sca, hum, uln, rad = (B[f"{n}_{k}"] for n in ("clavicle", "scapula", "humerus", "ulna", "radius"))
        # SC / AC
        cSC, _, _, _ = contact(cla, man, band=0.002)
        cAC, _, _, _ = contact(cla, sca, band=0.002)
        dcl = unit(cAC - cSC)
        joints[f"sternoclavicular_{k}"] = dict(
            center=J(cSC),
            axes=dict(elevation=J(unit(np.cross(dcl, Y))), protraction=J(unit(np.cross(dcl, Z))),
                      posteriorRotation=J(orient(dcl, -X))),
            method="centroid of clavicle–manubrium articular contact; elevation/protraction axes ⟂ clavicle (SC→AC), posteriorRotation about clavicle long axis")
        # glenohumeral
        V, N = hum.V, hum.N
        ytop = V[:, 1].max()
        L = np.ptp(V[:, 1])
        prox = V[:, 1] > ytop - 0.12 * L
        xm = np.median(V[prox, 0])
        seed = prox & (s * (V[:, 0] - xm) < 0.005)
        ghc, ghr, ghrms, _ = robust_sphere(V[seed], N[seed], normal_min=0.6)
        # glenoid centre: scapula vertices facing the head
        dS = np.linalg.norm(sca.V - ghc, axis=1)
        gl = (dS < dS.min() + 0.004) & (np.einsum("ij,ij->i", ghc - sca.V, sca.N) > 0)
        glen = sca.V[gl].mean(0)
        # humeroulnar: rotational-surface fit of trochlea + capitulum
        ybot = V[:, 1].min()
        p, ax = np.array([s * 0.22, ybot + 0.017, np.median(V[V[:, 1] < ybot + 0.03, 2])]), lat.copy()
        for _ in range(12):
            D = V - p
            t = D @ ax
            radv = D - np.outer(t, ax)
            rn = np.linalg.norm(radv, axis=1)
            sel = (V[:, 1] < ybot + 0.035) & (rn > 0.005) & (rn < 0.016) & (np.abs(t) < 0.024)
            sel &= np.einsum("ij,ij->i", radv, N) / np.maximum(rn, 1e-9) > 0.85
            sel &= radv[:, 1] / np.maximum(rn, 1e-9) < 0.3
            p, ax = rotational_axis_fit(V[sel], N[sel])
            ax = orient(ax, lat)
        tt = (V[sel] - p) @ ax
        art_c = closest_point_on_line(V[sel].mean(0), p, ax)
        elbow_c = art_c
        flex_ax = orient(ax, -X)   # elbow = Rx(-flex)
        epiM = extreme(hum, -lat, V[:, 1] < ybot + 0.06)
        epiL = extreme(hum, lat, V[:, 1] < ybot + 0.06)
        # radioulnar
        RV = rad.V
        d_r = long_axis(rad, -Y)
        tr = RV @ d_r
        headpts = (tr < tr.min() + 0.010) & (tr > tr.min() + 0.003) & (np.abs(rad.N @ d_r) < 0.4)
        rh_c, rh_r = circle_fit_plane(RV[headpts], d_r)
        UV = uln.V
        d_u = long_axis(uln, -Y)
        tu = UV @ d_u
        # ulnar head: distal 14 mm, excluding styloid (sphere fit with normals)
        dsel = (tu > tu.max() - 0.016) & (tu < tu.max() - 0.002)
        uh_c, uh_r, uh_rms, _ = robust_sphere(UV[dsel], uln.N[dsel], normal_min=0.5, tol=0.25)
        if not (0.004 < uh_r < 0.014):
            uh_c = UV[dsel].mean(0)
        pro_ax = orient(uh_c - rh_c, -s * Y)    # forearm = Ry(-s·pron)
        joints[f"humeroulnar_{k}"] = dict(
            center=J(elbow_c), axes=dict(flexion=J(flex_ax)),
            axisMedialEnd=J(p + ax * tt.min() if s > 0 else p + ax * tt.max()),
            axisLateralEnd=J(p + ax * tt.max() if s > 0 else p + ax * tt.min()),
            method=("surface-of-revolution (Pottmann normal-line) fit of the trochlea+capitulum articular surface; "
                    "centre = projection of articular-surface centroid onto the axis. Oblique axis → carrying angle"))
        joints[f"radioulnar_{k}"] = dict(
            center=J(rh_c), axes=dict(pronation=J(pro_ax)),
            proximal=J(rh_c), distal=J(uh_c),
            method=(f"axis from radial head centre (circle fit of head circumference, r={rh_r*1000:.1f} mm) "
                    f"to ulnar head centre (sphere fit, r={uh_r*1000:.1f} mm); radius rotates about it, ulna fixed"))
        joints[f"glenohumeral_{k}"] = dict(
            center=J(ghc), radius=round(ghr, 4),
            axes=dict(flexion=J(-X), abduction=J(s * Z), internalRotation=J(-s * unit(ghc - elbow_c))),
            glenoidCenter=J(glen),
            method=(f"robust sphere fit to humeral head (r={ghr*1000:.1f} mm, rms={ghrms*1000:.2f} mm); "
                    "flexion/abduction = thorax-frame axes, internalRotation about humeral long axis (GH→elbow)"))
        # scapula landmarks + AC axes
        SV = sca.V
        infA = SV[np.argmin(SV[:, 1])]
        acro = extreme(sca, lat, SV[:, 1] > glen[1] + 0.01)
        cora = extreme(sca, Z)
        mpos = SV @ (-lat)
        medial_border = mpos > mpos.max() - 0.015
        supA = SV[medial_border][np.argmax(SV[medial_border][:, 1])]
        nsc = orient(np.linalg.svd(SV - SV.mean(0), full_matrices=False)[2][2], Z)
        # root of spine: most medial point of the spine crest (points standing >30% of max height
        # behind the scapular plane)
        hgt = (SV - SV.mean(0)) @ (-nsc)
        crest = SV[hgt > 0.3 * hgt.max()]
        root = crest[np.argmax(crest @ (-lat))]
        spine_dir = orient(perp(acro - root, nsc), lat)
        joints[f"acromioclavicular_{k}"] = dict(
            center=J(cAC),
            axes=dict(upwardRotation=J(s * nsc), internalRotation=J(-s * Y), posteriorTilt=J(orient(spine_dir, -X))),
            scapularPlaneNormal=J(nsc),
            method=("centroid of clavicle–acromion articular contact; upwardRotation ⟂ scapular plane "
                    "(PCA normal), internalRotation ≈ vertical, posteriorTilt about scapular spine direction"))
        lm[f"acromion_{k}"] = J(acro)
        lm[f"inferiorAngleScapula_{k}"] = J(infA)
        lm[f"superiorAngleScapula_{k}"] = J(supA)
        lm[f"coracoid_{k}"] = J(cora)
        lm[f"glenoidCenter_{k}"] = J(glen)
        lm[f"rootOfSpineScapula_{k}"] = J(root)
        lm[f"medialEpicondyleHumerus_{k}"] = J(epiM)
        lm[f"lateralEpicondyleHumerus_{k}"] = J(epiL)
        lm[f"olecranon_{k}"] = J(extreme(uln, -Z, tu < tu.min() + 0.03))
        # wrist
        d_fa = unit(uh_c - elbow_c)
        rs = RV[np.argmax(RV @ d_fa)]
        us = UV[np.argmax(UV @ d_fa)]
        lm[f"radialStyloid_{k}"] = J(rs)
        lm[f"ulnarStyloid_{k}"] = J(us)
        lm[f"radialHead_{k}"] = J(extreme(rad, lat, tr < tr.min() + 0.012))
        sca_, lun, tri, pis, cap = (B[f"{n}_{k}"] for n in ("scaphoid", "lunate", "triquetrum", "pisiform", "capitate"))
        prow = Bone(np.vstack([sca_.V, lun.V]), np.vstack([sca_.F, lun.F + len(sca_.V)]))
        c_rc, mP, mRr, _ = contact(prow, rad, band=0.003)
        rc_c, rc_r, rc_rms, _ = robust_sphere(prow.V[mP], prow.N[mP], normal_min=0.3, tol=0.3)
        rc_method = f"sphere fit to proximal articular surface of scaphoid+lunate (r={rc_r*1000:.1f} mm)"
        if not (0.008 < rc_r < 0.04) or np.linalg.norm(rc_c - lun.centroid()) > 0.025:
            rc_c = lun.centroid()
            rc_method = "fallback: lunate centroid"
        dRow = Bone(np.vstack([sca_.V, lun.V]), np.vstack([sca_.F, lun.F + len(sca_.V)]))
        c_mc, mC, _, _ = contact(cap, dRow, band=0.003)
        mc_c, mc_r, mc_rms, _ = robust_sphere(cap.V[mC], cap.N[mC], normal_min=0.3, tol=0.3)
        mc_method = f"sphere fit to capitate head (r={mc_r*1000:.1f} mm)"
        if not (0.003 < mc_r < 0.02) or np.linalg.norm(mc_c - cap.centroid()) > 0.02:
            mc_c = c_mc
            mc_method = "fallback: capitate–proximal row contact centroid"
        # palm frame
        mcs = np.vstack([B[f"metacarpal{i}_{k}"].V for i in (2, 3, 4, 5)])
        n_palm = np.linalg.svd(mcs - mcs.mean(0), full_matrices=False)[2][2]
        n_palm = orient(n_palm, pis.centroid() - tri.centroid())
        d_hand = perp(unit(extreme(B[f"metacarpal3_{k}"], d_fa) - mc_c), n_palm)
        n_palm = perp(n_palm, d_fa)
        radial_dir = perp(rs - us, d_fa)
        wr_flex = unit(np.cross(d_fa, n_palm))
        wr_uld = unit(np.cross(d_fa, -radial_dir))
        for jn, c, meth in ((f"radiocarpal_{k}", rc_c, rc_method), (f"midcarpal_{k}", mc_c, mc_method)):
            joints[jn] = dict(center=J(c), axes=dict(flexion=J(wr_flex), ulnarDeviation=J(wr_uld)),
                              method=meth + "; flexion axis = forearm axis × palmar normal, ulnarDeviation = forearm axis × ulnar direction "
                                     "(palmar normal from metacarpal 2–5 plane, oriented toward pisiform)")
        lm[f"wristCenter_{k}"] = J(rc_c)
        lm[f"elbowCenter_{k}"] = J(elbow_c)
        lm[f"shoulderCenter_{k}"] = J(ghc)
        # fingers
        finger_ulnar = -radial_dir
        for f, mcn in (("index", 2), ("middle", 3), ("ring", 4), ("little", 5)):
            chain = [B[f"metacarpal{mcn}_{k}"], B[f"{f}Prox_{k}"], B[f"{f}Mid_{k}"], B[f"{f}Dist_{k}"]]
            names = [f"mcp{mcn}_{k}", f"pip{mcn}_{k}", f"dip{mcn}_{k}"]
            rr = [(0.003, 0.014), (0.002, 0.010), (0.0015, 0.008)]
            away = radial_dir if f in ("index", "middle") else finger_ulnar
            for i, jn in enumerate(names):
                pb, db = chain[i], chain[i + 1]
                d_long = unit(db.centroid() - pb.centroid())
                c, meth = head_center(pb, db, *rr[i], d_long)
                d = unit(db.centroid() - c)
                nP = perp(n_palm, d)
                axes = dict(flexion=J(unit(np.cross(d, nP))))
                if i == 0:
                    axes["abduction"] = J(unit(np.cross(d, perp(away, d))))
                joints[jn] = dict(center=J(c), axes=axes, method=meth + "; flexion axis = bone axis × palmar normal")
            tip = extreme(chain[3], unit(chain[3].centroid() - chain[2].centroid()))
            lm[f"{f}Tip_{k}"] = J(tip)
        # thumb
        tz, m1, tp, td = (B[f"{n}_{k}"] for n in ("trapezium", "metacarpal1", "thumbProx", "thumbDist"))
        c_cmc, _, _, _ = contact(m1, tz, band=0.002)
        d1 = unit(m1.centroid() - c_cmc)
        flex_dir = perp(finger_ulnar, d1)
        abd_dir = perp(n_palm, d1)
        joints[f"cmc1_{k}"] = dict(center=J(c_cmc),
                                   axes=dict(flexion=J(unit(np.cross(d1, flex_dir))), abduction=J(unit(np.cross(d1, abd_dir)))),
                                   method="centroid of trapezium–1st metacarpal saddle contact; flexion moves thumb ulnarward, abduction palmarward (approximate)")
        for jn, pb, db, rrng in ((f"mcp1_{k}", m1, tp, (0.003, 0.014)), (f"ip1_{k}", tp, td, (0.002, 0.010))):
            d_long = unit(db.centroid() - pb.centroid())
            c, meth = head_center(pb, db, *rrng, d_long)
            d = unit(db.centroid() - c)
            m = perp(unit(0.7 * finger_ulnar + 0.7 * n_palm), d)
            joints[jn] = dict(center=J(c), axes=dict(flexion=J(unit(np.cross(d, m)))),
                              method=meth + "; flexion toward palm/ulnar side (approximate thumb plane)")
        lm[f"thumbTip_{k}"] = J(extreme(td, unit(td.centroid() - tp.centroid())))
        rest[f"_upper_{k}"] = dict(gh=ghc, elbow=elbow_c, wrist=rc_c, n_palm=n_palm, d_fa=d_fa,
                                   radial=radial_dir, mc3=extreme(B[f"metacarpal3_{k}"], d_fa),
                                   midTip=np.array(lm[f"middleTip_{k}"]), flex_ax=flex_ax)
    return joints, lm, rest, pelvis_c


# ----------------------------------------------------------------- dims + rest pose
def angle_about(a, b, axis):
    """Signed angle (deg) rotating a to b about axis (both projected ⟂ axis)."""
    a, b = perp(a, axis), perp(b, axis)
    return float(np.degrees(np.arctan2(np.cross(a, b) @ unit(axis), a @ b)))


def compute_dims_rest(joints, lm, rest, pelvis_c, B):
    d = {}
    hipL, hipR = np.array(joints["hip_L"]["center"]), np.array(joints["hip_R"]["center"])
    ghL, ghR = rest["_upper_L"]["gh"], rest["_upper_R"]["gh"]
    sh_mid = 0.5 * (ghL + ghR)
    d["hipHalfWidth"] = 0.5 * abs(hipL[0] - hipR[0])
    d["thoraxHeight"] = sh_mid[1] - pelvis_c[1]
    d["shoulderHalfWidth"] = 0.5 * abs(ghL[0] - ghR[0])
    top = np.array(lm["vertex"])[1]
    nose = np.array(lm["noseTip"])[1]
    d["head"] = (top - nose) / 0.55
    d["neck"] = (top - sh_mid[1]) - d["head"]
    avg = lambda f: 0.5 * (f("L") + f("R"))  # noqa: E731
    d["upperArm"] = avg(lambda k: np.linalg.norm(rest[f"_upper_{k}"]["gh"] - rest[f"_upper_{k}"]["elbow"]))
    d["forearm"] = avg(lambda k: np.linalg.norm(rest[f"_upper_{k}"]["elbow"] - rest[f"_upper_{k}"]["wrist"]))
    d["hand"] = avg(lambda k: np.linalg.norm(rest[f"_upper_{k}"]["wrist"] - rest[f"_upper_{k}"]["midTip"]))
    d["thigh"] = avg(lambda k: np.linalg.norm(rest[f"_lower_{k}"]["hip"] - rest[f"_lower_{k}"]["knee"]))
    d["shank"] = avg(lambda k: np.linalg.norm(rest[f"_lower_{k}"]["knee"] - rest[f"_lower_{k}"]["ankle"]))
    d["footLength"] = avg(lambda k: rest[f"_lower_{k}"]["toe"][2] - rest[f"_lower_{k}"]["heel"][2])
    d["ankleHeight"] = avg(lambda k: rest[f"_lower_{k}"]["ankle"][1] - rest[f"_lower_{k}"]["sole"])
    dims = {k: round(float(v), 4) for k, v in d.items()}

    rp = {}
    for k, s in SIDE_S.items():
        u, lo = rest[f"_upper_{k}"], rest[f"_lower_{k}"]
        hd = unit(u["elbow"] - u["gh"])
        elev = np.degrees(np.arccos(np.clip(-hd[1], -1, 1)))
        plane = np.degrees(np.arctan2(hd[2], s * hd[0]))
        fa = unit(u["wrist"] - u["elbow"])
        elb = angle_about(hd, fa, u["flex_ax"])
        # pronation: palm normal vs the palm normal expected at the anatomical position (+Z, carried by the limb)
        pron = -90.0 + angle_about(Z, u["n_palm"], -s * fa)
        hand = unit(u["mc3"] - u["wrist"])
        wflex = angle_about(fa, hand, np.cross(fa, u["n_palm"]))
        wdev = angle_about(fa, hand, np.cross(fa, -u["radial"]))
        fd = unit(lo["knee"] - lo["hip"])
        hflex = np.degrees(np.arctan2(fd[2], -fd[1]))
        habd = np.degrees(np.arctan2(s * fd[0], -fd[1]))
        td = unit(lo["ankle"] - lo["knee"])
        kflex = angle_about(fd, td, X)
        foot = unit(lo["mt2"] - lo["heel"])
        dors = 90.0 - np.degrees(np.arccos(np.clip(foot @ (-td), -1, 1)))
        rp[k] = dict(shoulder=dict(elevation=round(elev, 1), plane=round(plane, 1)),
                     elbow=dict(flexion=round(elb, 1)), forearm=dict(pronation=round(pron, 1)),
                     wrist=dict(flexion=round(wflex, 1), ulnarDeviation=round(wdev, 1)),
                     hip=dict(flexion=round(hflex, 1), abduction=round(habd, 1)),
                     knee=dict(flexion=round(kflex, 1)), ankle=dict(dorsiflexion=round(dors, 1)))
    rp["note"] = ("Measured joint angles of the UNMODIFIED BodyParts3D mesh in conventions.js terms "
                  "(the mesh is close to, but not exactly, the anatomical position). To show Pose p, rotate each "
                  "child about its rig axis by (p − restPose); ankle dorsiflexion uses heel→2nd-metatarsal-head as foot axis.")
    return dims, rp


# ----------------------------------------------------------------- decimation + GLB
TARGETS = {
    "skull": 26000, "mandible": 5000, "hyoid": 400, "sacrum": 4000, "hipBone": 7000,
    "C1": 1600, "C2": 1600, "C": 1300, "T": 1000, "L": 1800, "rib": 900, "costalCartilage": 2000,
    "manubrium": 700, "sternumBody": 800, "xiphoid": 300,
    "clavicle": 1600, "scapula": 5000, "humerus": 9000, "ulna": 5500, "radius": 5500,
    "carpal": 350, "metacarpal": 500, "phal": 320,
    "femur": 9000, "patella": 2000, "tibia": 7000, "fibula": 2600, "talus": 1800, "calcaneus": 1800,
    "tarsal": 450, "metatarsal": 500, "toePhalanges": 2600, "skin": 30000,
}


def target_for(node):
    base = node.rsplit("_", 1)[0] if node.endswith(("_L", "_R")) else node
    if base in TARGETS:
        return TARGETS[base]
    if base.startswith("rib"):
        return TARGETS["rib"]
    if base in ("C3", "C4", "C5", "C6", "C7"):
        return TARGETS["C"]
    if base[0] == "T" and base[1:].isdigit():
        return TARGETS["T"]
    if base[0] == "L" and base[1:].isdigit():
        return TARGETS["L"]
    if base in ("scaphoid", "lunate", "triquetrum", "pisiform", "trapezium", "trapezoid", "capitate", "hamate"):
        return TARGETS["carpal"]
    if base.startswith("metacarpal"):
        return TARGETS["metacarpal"]
    if base.startswith("metatarsal"):
        return TARGETS["metatarsal"]
    if base.startswith(("navicular", "cuboid", "cuneiform")):
        return TARGETS["tarsal"]
    if base.endswith(("Prox", "Mid", "Dist")):
        return TARGETS["phal"]
    raise KeyError(node)


def decimate(V, F, target):
    import fast_simplification
    import trimesh
    if len(F) > target:
        V2, F2 = fast_simplification.simplify(V.astype(np.float32), F.astype(np.int32),
                                              target_reduction=1 - target / len(F), agg=7)
    else:
        V2, F2 = V, F
    m = trimesh.Trimesh(V2, F2, process=True)
    m.update_faces(m.nondegenerate_faces(height=1e-7))
    m.update_faces(m.unique_faces())
    m.remove_unreferenced_vertices()
    N = np.array(m.vertex_normals, float)
    nl = np.linalg.norm(N, axis=1)
    bad = nl < 0.5   # normals of opposite faces cancelled (thin sheets): use any adjacent face normal
    if bad.any():
        fn = m.face_normals
        for v in np.where(bad)[0]:
            fs = m.vertex_faces[v]
            fs = fs[fs >= 0]
            N[v] = fn[fs[0]] if len(fs) else (0, 1, 0)
    N /= np.linalg.norm(N, axis=1, keepdims=True)
    return np.asarray(m.vertices, np.float32), np.asarray(m.faces, np.uint32), N.astype(np.float32)


def write_glb(path, meshes, extras):
    """meshes: list of (name, V, F, N, material_index)."""
    import pygltflib as g
    blob = bytearray()
    views, accs, gmeshes, nodes = [], [], [], []

    def add_view(data: bytes, target):
        while len(blob) % 4:
            blob.append(0)
        views.append(g.BufferView(buffer=0, byteOffset=len(blob), byteLength=len(data), target=target))
        blob.extend(data)
        return len(views) - 1

    for name, V, F, N, mat in meshes:
        iv = add_view(V.astype("<f4").tobytes(), g.ARRAY_BUFFER)
        accs.append(g.Accessor(bufferView=iv, componentType=g.FLOAT, count=len(V), type=g.VEC3,
                               min=[float(x) for x in V.min(0)], max=[float(x) for x in V.max(0)]))
        ia = len(accs) - 1
        inn = add_view(N.astype("<f4").tobytes(), g.ARRAY_BUFFER)
        accs.append(g.Accessor(bufferView=inn, componentType=g.FLOAT, count=len(N), type=g.VEC3))
        na = len(accs) - 1
        if len(V) < 65536:
            idata, ct = F.astype("<u2").tobytes(), g.UNSIGNED_SHORT
        else:
            idata, ct = F.astype("<u4").tobytes(), g.UNSIGNED_INT
        ii = add_view(idata, g.ELEMENT_ARRAY_BUFFER)
        accs.append(g.Accessor(bufferView=ii, componentType=ct, count=F.size, type=g.SCALAR))
        xa = len(accs) - 1
        gmeshes.append(g.Mesh(name=name, primitives=[g.Primitive(
            attributes=g.Attributes(POSITION=ia, NORMAL=na), indices=xa, material=mat)]))
        nodes.append(g.Node(name=name, mesh=len(gmeshes) - 1, extras=extras.get(name, {})))
    mats = [
        g.Material(name="bone", pbrMetallicRoughness=g.PbrMetallicRoughness(
            baseColorFactor=[0.93, 0.91, 0.86, 1.0], metallicFactor=0.0, roughnessFactor=0.7)),
        g.Material(name="skin", alphaMode="BLEND", doubleSided=True,
                   pbrMetallicRoughness=g.PbrMetallicRoughness(
                       baseColorFactor=[0.85, 0.7, 0.62, 0.2], metallicFactor=0.0, roughnessFactor=0.9)),
    ]
    while len(blob) % 4:
        blob.append(0)
    gltf = g.GLTF2(
        asset=g.Asset(version="2.0", generator="BoneSim tools/build_assets.py",
                      copyright="BodyParts3D, (c) The Database Center for Life Science, CC BY-SA 2.1 JP (modified)"),
        scene=0, scenes=[g.Scene(name="BoneSim skeleton", nodes=list(range(len(nodes))))],
        nodes=nodes, meshes=gmeshes, materials=mats, accessors=accs, bufferViews=views,
        buffers=[g.Buffer(byteLength=len(blob))])
    gltf.set_binary_blob(bytes(blob))
    gltf.save_binary(path)


def skin_mesh(pitch=0.006):
    """Decimated OUTER body surface (FMA7163).  The BodyParts3D skin contains inner shells;
    keep only faces adjacent to the exterior (voxel flood fill from the bounding box)."""
    import fast_simplification
    import trimesh
    from scipy import ndimage
    V, F = geom.load_fma("FMA7163")
    V2, F2 = fast_simplification.simplify(V.astype(np.float32), F.astype(np.int32),
                                          target_reduction=1 - 200000 / len(F), agg=7)
    m = trimesh.Trimesh(V2, F2, process=True)
    lo = m.bounds[0] - 3 * pitch
    shape = np.ceil((m.bounds[1] + 3 * pitch - lo) / pitch).astype(int) + 1
    pts, _ = trimesh.sample.sample_surface(m, 3_000_000)
    occ = np.zeros(shape, bool)
    idx = np.floor((pts - lo) / pitch).astype(int)
    occ[idx[:, 0], idx[:, 1], idx[:, 2]] = True
    lab, _ = ndimage.label(~occ)
    exterior = lab == lab[0, 0, 0]
    c = m.triangles_center
    n = m.face_normals
    outer = np.zeros(len(c), bool)
    for sgn in (1, -1):
        for f in (1.0, 2.0):
            q = np.floor((c + sgn * f * pitch * n - lo) / pitch).astype(int)
            q = np.clip(q, 0, shape - 1)
            outer |= exterior[q[:, 0], q[:, 1], q[:, 2]]
    m.update_faces(outer)
    m.remove_unreferenced_vertices()
    comps = sorted(m.split(only_watertight=False), key=lambda c: c.area, reverse=True)
    keep = [cc for cc in comps if cc.area > 0.001 * comps[0].area]
    m = trimesh.util.concatenate(keep)
    return decimate(np.asarray(m.vertices), np.asarray(m.faces), TARGETS["skin"]), int(outer.size), int(outer.sum())


# ----------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stl", default=geom.STL_DIR)
    ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "assets"))
    ap.add_argument("--no-skin", action="store_true")
    a = ap.parse_args()
    geom.STL_DIR = a.stl
    out = os.path.abspath(a.out)
    os.makedirs(out, exist_ok=True)

    T = node_table()
    print(f"loading {len(T)} nodes ...", flush=True)
    B = {n: load_bone(d["fma"]) for n, d in T.items()}
    for n in T:  # side sanity: subject-left is +X
        cx = B[n].V[:, 0].mean()
        assert not (n.endswith("_L") and cx <= 0) and not (n.endswith("_R") and cx >= 0), n

    joints, lm, rest, pelvis_c = compute_rig(B)
    dims, restpose = compute_dims_rest(joints, lm, rest, pelvis_c, B)

    # segments
    segs = {}
    for n, d in T.items():
        segs.setdefault(d["segment"], []).append(n)
    parent = {"pelvis": (None, None), "L5": ("pelvis", "lumbosacral"), "L4": ("L5", "L4L5"),
              "L3": ("L4", "L3L4"), "L2": ("L3", "L2L3"), "L1": ("L2", "L1L2"), "thorax": ("L1", "T12L1"),
              "C7": ("thorax", "C7T1"), "C6": ("C7", "C6C7"), "C5": ("C6", "C5C6"), "C4": ("C5", "C4C5"),
              "C3": ("C4", "C3C4"), "C2": ("C3", "C2C3"), "C1": ("C2", "C1C2"), "skull": ("C1", "atlantooccipital")}
    for k in ("L", "R"):
        P = lambda s, j: (f"{s}_{k}" if s not in ("thorax", "pelvis") else s, f"{j}_{k}")  # noqa: E731
        parent.update({
            f"clavicle_{k}": P("thorax", "sternoclavicular"), f"scapula_{k}": P("clavicle", "acromioclavicular"),
            f"humerus_{k}": P("scapula", "glenohumeral"), f"ulna_{k}": P("humerus", "humeroulnar"),
            f"radius_{k}": P("ulna", "radioulnar"), f"carpalsProximal_{k}": P("radius", "radiocarpal"),
            f"hand_{k}": P("carpalsProximal", "midcarpal"), f"thumbMeta_{k}": P("hand", "cmc1"),
            f"thumbProx_{k}": P("thumbMeta", "mcp1"), f"thumbDist_{k}": P("thumbProx", "ip1"),
            f"femur_{k}": P("pelvis", "hip"), f"tibia_{k}": P("femur", "knee"),
            f"patella_{k}": P("femur", "patellofemoral"), f"talus_{k}": P("tibia", "talocrural"),
            f"foot_{k}": P("talus", "subtalar")})
        for i, f in enumerate(("index", "middle", "ring", "little"), start=2):
            parent[f"{f}Prox_{k}"] = P("hand", f"mcp{i}")
            parent[f"{f}Mid_{k}"] = P(f"{f}Prox", f"pip{i}")
            parent[f"{f}Dist_{k}"] = P(f"{f}Mid", f"dip{i}")
    assert set(parent) == set(segs), set(parent) ^ set(segs)
    segments = {s: dict(parent=parent[s][0], joint=parent[s][1], nodes=segs[s]) for s in parent}
    for s, d in segments.items():
        assert d["joint"] is None or d["joint"] in joints, (s, d["joint"])

    # decimate + GLB
    meshes, extras = [], {}
    stats = {}
    for n, d in T.items():
        Vd, Fd, Nd = decimate(B[n].V, B[n].F, target_for(n))
        meshes.append((n, Vd, Fd, Nd, 0))
        extras[n] = dict(segment=d["segment"], fma=",".join(d["fma"]))
        stats[n] = len(Fd)
    skin_info = None
    if not a.no_skin:
        (Vs, Fs, Ns), ncomp, nkeep = skin_mesh()
        meshes.append(("skin", Vs, Fs, Ns, 1))
        extras["skin"] = dict(segment=None, fma="FMA7163")
        skin_info = (len(Fs), ncomp, nkeep)
    glb = os.path.join(out, "skeleton.glb")
    write_glb(glb, meshes, extras)

    nodes = {n: dict(segment=d["segment"], nameJa=d["nameJa"], nameEn=d["nameEn"],
                     fma=d["fma"][0] if len(d["fma"]) == 1 else d["fma"], group=d["group"]) for n, d in T.items()}
    if skin_info:
        nodes["skin"] = dict(segment=None, nameJa="皮膚（体表）", nameEn="Skin (body surface, decimated)",
                             fma="FMA7163", group="skin")
    rig = {
        "units": "m", "frame": {"left": "+X", "up": "+Y", "forward": "+Z"},
        "source": ("BodyParts3D 3.0 (20110915), (c) The Database Center for Life Science, CC BY-SA 2.1 JP; "
                   "STL via github.com/Kevin-Mattheus-Moerman/BodyParts3D. Modified: re-oriented, scaled to metres, "
                   "decimated, grouped. See assets/LICENSE-ASSETS.md"),
        "transformFromBodyParts3D": "p_bonesim = (x, z, -y) / 1000 (BodyParts3D mm, +X left, +Y posterior, +Z superior)",
        "axisConvention": AXIS_CONVENTION,
        "restPoseNote": ("GLB vertices are at the BodyParts3D rest pose (≈ anatomical position, palms forward) in world "
                         "coordinates; nodes are NOT re-pivoted. Pivot each segment at joints[segment.joint].center."),
        "nodes": nodes, "segments": segments, "joints": joints, "landmarks": lm, "dims": dims,
        "dimsDefinitions": {
            "hipHalfWidth": "½·|x(hip_L) − x(hip_R)| (femoral head centres)",
            "thoraxHeight": "y(mid glenohumeral centres) − y(pelvisCenter)",
            "shoulderHalfWidth": "½·|x(GH_L) − x(GH_R)|",
            "head": "(y(vertex) − y(noseTip)) / 0.55  (so stickFK nose ≈ nose tip)",
            "neck": "y(vertex) − y(shoulder line) − head",
            "upperArm": "|GH − humeroulnar centre|", "forearm": "|humeroulnar − radiocarpal centre|",
            "hand": "|radiocarpal centre − middle fingertip|", "thigh": "|hip − knee centre|",
            "shank": "|knee − talocrural centre|", "footLength": "z(toe tip) − z(heel)",
            "ankleHeight": "y(talocrural centre) − y(sole)"},
        "restPose": restpose,
        "pelvisCenter": J(pelvis_c),
        "stats": {"glbBytes": os.path.getsize(glb), "triangles": int(sum(stats.values())),
                  "skinTriangles": skin_info[0] if skin_info else 0},
    }
    with open(os.path.join(out, "rig.json"), "w", encoding="utf-8") as f:
        json.dump(rig, f, ensure_ascii=False, indent=1)
    print(f"GLB {os.path.getsize(glb)/1e6:.2f} MB, {len(meshes)} meshes, {sum(stats.values())} bone tris, skin {skin_info}")
    print(f"segments {len(segments)}, joints {len(joints)}, landmarks {len(lm)}")
    print("dims", dims)
    print("restPose", json.dumps(restpose, ensure_ascii=False))


if __name__ == "__main__":
    main()
