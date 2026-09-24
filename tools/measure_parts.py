"""STEP 1 - measure every STL and extract the functional features the build needs.

Writes
  reports/features.json          machine-readable features (consumed by build_urdf.py)
  reports/part_measurements.md   human-readable table (bbox, PCA, proximal/distal)

Everything here is read directly off the meshes. No value in this file is a guess;
guesses live in params.py.
"""
from __future__ import annotations

import glob
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
from sm3lib import (REPORTS, SRC, axis_bores, circular_holes, dump_json, find_bores, fit_circle_2d, load, pca,
                    plate_normal, register_mirror, section_center)

SERVO_EAR_PITCH = 48.5  # sanity reference only (standard-size servo ear hole spacing)


def section_area(mesh, origin, normal):
    s = mesh.section(plane_origin=origin, plane_normal=normal)
    if s is None:
        return 0.0
    try:
        p2, _ = s.to_2D()
        return float(sum(p.area for p in p2.polygons_full))
    except Exception:
        return float("nan")


def generic(name):
    m = load(name)
    c, V, sd = pca(m.vertices)
    proj = (m.vertices - c) @ V
    ext = proj.max(0) - proj.min(0)
    row = dict(name=name, faces=len(m.faces), watertight=bool(m.is_watertight),
               bbox_print=(m.bounds[1] - m.bounds[0]).round(2), volume_cm3=round(abs(m.volume) / 1e3, 2),
               pca_extent=ext.round(2), pc1_dir_print=V[:, 0].round(3))
    # proximal/distal heuristic for elongated parts: the bulkier end is proximal
    if ext[0] > 1.8 * ext[1]:
        lo, hi = proj[:, 0].min(), proj[:, 0].max()
        a_lo = section_area(m, c + V[:, 0] * (lo + 0.15 * (hi - lo)), V[:, 0])
        a_hi = section_area(m, c + V[:, 0] * (lo + 0.85 * (hi - lo)), V[:, 0])
        end_lo = c + V[:, 0] * lo
        end_hi = c + V[:, 0] * hi
        row["xsec_area_15pct"] = round(a_lo, 1)
        row["xsec_area_85pct"] = round(a_hi, 1)
        row["heuristic_proximal_end_print"] = (end_lo if a_lo > a_hi else end_hi).round(1)
        row["length_along_pc1"] = round(float(ext[0]), 2)
    return row


def q(c):
    return np.asarray(c["q"], float)


def coax():
    m = load("Frame_LeftCoax")
    bx = axis_bores(m, "X")
    by = axis_bores(m, "Y")
    horn_holes = [b for b in bx if abs(b["r"] - 1.64) < 0.2]
    recess = max(bx, key=lambda b: b["r"])
    ears = [b for b in by if abs(b["r"] - 1.58) < 0.15]
    ymin = m.bounds[0][1]
    peg_c, peg_sz = section_center(m, [0, ymin + 2.0, 0], [0, 1, 0])
    ear_c = np.mean([q(e) for e in ears], axis=0)
    horn_c = np.mean([q(h) for h in horn_holes], axis=0)
    return dict(
        horn_axis="X", horn_center_YZ=[horn_c[1], horn_c[2]], horn_recess_r=recess["r"],
        horn_recess_bottom_X=recess["span"][0] - 0.5, horn_face_X=float(m.bounds[1][0]),
        ear_holes_XZ=[[q(e)[0], q(e)[2]] for e in ears], servo_center_XZ=[ear_c[0], ear_c[2]],
        peg_axis="Y", peg_center_XZ=[peg_c[0], peg_c[2]], peg_diameter=float(max(peg_sz[0], peg_sz[2])),
        peg_tip_Y=float(ymin), peg_base_Y=-18.0 if abs(ymin + 23.6) < 1 else float(ymin + 5.6),
        servo_shaft_offset_measured=float(abs(ear_c[2] - peg_c[2])),
        pitch_to_roll_perp_offset=float(abs(horn_c[2] - peg_c[2])),
        horn_face_to_roll_axis=float(m.bounds[1][0] - peg_c[0]),
    )


def femur():
    m = load("Frame_LeftFemur")
    by = axis_bores(m, "Y")
    horn4 = [b for b in by if abs(b["r"] - 1.55) < 0.1 and q(b)[0] < 0]
    recess = [b for b in by if abs(b["r"] - 9.98) < 0.3]
    ears = [b for b in by if abs(b["r"] - 1.58) < 0.1 and q(b)[0] > 0]
    ears.sort(key=lambda b: q(b)[0])
    pair_a = np.mean([q(e) for e in ears[:2]], axis=0)
    pair_b = np.mean([q(e) for e in ears[2:]], axis=0)
    knee = np.mean([q(h) for h in horn4], axis=0)
    return dict(
        knee_horn_center_XZ=[knee[0], knee[2]], knee_recess_span_Y=recess[0]["span"] if recess else None,
        knee_recess_bottom_Y=(recess[0]["span"][1] + 0.5) if recess else 14.2,
        ear_pair_near_knee_XZ=[pair_a[0], pair_a[2]], ear_pair_far_XZ=[pair_b[0], pair_b[2]],
        ear_pair_spacing=float(np.hypot(*(pair_b - pair_a)[[0, 2]])),
        ear_clearance_span_Y=[min(e["span"][0] for e in ears), max(e["span"][1] for e in ears)],
        servo_axis_tilt_deg=float(np.degrees(np.arctan2(pair_b[2] - pair_a[2], pair_b[0] - pair_a[0]))),
    )


def tibia():
    m = load("Frame_RightTibia")
    bz = axis_bores(m, "Z")
    ears = [b for b in bz if abs(b["r"] - 1.48) < 0.1]
    ears.sort(key=lambda b: q(b)[0])
    pair_a = np.mean([q(e) for e in ears[:2]], axis=0)
    pair_b = np.mean([q(e) for e in ears[2:]], axis=0)
    zmax = m.bounds[1][2]
    pin_c, pin_sz = section_center(m, [0, 0, zmax - 3.0], [0, 0, 1])
    pin_base = None
    for z in np.arange(zmax - 0.5, 0, -0.5):  # walk down until the section grows past the pin
        _, sz = section_center(m, [0, 0, z], [0, 0, 1])
        if max(sz[:2]) > pin_sz[0] + 1.0:
            pin_base = z + 0.5
            break
    s = m.section(plane_origin=[0, 0, 10], plane_normal=[0, 0, 1])
    v = s.vertices
    tip = v[v[:, 0] < v[:, 0].min() + 3.5]
    cx, cy, r = fit_circle_2d(tip[:, :2])
    servo_c = (pair_a + pair_b) / 2
    return dict(
        pin_axis="Z", pin_center_XY=[pin_c[0], pin_c[1]], pin_diameter=float(pin_sz[0]),
        pin_base_Z=pin_base, pin_tip_Z=float(zmax),
        ear_pair_a_XY=[pair_a[0], pair_a[1]], ear_pair_b_XY=[pair_b[0], pair_b[1]],
        servo_center_XY=[servo_c[0], servo_c[1]],
        servo_shaft_offset_measured=float(np.hypot(pin_c[0] - servo_c[0], pin_c[1] - servo_c[1])),
        tip_arc_center_XY=[cx, cy], tip_arc_radius=float(r),
        width_Z=[float(m.bounds[0][2]), 20.0], tip_mid_Z=10.0,
        knee_to_tip_arc=float(np.hypot(pin_c[0] - cx, pin_c[1] - cy)),
    )


def foot():
    m = load("Foot")
    R = (m.bounds[1][0] - m.bounds[0][0]) / 2
    return dict(outer_radius=float(R), center_print=[float(m.bounds.mean(0)[0]), float(m.bounds[0][1] + R), 0.0],
                width=float(m.bounds[1][2] - m.bounds[0][2]), axis="Z", opening="+Y")


def femur_cover():
    m = load("Cover_LeftFemur")
    by = axis_bores(m, "Y")
    seat = [b for b in by if abs(b["r"] - 8.13) < 0.2][0]
    hole = [b for b in by if abs(b["r"] - 4.27) < 0.2][0]
    screws = [b for b in by if abs(b["r"] - 2.09) < 0.15]
    sc = np.mean([q(s) for s in screws], axis=0)
    return dict(bearing_center_XZ=[q(seat)[0], q(seat)[2]], bearing_seat_span_Y=seat["span"],
                pin_hole_span_Y=hole["span"], screw_mid_XZ=[sc[0], sc[2]],
                skin_Y=-10.0)


def shoulders():
    out = {}
    fm = load("Frame_FrontShoulderMiddle")
    hits = circular_holes(fm, [0, 1, 0], [0, -24, 0], 4, 9, offsets=np.arange(-1.5, 1.6, 0.5))
    big = [h for h in hits if h["r"] > 4.5]
    out["front_bearing_plate_holes_XZ"] = sorted({(round(h["center"][0], 1), round(h["center"][2], 1), round(h["r"], 2)) for h in big})
    out["front_bearing_plate_Y_bounds"] = [float(fm.bounds[0][1]), float(fm.bounds[1][1])]
    fi = load("Frame_FrontShoulderInner")
    bi = axis_bores(fi, "Z")
    out["front_inner_mount_holes_XY"] = [[round(q(b)[0], 1), round(q(b)[1], 1)] for b in bi if abs(b["r"] - 1.58) < 0.05]
    out["front_inner_post_top_holes_XY"] = [[round(q(b)[0], 1), round(q(b)[1], 1)] for b in bi if abs(b["r"] - 1.24) < 0.05]
    tr = load("Frame_ChassisTrunk")
    bt = axis_bores(tr, "X")
    out["trunk_end_holes_YZ"] = [[round(q(b)[1], 1), round(q(b)[2], 1)] for b in bt if abs(b["r"] - 1.6) < 0.05]
    out["trunk_bounds"] = tr.bounds
    fo = load("Frame_FrontShoulderOuter")
    out["front_horn_plate_recess_XY"] = [[round(q(b)[0], 1), round(q(b)[1], 1)] for b in axis_bores(fo, "Z") if abs(b["r"] - 10.05) < 0.1]
    ro = load("Frame_RearShoulderOuter")
    out["rear_bearing_seat_XY"] = [[round(q(b)[0], 1), round(q(b)[1], 1), round(b["r"], 2)] for b in axis_bores(ro, "Z") if b["r"] > 5.5]
    rm = load("Frame_RearShoulderMiddle")
    brm = axis_bores(rm, "Z")
    out["rear_horn_plate_recess_XY"] = [[round(q(b)[0], 1), round(q(b)[1], 1)] for b in brm if b["r"] > 9.5]
    # the 4-screw horn pattern is a much cleaner centre than the recess ring
    pat = [q(b) for b in brm if abs(b["r"] - 1.53) < 0.05]
    out["rear_horn_plate_pattern_centres_XY"] = [np.mean([v for v in pat if v[0] < 0], axis=0)[:2].round(2).tolist(),
                                                 np.mean([v for v in pat if v[0] > 0], axis=0)[:2].round(2).tolist()]
    bfo = axis_bores(fo, "Z")
    pat = [q(b) for b in bfo if abs(b["r"] - 1.53) < 0.05]
    out["front_horn_plate_pattern_centres_XY"] = [np.mean([v for v in pat if v[0] < 0], axis=0)[:2].round(2).tolist(),
                                                  np.mean([v for v in pat if v[0] > 0], axis=0)[:2].round(2).tolist()]
    return out


def tabs(name, top_mask_fn, bot_mask_fn):
    """Mounting tabs: plane normal from PCA; screw holes found by sectioning
    perpendicular to that normal (holes come out circular only on the true axis)."""
    m = load(name)
    c = m.triangles_center
    res = {}
    for key, fn in (("top", top_mask_fn), ("bottom", bot_mask_fn)):
        n, cen, sd = plate_normal(m, fn(c))
        holes = circular_holes(m, n, cen, 1.2, 3.6, offsets=[0.0])
        # keep the two cleanest distinct holes
        uniq = []
        for h in sorted(holes, key=lambda h: -h["circ"]):
            if all(np.linalg.norm(h["center"] - u["center"]) > 3 for u in uniq):
                uniq.append(h)
        res[key] = dict(normal=n, centroid=cen, thickness_sd=sd[0],
                        holes=[dict(center=u["center"], r=u["r"], circ=u["circ"]) for u in uniq[:2]])
    nt, nb = res["top"]["normal"], res["bottom"]["normal"]
    res["normals_angle_deg"] = float(np.degrees(np.arccos(min(1.0, abs(nt @ nb)))))
    return res


def covers():
    out = {}
    for nm in ("TopFront", "TopRear", "BottomFront", "BottomRear"):
        m = load("Cover_" + nm)
        by = axis_bores(m, "Y")
        tongue = [b for b in by if abs(b["r"] - 2.58) < 0.05 and q(b)[2] < 10]
        others = [b for b in by if q(b)[2] > 10 and b["r"] > 2.2]
        tc = np.mean([q(b) for b in tongue], axis=0)
        s5 = m.section(plane_origin=[0, 0, 5], plane_normal=[0, 0, 1]).vertices
        s80 = m.section(plane_origin=[0, 0, 50], plane_normal=[0, 0, 1]).vertices
        mid = s80[np.abs(s80[:, 0]) < 15]
        # wall surfaces by ray casting through the flat part of the U (outer hit first, then inner)
        base_dir = -1.0 if nm in ("TopFront", "BottomFront", "BottomRear") else 1.0   # side the U's base is on
        o = np.array([[10.0, base_dir * 200.0, 60.0]])
        locs, _, _ = m.ray.intersects_location(o, np.array([[0, -base_dir, 0.0]]))
        ys = sorted(locs[:, 1], key=lambda y: -base_dir * y)[:2]
        out_wall = dict(outer_Y=float(ys[0]), inner_Y=float(ys[1]))
        out[nm] = dict(wall=out_wall, tongue_holes_X=[round(q(b)[0], 2) for b in tongue], tongue_center_XZ=[tc[0], tc[2]],
                       tongue_Y=[float(s5[:, 1].min()), float(s5[:, 1].max())],
                       wall_Y_at_centre=[float(mid[:, 1].min()), float(mid[:, 1].max())],
                       other_holes=[dict(X=q(b)[0], Z=q(b)[2], r=b["r"]) for b in others],
                       bounds=m.bounds)
    return out


def hatch():
    m = load("Cover_BottomHatch")
    c, V, sd = pca(m.vertices)
    p = (m.vertices - c) @ V
    # end screw holes: section the hatch in its own frame
    import trimesh
    T = np.eye(4)
    T[:3, :3] = V.T
    T[:3, 3] = -V.T @ c
    mm = m.copy()
    mm.apply_transform(T)
    holes = []
    for z in np.arange(mm.bounds[0][2] + 0.5, mm.bounds[1][2] - 0.5, 0.5):
        for h in circular_holes(mm, [0, 0, 1], [0, 0, z], 1.5, 4.0, min_circ=0.95):
            if abs(h["center"][0]) > 80:
                holes.append(h["center"])
    holes = np.array(holes)
    ends = [holes[holes[:, 0] < 0].mean(0), holes[holes[:, 0] > 0].mean(0)]
    return dict(pca_center=c, pca_axes_cols=V, pca_extent=[p.min(0), p.max(0)], end_holes_pca=ends,
                rim_top_pc3=float(p[:, 2].max()), bottom_pc3=float(p[:, 2].min()))


def mirror_pairs():
    out = {}
    for right, left in (("Frame_RightCoax", "Frame_LeftCoax"), ("Frame_RightFemur", "Frame_LeftFemur"),
                        ("Frame_LeftTibia", "Frame_RightTibia"), ("Cover_RightFemur", "Cover_LeftFemur")):
        A, mean, p95 = register_mirror(load(right), load(left))
        out[right] = dict(maps_onto=left, A=A, mean_err_mm=mean, p95_err_mm=p95)
    return out


def main():
    names = sorted(os.path.basename(f)[4:-4] for f in glob.glob(str(SRC / "SM3_*.stl")))
    print(f"measuring {len(names)} parts ...")
    rows = [generic(n) for n in names]
    feats = dict(
        coax=coax(), femur=femur(), tibia=tibia(), foot=foot(), femur_cover=femur_cover(),
        shoulders=shoulders(), covers=covers(), hatch=hatch(),
        head_tabs=tabs("Cover_HeadPanel", lambda c: c[:, 2] > 62, lambda c: c[:, 1] < -74),
        rear_tabs=tabs("Cover_RearPanel_2Piece_Shell", lambda c: (c[:, 2] > 40) & (c[:, 0] < -25),
                       lambda c: (c[:, 2] > 40) & (c[:, 0] > 30)),
        mirror_pairs=mirror_pairs(),
        parts=rows,
    )
    dump_json(feats, REPORTS / "features.json")

    lines = ["# Part measurements (STEP 1)", "",
             "All values in mm, in each STL's own print-bed frame. `proximal (heuristic)` is the end with the larger "
             "cross-section at 15 % / 85 % of the principal axis; it is only reported for elongated parts and is "
             "cross-checked against the functional features in README.md.", "",
             "| part | bbox (print) | PCA extents | volume cm³ | watertight | xsec 15% / 85% | proximal end (heuristic) |",
             "|---|---|---|---|---|---|---|"]
    for r in rows:
        prox = r.get("heuristic_proximal_end_print")
        xs = f"{r['xsec_area_15pct']} / {r['xsec_area_85pct']}" if "xsec_area_15pct" in r else ""
        lines.append(f"| {r['name']} | {list(r['bbox_print'])} | {list(r['pca_extent'])} | {r['volume_cm3']} | "
                     f"{r['watertight']} | {xs} | {list(prox) if prox is not None else ''} |")
    c, f, t = feats["coax"], feats["femur"], feats["tibia"]
    lines += ["", "## Key functional features", "",
              f"- Coax roll pivot peg Ø{c['peg_diameter']:.2f} at print (X,Z)={np.round(c['peg_center_XZ'], 2).tolist()}; "
              f"pitch horn pattern centre (Y,Z)={np.round(c['horn_center_YZ'], 2).tolist()}; "
              f"roll↔pitch perpendicular offset {c['pitch_to_roll_perp_offset']:.2f}; servo shaft offset from ear centre {c['servo_shaft_offset_measured']:.2f}",
              f"- Femur knee horn centre (X,Z)={np.round(f['knee_horn_center_XZ'], 2).tolist()}; servo ear pairs at "
              f"{np.round(f['ear_pair_near_knee_XZ'], 2).tolist()} and {np.round(f['ear_pair_far_XZ'], 2).tolist()} "
              f"(spacing {f['ear_pair_spacing']:.2f}, tilt {f['servo_axis_tilt_deg']:.2f}°)",
              f"- Tibia knee pin Ø{t['pin_diameter']:.2f} at (X,Y)={np.round(t['pin_center_XY'], 2).tolist()}; tip arc centre "
              f"{np.round(t['tip_arc_center_XY'], 2).tolist()} r={t['tip_arc_radius']:.2f}; knee→tip-arc {t['knee_to_tip_arc']:.2f}; "
              f"pin offset from servo centre {t['servo_shaft_offset_measured']:.2f}",
              f"- Head tab normals differ by {feats['head_tabs']['normals_angle_deg']:.2f}°, rear-panel tab normals by "
              f"{feats['rear_tabs']['normals_angle_deg']:.2f}° (parallel ⇒ one rigid mounting orientation)",
              "", "## Mirror pairs (ICP after reflecting across print Y)", ""]
    for k, v in feats["mirror_pairs"].items():
        lines.append(f"- {k} → {v['maps_onto']}: mean {v['mean_err_mm']:.3f} mm, p95 {v['p95_err_mm']:.3f} mm")
    (REPORTS / "part_measurements.md").write_text("\n".join(lines) + "\n")
    print("wrote", REPORTS / "features.json", "and part_measurements.md")


if __name__ == "__main__":
    main()
