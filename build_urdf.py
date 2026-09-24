"""STEPS 2-4 - re-orient every part into its link frame and write the URDF.

    python build_urdf.py            # uses reports/features.json (runs measure_parts.py if missing)

Outputs (all paths relative to this file, so the folder can live anywhere):
    spotmicro_sm3.urdf
    meshes/*.stl                  STL in millimetres, baked into their link frames (URDF scale 0.001)
    reports/build_summary.json    every derived length / offset / mass, with its provenance tag
"""
from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / "tools"))
import params as P  # noqa: E402
from sm3lib import (MESH_OUT, MIRROR_Y, REPORTS, axis_map, combine_boxes, dump_json, load, make_T,  # noqa: E402
                    rot_y)

S = MIRROR_Y  # mirror across the robot's sagittal plane (left leg -> right leg)
LEGS = {"FL": (+1, "front"), "FR": (-1, "front"), "RL": (+1, "rear"), "RR": (-1, "rear")}
SUMMARY: dict = {"notes": "mm unless noted; tags: MEASURED / INFERRED / ESTIMATED"}


def tag(key, value, prov):
    SUMMARY[key] = {"value": value, "tag": prov}
    return value


def features():
    fp = REPORTS / "features.json"
    if not fp.exists():
        subprocess.check_call([sys.executable, str(ROOT / "tools" / "measure_parts.py")])
    return json.loads(fp.read_text())


# =========================================================================== #
# LEG GEOMETRY (canonical LEFT leg; right leg = mirror)
# =========================================================================== #
def leg_geometry(F):
    c, f, t, ft = F["coax"], F["femur"], F["tibia"], F["foot"]
    G = {}

    # --- coax: carries the ROLL servo (roll axis = its Ø8 peg / servo spline line) and
    #     receives the PITCH servo horn in the round recess on its +X face.
    roll_x, roll_z = c["peg_center_XZ"]
    pitch_y, pitch_z = c["horn_center_YZ"]
    R_c = axis_map("X>+Y Y>+X Z>-Z")                       # INFERRED (see README "coax orientation")
    P0 = np.array([roll_x, pitch_y, roll_z])                # link origin: on roll axis, level with pitch axis
    G["T_coax"] = make_T(R_c, P0)
    pitch_pt = np.array([c["horn_recess_bottom_X"], pitch_y, pitch_z])
    G["hip_pitch_origin"] = R_c @ (pitch_pt - P0)
    tag("coax.roll_to_pitch_offset_z", float(-G["hip_pitch_origin"][2]), "MEASURED")
    tag("coax.roll_axis_to_pitch_horn_floor_y", float(G["hip_pitch_origin"][1]), "MEASURED")
    G["coax_peg_face_to_pitch_x"] = pitch_y - c["peg_base_Y"]
    G["roll_servo_center_print"] = np.array([c["servo_center_XZ"][0], c["peg_base_Y"] + 2.0 + P.SERVO_BOX[2] / 2,
                                             c["servo_center_XZ"][1]])
    G["roll_servo_dims_print"] = np.array([P.SERVO_BOX[1], P.SERVO_BOX[2], P.SERVO_BOX[0]])

    # --- femur: carries the PITCH servo (shaft on the servo centre line, offset toward the
    #     end away from the knee) and the knee horn recess at its other end.
    a, b = np.array(f["ear_pair_near_knee_XZ"]), np.array(f["ear_pair_far_XZ"])
    shaft_off = 0.5 * (c["servo_shaft_offset_measured"] + t["servo_shaft_offset_measured"])
    tag("servo.shaft_offset_from_body_centre", shaft_off, "MEASURED (mean of coax peg & tibia pin)")
    u = (b - a) / np.linalg.norm(b - a)
    shaft = (a + b) / 2 + u * shaft_off
    F0 = np.array([shaft[0], P.FEMUR_SPLINE_TOP_Y, shaft[1]])
    K = np.array([f["knee_horn_center_XZ"][0], f["knee_recess_bottom_Y"], f["knee_horn_center_XZ"][1]])
    R_f0 = axis_map("X>+Z Y>-Y Z>+X")                       # INFERRED
    v = R_f0 @ (K - F0)
    beta = np.degrees(np.arctan2(v[0], -v[2]))
    R_f = rot_y(beta) @ R_f0                                 # tilt so the knee lies on the femur's -Z axis
    G["T_femur"] = make_T(R_f, F0)
    G["knee_origin"] = R_f @ (K - F0)
    tag("femur.length_hip_to_knee", float(-G["knee_origin"][2]), "MEASURED")
    tag("femur.frame_tilt_deg", float(beta), "MEASURED")
    G["pitch_servo_center_print"] = np.array([(a[0] + b[0]) / 2, -8.0, (a[1] + b[1]) / 2])
    G["pitch_servo_dims_print"] = np.array([P.SERVO_BOX[0], P.SERVO_BOX[2], P.SERVO_BOX[1]])

    # --- tibia (canonical = RightTibia mirrored across print Y -> a left tibia):
    #     knee axis = its Ø8 idler pin (coaxial with the knee servo spline); foot = tip arc centre.
    px, py = t["pin_center_XY"]
    pin = np.array([px, -py, P.TIBIA_Z_AT_KNEE_PLANE])
    ax_, ay_ = t["tip_arc_center_XY"]
    footc = np.array([ax_, -ay_, t["tip_mid_Z"]])
    R_t0 = axis_map("X>+Z Y>+X Z>+Y")                       # INFERRED (pin points outboard)
    v = R_t0 @ (footc - pin)
    gamma = np.degrees(np.arctan2(v[0], -v[2]))
    R_t = rot_y(gamma) @ R_t0
    G["T_tibia_C"] = make_T(R_t, pin)                        # canonical (mirrored right tibia) -> link
    G["foot_origin"] = R_t @ (footc - pin)
    tag("tibia.length_knee_to_foot_centre", float(-G["foot_origin"][2]), "MEASURED")
    tag("tibia.frame_tilt_deg", float(gamma), "MEASURED")
    G["knee_servo_center_C"] = np.array([t["servo_center_XY"][0], -t["servo_center_XY"][1],
                                         t["pin_base_Z"] - P.SERVO_BOX[2] / 2])
    G["knee_servo_dims_C"] = np.array([P.SERVO_BOX[0], P.SERVO_BOX[1], P.SERVO_BOX[2]])

    # --- foot: half tyre, axis = print Z (-> link Y), opening +Y (-> link +Z, toward the knee)
    R_ft = axis_map("X>-X Y>+Z Z>+Y")
    G["T_foot"] = make_T(R_ft, ft["center_print"])
    G["foot_radius"] = tag("foot.tyre_outer_radius", ft["outer_radius"], "MEASURED")

    # --- femur cover: its Ø16.26 bearing seat is coaxial with the knee; its inner skin
    #     rests on the femur's outer face.
    fc = F["femur_cover"]
    cov = load("Cover_LeftFemur")
    fem = load("Frame_LeftFemur")
    skin_in = cov.section(plane_origin=[40, 0, 0], plane_normal=[1, 0, 0]).vertices[:, 1].max()
    fem_out = fem.section(plane_origin=[40, 0, 0], plane_normal=[1, 0, 0]).vertices[:, 1].min()
    d = np.array([f["knee_horn_center_XZ"][0] - fc["bearing_center_XZ"][0], fem_out - skin_in,
                  f["knee_horn_center_XZ"][1] - fc["bearing_center_XZ"][1]])
    G["T_cover_to_femur_print"] = make_T(np.eye(3), [0, 0, 0], d)
    tag("femur_cover.offset_in_femur_print", d, "INFERRED (bearing seat on knee axis, skin on femur face)")
    # consistency check: how far the tibia's Ø8 pin reaches into the cover's 688 bearing
    seat_lo, seat_hi = np.array(fc["bearing_seat_span_Y"]) + d[1]
    brg_face = seat_hi + 0.5                         # bearing's inner face (seat opens toward +Y)
    brg_far = brg_face - 5.0
    pin_y = [K[1] - (z - P.TIBIA_Z_AT_KNEE_PLANE) for z in (t["pin_base_Z"], t["pin_tip_Z"])]
    engage = min(brg_face, max(pin_y)) - max(brg_far, min(pin_y))
    tag("check.knee_pin_engagement_in_cover_bearing_mm", float(engage), "INFERRED check (5 mm = full)")
    screw_err = np.array(fc["screw_mid_XZ"]) + d[[0, 2]] - np.array(f["ear_pair_far_XZ"])
    tag("check.cover_screw_vs_femur_ear_holes_mm", float(np.linalg.norm(screw_err)), "INFERRED check")
    return G


# =========================================================================== #
# BODY: chassis trunk + shoulder stacks (base_link) and hip positions
# =========================================================================== #
def body_geometry(F):
    sh = F["shoulders"]
    B = {}
    trunk_b = np.array(sh["trunk_bounds"])
    half_len = float(trunk_b[1][0])
    # Inner plate <-> trunk: screw pattern (y=+-51.7, z_trunk=5/24) vs Inner (x=+-51.7, y=-7/12)
    th = np.array(sh["trunk_end_holes_YZ"])
    ih = np.array(sh["front_inner_mount_holes_XY"])
    inner_c_zt = float(np.mean(np.sort(np.unique(th[:, 1]))) - np.mean(np.sort(np.unique(ih[:, 1]))))
    brg = np.array(sh["front_bearing_plate_holes_XZ"])
    roll_zt = inner_c_zt - float(np.mean(brg[:, 1]))          # bearing plate Z maps to -z
    B["z_off"] = roll_zt                                       # base z = trunk z - z_off
    tag("body.front_roll_axis_above_trunk_floor", roll_zt, "INFERRED (trunk↔inner↔bearing-plate screw patterns)")
    zc = inner_c_zt - roll_zt                                  # front shoulder centre, base z
    hip_y = tag("body.hip_roll_axis_half_spacing", float(np.mean(np.abs(brg[:, 0]))), "MEASURED")
    inner_len = 48.98
    x_bp_in = half_len + inner_len                             # front bearing plate inboard face
    bp = sh["front_bearing_plate_Y_bounds"]
    x_bay_front = x_bp_in + 5.0
    parts = []
    parts.append(("Frame_ChassisTrunk", make_T(np.eye(3), [0, 0, 0], [0, 0, -roll_zt])))
    parts.append(("Frame_FrontShoulderInner", make_T(axis_map("X>+Y Y>+Z Z>+X"), [0, 0, 0], [half_len, 0, zc])))
    parts.append(("Frame_FrontShoulderMiddle", make_T(axis_map("X>+Y Y>+X Z>-Z"), [0, bp[0], 0], [x_bp_in, 0, zc])))
    x_flange_end = x_bp_in + (bp[1] - bp[0])
    parts.append(("Frame_FrontShoulderOuter", make_T(axis_map("X>+Y Y>-Z Z>-X"), [0, 0, 5.0], [x_flange_end, 0, zc])))
    # rear stack: Inner -> horn plate ("RearShoulderMiddle") -> flanges -> bearing plate ("RearShoulderOuter")
    rih_zt_of_Y = lambda Y: 14.0 - Y                           # rear inner holes Y=9/-10 <-> z_trunk 5/24
    parts.append(("Frame_RearShoulderInner", make_T(axis_map("X>+Y Y>-Z Z>-X"), [4.05, 0, 0],
                                                     [-half_len, 0, rih_zt_of_Y(0) - roll_zt])))
    x_hp = -half_len - inner_len                               # horn plate inboard face
    # post-hole match: horn-plate Y = inner Y + 7.1  -> z_trunk = 21.1 - Y
    rear_roll_zt = 21.1 - float(np.mean([h[1] for h in sh["rear_horn_plate_pattern_centres_XY"]]))
    rear_hc = np.array(sh["rear_horn_plate_pattern_centres_XY"])
    parts.append(("Frame_RearShoulderMiddle", make_T(axis_map("X>+Y Y>-Z Z>-X"), [float(rear_hc[:, 0].mean()), 0, 0],
                                                      [x_hp, 0, 21.1 - roll_zt])))
    rs = np.array(sh["rear_bearing_seat_XY"])
    rs = rs[np.abs(np.abs(rs[:, 0]) - 39) < 2]
    seat_Y = float(np.mean(rs[:, 1]))
    x_rbp_out = x_hp - 5.0 - 52.5                              # rear bearing plate outboard face
    parts.append(("Frame_RearShoulderOuter", make_T(axis_map("X>+Y Y>+Z Z>+X"), [0, seat_Y, 0],
                                                     [x_rbp_out, 0, rear_roll_zt - roll_zt])))
    x_bay_rear = x_rbp_out + 5.0
    tag("body.rear_roll_axis_z_rel_front", rear_roll_zt - roll_zt, "INFERRED")

    B["parts"] = parts
    B["zc_front"] = zc
    B["zc_rear"] = (rear_roll_zt - roll_zt) - seat_Y         # centre of the rear bearing plate (base z)
    B["x_front_flange_holes"] = x_bp_in + (-4.55 - bp[0])      # Middle flange screw holes at print Y=-4.55
    B["x_rear_flange_holes"] = x_rbp_out + 35.95
    B["hip_y"] = hip_y
    B["x_bay_front"], B["x_bay_rear"] = x_bay_front, x_bay_rear
    B["rear_roll_z"] = rear_roll_zt - roll_zt
    return B


def hip_positions(B, G):
    peg_to_pitch = G["coax_peg_face_to_pitch_x"]
    hp = dict(
        front_x=B["x_bay_front"] + P.BEARING_PROTRUSION + peg_to_pitch,   # peg faces the inboard bearing plate
        rear_x=B["x_bay_rear"] + P.BEARING_PROTRUSION + peg_to_pitch,     # peg faces the outboard bearing plate
        y=B["hip_y"], front_z=0.0, rear_z=B["rear_roll_z"])
    if P.HIP_OVERRIDE:
        hp.update(P.HIP_OVERRIDE)
    for k, v in hp.items():
        tag(f"hip.{k}", float(v), "INFERRED" if not P.HIP_OVERRIDE else "USER OVERRIDE")
    return hp


# =========================================================================== #
# SHELL PANELS (fixed links on base_link)
# =========================================================================== #
def panel_geometry(F, B):
    out = []
    zc_f = B["zc_front"]
    zc_r = B["zc_rear"]
    xf, xr = B["x_front_flange_holes"], B["x_rear_flange_holes"]
    # ---- head panel: tab normals give "up"; print -X is lateral (tab holes +-20 / +-5)
    ht = F["head_tabs"]
    ztop = np.array(ht["top"]["normal"])
    ztop = ztop if ztop @ (np.array(ht["top"]["centroid"]) - ht["bottom"]["centroid"]) > 0 else -ztop
    zbot = -np.array(ht["bottom"]["normal"])
    zbot = zbot if zbot @ ztop > 0 else -zbot
    Zr = (ztop + zbot) / np.linalg.norm(ztop + zbot)
    Yr = np.array([-1.0, 0, 0]) - Zr * (-Zr[0])
    Yr /= np.linalg.norm(Yr)
    Xr = np.cross(Yr, Zr)
    R = np.vstack([Xr, Yr, Zr])
    holes = np.array([h["center"] for k in ("top", "bottom") for h in ht[k]["holes"]])
    hx = float(np.mean(holes @ Xr))
    hy = float(np.mean(holes @ Yr))
    hz = float(np.mean([np.array(ht[k]["centroid"]) @ Zr for k in ("top", "bottom")]))
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.array([xf - hx, -hy, zc_f - hz])
    out.append(("cover_head", "Cover_HeadPanel", T))
    bed_n = -R[:, 2]                                  # outward normal of the print-bed face, in robot axes
    tag("head.front_face_normal_below_horizontal_deg", float(np.degrees(np.arcsin(-bed_n[2]))),
        "MEASURED (tab normals / circular screw holes)")

    # ---- rear panel shell (+ its inner base plate): same method
    rt = F["rear_tabs"]
    ztop = -np.array(rt["top"]["normal"])
    ztop = ztop if ztop @ (np.array(rt["top"]["centroid"]) - rt["bottom"]["centroid"]) > 0 else -ztop
    zbot = -np.array(rt["bottom"]["normal"])
    zbot = zbot if zbot @ ztop > 0 else -zbot
    Zr = (ztop + zbot) / np.linalg.norm(ztop + zbot)
    Yr = np.array([0, 1.0, 0]) - Zr * Zr[1]
    Yr /= np.linalg.norm(Yr)
    Xr = np.cross(Yr, Zr)
    R = np.vstack([Xr, Yr, Zr])
    holes = np.array([h["center"] for k in ("top", "bottom") for h in rt[k]["holes"]])
    hx, hy = float(np.mean(holes @ Xr)), float(np.mean(holes @ Yr))
    hz = float(np.mean([np.array(rt[k]["centroid"]) @ Zr for k in ("top", "bottom")]))
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.array([xr - hx, -hy, zc_r - hz])
    out.append(("cover_rear_shell", "Cover_RearPanel_2Piece_Shell", T))
    out.append(("cover_rear_base", "Cover_RearPanel_2Piece_Base", T @ make_T(np.eye(3), [0, 0, 0], [0, 0, 10.3])))
    bed_n = -R[:, 2]
    tag("rear_panel.back_face_normal_below_horizontal_deg", float(np.degrees(np.arcsin(-bed_n[2]))),
        "MEASURED (tab normals / circular screw holes)")

    # ---- U-section covers: tongue screw holes share the flange screws with the panel tabs.
    C = F["covers"]
    tab_t = 2.9                       # tab thickness implied by tab-plane separation (68.9) - flange span (66)
    top_f = zc_f + 33.0 + tab_t       # top of the head tab stack, front
    top_r = zc_r + 33.0 + tab_t
    bot_f = zc_f - 33.0 - tab_t
    bot_r = zc_r - 33.0 - tab_t
    specs = [
        # link, part, axis map, print point, base point
        ("cover_top_front", "TopFront", "X>+Y Y>-Z Z>-X",
         [C["TopFront"]["tongue_center_XZ"][0], C["TopFront"]["tongue_Y"][1], C["TopFront"]["tongue_center_XZ"][1]], [xf, 0, top_f]),
        ("cover_top_rear", "TopRear", "X>+Y Y>+Z Z>+X",
         [C["TopRear"]["tongue_center_XZ"][0], C["TopRear"]["tongue_Y"][0], C["TopRear"]["tongue_center_XZ"][1]], [xr, 0, top_r]),
        ("cover_bottom_front", "BottomFront", "X>-Y Y>+Z Z>-X",
         [C["BottomFront"]["tongue_center_XZ"][0], C["BottomFront"]["tongue_Y"][1], C["BottomFront"]["tongue_center_XZ"][1]], [xf, 0, bot_f]),
        ("cover_bottom_rear", "BottomRear", "X>+Y Y>+Z Z>+X",
         [C["BottomRear"]["tongue_center_XZ"][0], C["BottomRear"]["tongue_Y"][1], C["BottomRear"]["tongue_center_XZ"][1]], [xr, 0, bot_r]),
    ]
    placed = {}
    for link, nm, amap, p_src, p_dst in specs:
        T = make_T(axis_map(amap), p_src, p_dst)
        out.append((link, "Cover_" + nm, T))
        placed[nm] = T
    # seam checks: top cover side-walls must overlap the bottom cover side-walls
    def zrange(nm):
        b = np.array(C[nm]["bounds"])
        pts = trimesh.transform_points(np.array([[0, b[0][1], 60], [0, b[1][1], 60]]), placed[nm])
        return sorted(pts[:, 2])
    # the covers' inner wall surface should rest on the shoulder plates' outer faces
    sh_top = {"front": B["zc_front"] + 35.0, "rear": B["zc_rear"] + 35.0}
    sh_bot = {"front": B["zc_front"] - 35.0, "rear": B["zc_rear"] - 35.0}
    for nm, end, which in (("TopFront", "front", 1), ("TopRear", "rear", 1), ("BottomFront", "front", 0), ("BottomRear", "rear", 0)):
        inner = trimesh.transform_points(np.array([[0, C[nm]["wall"]["inner_Y"], 60]]), placed[nm])[0, 2]
        ref = sh_top[end] if which else sh_bot[end]
        tag(f"check.{nm}_inner_wall_vs_shoulder_face_mm", float(inner - ref), "INFERRED check (+ = gap, - = overlap)")
    tag("check.front_side_seam_overlap_mm", float(zrange("BottomFront")[1] - zrange("TopFront")[0]), "INFERRED check")
    tag("check.rear_side_seam_overlap_mm", float(zrange("BottomRear")[1] - zrange("TopRear")[0]), "INFERRED check")

    # ---- belly hatch: principal axes; end screw holes <-> the bottom covers' centre bosses
    H = F["hatch"]
    V = np.array(H["pca_axes_cols"])
    cen = np.array(H["pca_center"])
    bf = C["BottomFront"]["other_holes"][0]
    br = [h for h in C["BottomRear"]["other_holes"] if h["r"] < 3][0]
    pf = trimesh.transform_points([[bf["X"], 0, bf["Z"]]], placed["BottomFront"])[0]
    pr = trimesh.transform_points([[br["X"], 0, br["Z"]]], placed["BottomRear"])[0]
    ends = np.array(H["end_holes_pca"])
    mid_pca = ends.mean(0)
    cover_bottom = min(trimesh.transform_points(np.array([[0, C[n]["wall"]["outer_Y"], 60]]), placed[n])[0][2]
                       for n in ("BottomFront", "BottomRear"))
    R = V.T                                         # pca axes -> base axes (pc1=X, pc2=Y, pc3=Z up, rim on top)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = -R @ cen + np.array([(pf[0] + pr[0]) / 2 - mid_pca[0], -mid_pca[1], cover_bottom - H["rim_top_pc3"]])
    out.append(("cover_belly_hatch", "Cover_BottomHatch", T))
    tag("check.hatch_hole_spacing_vs_cover_bosses_mm",
        float(abs(np.linalg.norm(ends[1] - ends[0]) - abs(pf[0] - pr[0]))), "INFERRED check")
    return out


# =========================================================================== #
# mesh export + inertia
# =========================================================================== #
_mesh_cache = {}


def baked(stem, T, out_name, decimate=True):
    m = load(stem).copy()
    if decimate and len(m.faces) > P.VISUAL_MAX_FACES:
        try:
            m = m.simplify_quadric_decimation(face_count=P.VISUAL_MAX_FACES)
        except Exception as e:  # pragma: no cover
            print("  (decimation skipped:", e, ")")
    m.apply_transform(T)
    if np.linalg.det(T[:3, :3]) < 0:
        raise RuntimeError(f"improper transform for {stem}")
    m.fix_normals()
    MESH_OUT.mkdir(exist_ok=True)
    path = MESH_OUT / f"{out_name}.stl"
    m.export(path)
    return path, m


def density_for(stem):
    if stem == "Foot":
        return P.PRINT_DENSITY_G_CM3["foot"]
    return P.PRINT_DENSITY_G_CM3["cover" if stem.startswith("Cover_") else "frame"]


def printed_component(stem, T):
    """(mass kg, com m, bbox dims m) of a printed part after transform T (mm)."""
    m = load(stem).copy()
    m.apply_transform(T)
    mass = abs(m.volume) / 1000.0 * density_for(stem) / 1000.0
    com = m.center_mass if m.is_watertight else m.bounds.mean(0)
    dims = m.bounds[1] - m.bounds[0]
    return mass, com / 1000.0, dims / 1000.0


def servo_component(center_print, dims_print, T):
    c = trimesh.transform_points([center_print], T)[0]
    dims = np.abs(T[:3, :3]) @ np.asarray(dims_print)
    return P.SERVO_MASS, c / 1000.0, dims / 1000.0


# =========================================================================== #
# URDF writer
# =========================================================================== #
def f3(v, scale=1e-3):
    return " ".join(f"{x * scale:.6g}" for x in v)


def rpy_of(R):
    sy = np.hypot(R[0, 0], R[1, 0])
    if sy > 1e-9:
        return np.array([np.arctan2(R[2, 1], R[2, 2]), np.arctan2(-R[2, 0], sy), np.arctan2(R[1, 0], R[0, 0])])
    return np.array([np.arctan2(-R[1, 2], R[1, 1]), np.arctan2(-R[2, 0], sy), 0.0])


def inertial_xml(M, com_m, I):
    return (f'    <inertial>\n      <origin xyz="{f3(com_m, 1)}" rpy="0 0 0"/>\n      <mass value="{M:.6g}"/>\n'
            f'      <inertia ixx="{I[0,0]:.6e}" ixy="{I[0,1]:.6e}" ixz="{I[0,2]:.6e}" iyy="{I[1,1]:.6e}" '
            f'iyz="{I[1,2]:.6e}" izz="{I[2,2]:.6e}"/>\n    </inertial>\n')


def visual_xml(mesh_path, color, name=None):
    rel = mesh_path.relative_to(ROOT).as_posix()
    nm = f' name="{name}"' if name else ""
    return (f'    <visual{nm}>\n      <origin xyz="0 0 0" rpy="0 0 0"/>\n      <geometry><mesh filename="{rel}" '
            f'scale="0.001 0.001 0.001"/></geometry>\n      <material name="{color}"/>\n    </visual>\n')


def box_collision_xml(lo, hi):
    c, d = (np.asarray(lo) + hi) / 2, np.asarray(hi) - lo
    return (f'    <collision>\n      <origin xyz="{f3(c)}" rpy="0 0 0"/>\n      <geometry><box size="{f3(d)}"/></geometry>\n'
            f'    </collision>\n')


def sphere_collision_xml(r):
    return (f'    <collision>\n      <origin xyz="0 0 0" rpy="0 0 0"/>\n      <geometry><sphere radius="{r / 1000:.6g}"/></geometry>\n'
            f'    </collision>\n')


def joint_xml(name, jtype, parent, child, xyz_mm, axis=None, limits=None):
    s = f'  <joint name="{name}" type="{jtype}">\n    <parent link="{parent}"/>\n    <child link="{child}"/>\n'
    s += f'    <origin xyz="{f3(xyz_mm)}" rpy="0 0 0"/>\n'
    if axis is not None:
        s += f'    <axis xyz="{axis}"/>\n'
        lo, hi = limits
        s += (f'    <limit lower="{lo}" upper="{hi}" effort="{P.JOINT_EFFORT}" velocity="{P.JOINT_VELOCITY}"/>\n'
              f'    <dynamics damping="{P.JOINT_DAMPING}" friction="{P.JOINT_FRICTION}"/>\n')
    return s + "  </joint>\n"


def main():
    F = features()
    for old in MESH_OUT.glob("*.stl"):
        old.unlink()
    G = leg_geometry(F)
    B = body_geometry(F)
    hp = hip_positions(B, G)
    panels = panel_geometry(F, B)
    A = {k: np.array(v["A"]) for k, v in F["mirror_pairs"].items()}

    links, joints = [], []
    mats = "".join(f'  <material name="{k}"><color rgba="{" ".join(map(str, v))}"/></material>\n'
                   for k, v in P.COLORS.items())

    # ---------------- base_link ----------------
    vis, comps = "", []
    all_lo, all_hi = np.full(3, 1e9), np.full(3, -1e9)
    for stem, T in B["parts"]:
        path, m = baked(stem, T, f"base__{stem[6:]}")
        vis += visual_xml(path, P.PART_COLOR.get(stem, "black"), stem[6:])
        comps.append(printed_component(stem, T))
        all_lo, all_hi = np.minimum(all_lo, m.bounds[0]), np.maximum(all_hi, m.bounds[1])
    e = P.ELECTRONICS
    comps.append((e["mass"], np.array(e["com"]) / 1000, np.array(e["box"]) / 1000))
    M, com, I = combine_boxes(comps)
    panel_links = []
    for link, stem, T in panels:
        c = trimesh.transform_points([load(stem).bounds.mean(0)], T)[0]
        Tl = make_T(np.eye(3), c) @ T
        path, m = baked(stem, Tl, link)
        pm, pc, pd = printed_component(stem, Tl)
        lo, hi = m.bounds[0] + c, m.bounds[1] + c
        all_lo, all_hi = np.minimum(all_lo, lo), np.maximum(all_hi, hi)
        panel_links.append((link, stem, c, path, (pm, pc, pd)))
    links.append(f'  <link name="base_link">\n{inertial_xml(M, com, I)}{vis}{box_collision_xml(all_lo, all_hi)}  </link>\n')
    tag("mass.base_link_kg", M, "ESTIMATED")
    tag("body.bounding_box_mm", [all_lo.round(1).tolist(), all_hi.round(1).tolist()], "MEASURED (assembled)")
    for link, stem, c, path, (pm, pc, pd) in panel_links:
        Mp, cp, Ip = combine_boxes([(pm, pc, pd)])
        links.append(f'  <link name="{link}">\n{inertial_xml(Mp, cp, Ip)}{visual_xml(path, "yellow")}  </link>\n')
        joints.append(joint_xml(f"{link}_fixed", "fixed", "base_link", link, c))
        tag(f"mass.{link}_kg", Mp, "ESTIMATED")

    # ---------------- legs ----------------
    L = P.JOINT_LIMITS
    leg_masses = {}
    for leg, (side, end) in LEGS.items():
        sfx = "L" if side > 0 else "R"
        # file -> canonical maps
        g_coax = np.eye(4) if side > 0 else A["Frame_RightCoax"]
        g_fem = np.eye(4) if side > 0 else A["Frame_RightFemur"]
        g_cov = np.eye(4) if side > 0 else A["Cover_RightFemur"]
        g_tib = (MIRROR_Y @ A["Frame_LeftTibia"]) if side > 0 else MIRROR_Y
        mir = np.eye(4) if side > 0 else S
        coax_stem = f"Frame_{'Left' if side > 0 else 'Right'}Coax"
        fem_stem = f"Frame_{'Left' if side > 0 else 'Right'}Femur"
        tib_stem = f"Frame_{'Left' if side > 0 else 'Right'}Tibia"
        cov_stem = f"Cover_{'Left' if side > 0 else 'Right'}Femur"

        T_coax = mir @ G["T_coax"] @ g_coax
        T_fem = mir @ G["T_femur"] @ g_fem
        T_cov = mir @ G["T_femur"] @ G["T_cover_to_femur_print"] @ g_cov
        T_tib = mir @ G["T_tibia_C"] @ g_tib
        T_ft = G["T_foot"] if side > 0 else make_T(np.diag([-1.0, -1.0, 1.0]), [0, 0, 0]) @ G["T_foot"]
        # servo pose in each link (canonical coords -> link)
        sv_coax = servo_component(G["roll_servo_center_print"], G["roll_servo_dims_print"], mir @ G["T_coax"])
        sv_fem = servo_component(G["pitch_servo_center_print"], G["pitch_servo_dims_print"], mir @ G["T_femur"])
        sv_tib = servo_component(G["knee_servo_center_C"], G["knee_servo_dims_C"], mir @ G["T_tibia_C"])

        def mk(link, stem, T, servo=None, collision="box", extra_vis=""):
            path, m = baked(stem, T, f"{sfx}_{link}")
            comps = [printed_component(stem, T)] + ([servo] if servo else [])
            Mk, ck, Ik = combine_boxes(comps)
            col = ""
            if collision == "box":
                col = box_collision_xml(m.bounds[0], m.bounds[1])
            elif collision == "tibia":
                lo, hi = m.bounds[0].copy(), m.bounds[1].copy()
                lo[2] = G["foot_origin"][2] + 20.0      # stop above the tyre so the foot sphere makes contact
                col = box_collision_xml(lo, hi)
            elif collision == "sphere":
                col = sphere_collision_xml(G["foot_radius"])
            color = P.PART_COLOR.get(link, "black")
            links.append(f'  <link name="{leg}_{link}">\n{inertial_xml(Mk, ck, Ik)}'
                         f'{visual_xml(path, color)}{extra_vis}{col}  </link>\n')
            return Mk

        m1 = mk("coax", coax_stem, T_coax, sv_coax)
        cov_path, _ = baked(cov_stem, T_cov, f"{sfx}_femur_cover")
        # femur cover rides on the femur link (fixed) - give it its own link for clarity
        m2 = mk("femur", fem_stem, T_fem, sv_fem)
        cm, cc, cd = printed_component(cov_stem, T_cov)
        Mc, ccm, Ic = combine_boxes([(cm, cc, cd)])
        links.append(f'  <link name="{leg}_femur_cover">\n{inertial_xml(Mc, ccm, Ic)}{visual_xml(cov_path, "yellow")}  </link>\n')
        m3 = mk("tibia", tib_stem, T_tib, sv_tib, collision="tibia")
        m4 = mk("foot", "Foot", T_ft, None, collision="sphere")
        leg_masses[leg] = m1 + m2 + Mc + m3 + m4

        hx = hp["front_x"] if end == "front" else hp["rear_x"]
        hz = hp["front_z"] if end == "front" else hp["rear_z"]
        yflip = np.array([1, side, 1.0])
        joints.append(joint_xml(f"{leg}_hip_roll", "revolute", "base_link", f"{leg}_coax",
                                [hx, side * hp["y"], hz], "1 0 0", L["hip_roll"]))
        joints.append(joint_xml(f"{leg}_hip_pitch", "revolute", f"{leg}_coax", f"{leg}_femur",
                                G["hip_pitch_origin"] * yflip, "0 1 0", L["hip_pitch"]))
        joints.append(joint_xml(f"{leg}_femur_cover_fixed", "fixed", f"{leg}_femur", f"{leg}_femur_cover", [0, 0, 0]))
        joints.append(joint_xml(f"{leg}_knee", "revolute", f"{leg}_femur", f"{leg}_tibia",
                                G["knee_origin"] * yflip, "0 1 0", L["knee"]))
        joints.append(joint_xml(f"{leg}_foot_fixed", "fixed", f"{leg}_tibia", f"{leg}_foot",
                                G["foot_origin"] * yflip))
    tag("mass.per_leg_kg", leg_masses, "ESTIMATED")
    total = SUMMARY["mass.base_link_kg"]["value"] + sum(SUMMARY[k]["value"] for k in SUMMARY if k.startswith("mass.cover_")) \
        + sum(leg_masses.values())
    tag("mass.total_kg", total, "ESTIMATED")
    foot_y = hp["y"] + G["hip_pitch_origin"][1] + G["knee_origin"][1] + G["foot_origin"][1]
    tag("leg.foot_lateral_offset_from_centreline", float(foot_y), "INFERRED")

    urdf = (f'<?xml version="1.0"?>\n<!-- Generated by build_urdf.py - do not edit by hand; edit params.py and rebuild. -->\n'
            f'<robot name="{P.ROBOT_NAME}">\n{mats}' + "".join(links) + "".join(joints) + "</robot>\n")
    out = ROOT / f"{P.ROBOT_NAME}.urdf"
    out.write_text(urdf)
    dump_json(SUMMARY, REPORTS / "build_summary.json")
    print(f"wrote {out.name}: {len(links)} links, {len(joints)} joints, {len(list(MESH_OUT.glob('*.stl')))} meshes")
    for k in ("hip.front_x", "hip.rear_x", "hip.y", "coax.roll_to_pitch_offset_z", "coax.roll_axis_to_pitch_horn_floor_y",
              "femur.length_hip_to_knee", "tibia.length_knee_to_foot_centre", "leg.foot_lateral_offset_from_centreline",
              "mass.total_kg", "check.knee_pin_engagement_in_cover_bearing_mm", "check.cover_screw_vs_femur_ear_holes_mm",
              "check.front_side_seam_overlap_mm", "check.rear_side_seam_overlap_mm",
              "check.hatch_hole_spacing_vs_cover_bosses_mm", "check.TopFront_inner_wall_vs_shoulder_face_mm",
              "check.TopRear_inner_wall_vs_shoulder_face_mm", "check.BottomFront_inner_wall_vs_shoulder_face_mm",
              "check.BottomRear_inner_wall_vs_shoulder_face_mm", "hip.rear_z"):
        v = SUMMARY[k]["value"]
        print(f"  {k:52s} {np.round(v, 2) if not isinstance(v, dict) else {a: round(b, 3) for a, b in v.items()}}")


if __name__ == "__main__":
    main()
