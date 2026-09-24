"""STEPS 5-6 - load the URDF in PyBullet, check topology, run a real gravity simulation,
verify the COM stays inside the support polygon, and render images for comparison with
the reference photo.

    python validate_pybullet.py                 # headless (DIRECT), writes reports/
    python validate_pybullet.py --photo ref.png # also builds a side-by-side with your photo

Exit code 0 = all checks passed.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
import pybullet as p
import pybullet_data

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent / "tools"))
import params as P  # noqa: E402
from sm3_sim import LEGS, ROOT, joint_map, load_robot, set_pose, stance_angles, stance_height, whole_body_com  # noqa: E402

REP = ROOT / "reports"


def check(results, name, ok, detail):
    results.append({"check": name, "pass": bool(ok), "detail": detail})
    print(f"[{'PASS' if ok else 'FAIL'}] {name}: {detail}")


def topology(results):
    rid = load_robot(fixed=True)
    jm = joint_map(rid)
    rev = [n for n, j in jm.items() if j["type"] == p.JOINT_REVOLUTE]
    fixed = [n for n, j in jm.items() if j["type"] == p.JOINT_FIXED]
    check(results, "12 revolute joints", len(rev) == 12, f"{len(rev)} revolute, {len(fixed)} fixed, {len(jm)} total")
    ok, bad = True, []
    names = {j["index"]: n for n, j in jm.items()}
    for leg in LEGS:
        chain = [f"{leg}_hip_roll", f"{leg}_hip_pitch", f"{leg}_knee", f"{leg}_foot_fixed"]
        axes = [(1, 0, 0), (0, 1, 0), (0, 1, 0)]
        for a, b in zip(chain, chain[1:]):
            if names.get(jm[b]["parent"]) != a:
                ok = False
                bad.append(f"{b} parent is {names.get(jm[b]['parent'])}")
        if jm[chain[0]]["parent"] != -1:
            ok = False
            bad.append(f"{chain[0]} not on base")
        for jn, ax in zip(chain, axes):
            # PyBullet reports the axis in the child's inertial frame; rotate it to world
            R = np.array(p.getMatrixFromQuaternion(p.getLinkState(rid, jm[jn]["index"])[1])).reshape(3, 3)
            w = R @ np.array(jm[jn]["axis"])
            if not np.allclose(w, ax, atol=1e-6):
                ok = False
                bad.append(f"{jn} world axis {np.round(w, 4)}")
    check(results, "per-leg chain base→coax→femur→tibia→foot, axes X/Y/Y", ok, "ok" if ok else "; ".join(bad))
    # link lengths from forward kinematics at zero pose
    L = {}
    for leg in LEGS:
        ks = [p.getLinkState(rid, jm[f"{leg}_{n}"]["index"])[4] for n in ("hip_pitch", "knee", "foot_fixed")]
        xz = lambda a, b: np.linalg.norm(np.subtract(b, a)[[0, 2]])   # sagittal (joint-axis-normal) length
        L[leg] = (xz(ks[0], ks[1]), xz(ks[1], ks[2]))
    spread = np.ptp([v for v in L.values()], axis=0)
    check(results, "all legs have identical femur/tibia lengths (sagittal, FK)", np.all(spread < 1e-6),
          f"femur {L['FL'][0]*1000:.2f} mm, tibia {L['FL'][1]*1000:.2f} mm")
    hips = {leg: np.array(p.getLinkState(rid, jm[f"{leg}_hip_roll"]["index"])[4]) for leg in LEGS}
    sym = abs(hips["FL"][1] + hips["FR"][1]) < 1e-6 and abs(hips["RL"][1] + hips["RR"][1]) < 1e-6
    check(results, "left/right mirror symmetry of hips", sym,
          f"FL {np.round(hips['FL']*1000,1).tolist()}  RR {np.round(hips['RR']*1000,1).tolist()}")
    p.removeBody(rid)
    return jm


def support_polygon_margin(pt, poly):
    """Signed distance of 2D point to convex polygon (positive = inside)."""
    from scipy.spatial import ConvexHull
    hull = ConvexHull(poly)
    d = []
    for eq in hull.equations:  # a*x + b*y + c <= 0 inside
        d.append(-(eq[0] * pt[0] + eq[1] * pt[1] + eq[2]))
    return min(d)


def base_frame_pos(rid):
    """PyBullet reports the base's inertial (COM) frame; convert to the URDF base_link origin."""
    pos, orn = p.getBasePositionAndOrientation(rid)
    loc = np.array(p.getDynamicsInfo(rid, -1)[3])
    R = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
    return np.array(pos) - R @ loc, orn


def gravity_stand(results, seconds=8.0):
    q1, q2 = stance_angles(P.STANCE_LEG_EXTENSION / 1000.0)
    h = stance_height(P.STANCE_LEG_EXTENSION / 1000.0)
    plane = p.loadURDF("plane.urdf")
    drop = 0.03
    rid = load_robot(base_pos=(0, 0, h + drop))
    jm = joint_map(rid)
    names = {j["index"]: j["link"] for j in jm.values()}
    names[-1] = "base_link"
    feet = {jm[f"{leg}_foot_fixed"]["index"]: leg for leg in LEGS}
    for fi in feet:
        p.changeDynamics(rid, fi, lateralFriction=P.FOOT_LATERAL_FRICTION, spinningFriction=P.FOOT_SPINNING_FRICTION,
                         rollingFriction=P.FOOT_ROLLING_FRICTION)
    set_pose(rid, jm, {"hip_roll": 0.0, "hip_pitch": q1, "knee": q2})
    dt = 1 / 240.0
    push_t, push_dur, push_f = 4.0, 0.05, np.array([6.0, 10.0, 0.0])   # 0.5 N*s shove, mostly sideways
    log, forces = [], {}
    weight = None
    for k in range(int(seconds / dt)):
        t = k * dt
        if push_t <= t < push_t + push_dur:
            pos, _ = p.getBasePositionAndOrientation(rid)
            p.applyExternalForce(rid, -1, push_f.tolist(), pos, p.WORLD_FRAME)
        p.stepSimulation()
        cps = p.getContactPoints(rid, plane)
        win = "settled" if 3.0 <= t < push_t else ("end" if t >= seconds - 1.0 else None)
        if win:
            for c in cps:
                forces.setdefault(win, {}).setdefault(names[c[3]], 0.0)
                forces[win][names[c[3]]] += c[9] * dt
        if k % 6 == 0:
            bpos, orn = base_frame_pos(rid)
            rpy = np.degrees(p.getEulerFromQuaternion(orn))
            com, mass = whole_body_com(rid)
            weight = mass * 9.81
            fpos = {feet[fi]: p.getLinkState(rid, fi)[4][:2] for fi in feet}
            log.append(dict(t=t, z=bpos[2], roll=rpy[0], pitch=rpy[1], com=com.tolist(), mass=mass, feet_xy=fpos))
    for w in forces:
        span = (push_t - 3.0) if w == "settled" else 1.0
        forces[w] = {k: v / span for k, v in forces[w].items()}

    settled = [r for r in log if 1.0 <= r["t"] < push_t]
    zs = np.array([r["z"] for r in settled])
    rp = np.abs(np.array([[r["roll"], r["pitch"]] for r in settled]))
    check(results, "stands under gravity at the IK height (1-4 s)",
          abs(np.median(zs) - h) < 0.005 and rp.max() < 3.0,
          f"base_link height {zs.min()*1000:.1f}-{zs.max()*1000:.1f} mm vs IK {h*1000:.1f} mm "
          f"(dropped from +{drop*1000:.0f} mm); max |roll| {rp[:,0].max():.2f} deg, |pitch| {rp[:,1].max():.2f} deg")
    fs = forces["settled"]
    foot_names = [f"{leg}_foot" for leg in LEGS]
    others = {k: v for k, v in fs.items() if k not in foot_names}
    share = {k: fs.get(k, 0.0) / weight for k in foot_names}
    check(results, "load carried by all 4 feet and nothing else (time-averaged contact force)",
          all(v > 0.10 for v in share.values()) and sum(others.values()) < 0.01 * weight,
          "weight share " + ", ".join(f"{k[:2]} {v*100:.0f}%" for k, v in share.items())
          + f"; sum {sum(fs.values()):.2f} N vs weight {weight:.2f} N; other links {sum(others.values()):.3f} N")
    margins = []
    for r in settled:
        poly = np.array([r["feet_xy"][leg] for leg in LEGS])
        margins.append(support_polygon_margin(np.array(r["com"][:2]), poly))
    r = settled[-1]
    check(results, "COM projection inside support polygon (every sample, 1-4 s)", min(margins) > 0,
          f"min margin {min(margins)*1000:.1f} mm; COM xy {np.round(np.array(r['com'][:2])*1000, 1).tolist()} mm; "
          f"feet at x {np.round([r['feet_xy'][l][0]*1000 for l in LEGS], 1).tolist()} mm; total mass {r['mass']:.3f} kg")
    after = [r for r in log if r["t"] >= seconds - 1.0]
    rp2 = np.abs(np.array([[r["roll"], r["pitch"]] for r in after]))
    peak = np.abs(np.array([[r["roll"], r["pitch"]] for r in log if push_t <= r["t"] < push_t + 1.0])).max()
    fe = forces.get("end", {})
    check(results, f"recovers from a {np.linalg.norm(push_f)*push_dur:.2f} N*s shove at t={push_t:.0f} s",
          rp2.max() < 3.0 and all(fe.get(k, 0) > 0.1 * weight for k in foot_names),
          f"peak tilt {peak:.2f} deg; final 1 s max |roll| {rp2[:,0].max():.2f}, |pitch| {rp2[:,1].max():.2f} deg; "
          f"all feet loaded again: {all(fe.get(k, 0) > 0.1 * weight for k in foot_names)}")
    json.dump(dict(stance=dict(hip_pitch=q1, knee=q2, base_height_m=h), foot_forces=forces, log=log),
              open(REP / "gravity_sim_log.json", "w"), indent=1)
    return rid, jm, plane, r


def feet_idx(jm):
    return [jm[f"{leg}_foot_fixed"]["index"] for leg in LEGS]


def name_of(jm, idx):
    if idx == -1:
        return "base_link"
    for n, j in jm.items():
        if j["index"] == idx:
            return j["link"]
    return str(idx)


def render(rid, target, dist, yaw, pitch, w=960, h=720, fov=35):
    V = p.computeViewMatrixFromYawPitchRoll(target, dist, yaw, pitch, 0, 2)
    Pm = p.computeProjectionMatrixFOV(fov, w / h, 0.01, 5)
    img = p.getCameraImage(w, h, V, Pm, renderer=p.ER_TINY_RENDERER, shadow=1, lightDirection=[1, -1, 2],
                           lightAmbientCoeff=0.55, lightDiffuseCoeff=0.55, lightSpecularCoeff=0.1)
    return np.reshape(img[2], (h, w, 4))[:, :, :3]


def renders(rid, jm):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    pos, _ = p.getBasePositionAndOrientation(rid)
    views = [("front-left iso", 130, -25), ("side (left)", 180, -5), ("front", 90, -5), ("top", 90, -89)]
    ims = [(n, render(rid, [pos[0], pos[1], pos[2] * 0.6], 0.75, yaw, pit)) for n, yaw, pit in views]
    fig, axs = plt.subplots(2, 2, figsize=(14, 10.5))
    for a, (n, im) in zip(axs.ravel(), ims):
        a.imshow(im)
        a.set_title(n)
        a.axis("off")
    fig.suptitle("SM3 URDF standing under gravity in PyBullet (after 8 s sim incl. a shove at 4 s)")
    plt.tight_layout()
    plt.savefig(REP / "render_standing.png", dpi=80)
    plt.close()



def photo_check(results, photo):
    """Fit a camera to the reference photo (silhouette IoU) and render the URDF from it."""
    import subprocess
    subprocess.check_call([sys.executable, str(ROOT / "tools" / "photo_match.py"), photo])
    pm = json.loads((REP / "photo_match.json").read_text())
    check(results, "visual sanity vs reference photo (silhouette IoU from fitted camera)", pm["iou"] > 0.6,
          f"IoU {pm['iou']:.3f} (robot+stand silhouettes; legs posed by hand, so ~0.7-0.8 is a good match); "
          f"see reports/render_vs_photo.png")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--photo", help="reference photo for the side-by-side image")
    ap.add_argument("--no-render", action="store_true")
    args = ap.parse_args()
    REP.mkdir(exist_ok=True)
    p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    results = []
    topology(results)
    rid, jm, plane, final = gravity_stand(results)
    if not args.no_render:
        renders(rid, jm)
        if args.photo:
            photo_check(results, args.photo)
    json.dump(results, open(REP / "validation_results.json", "w"), indent=2)
    n_fail = sum(not r["pass"] for r in results)
    print(f"\n{len(results) - n_fail}/{len(results)} checks passed")
    sys.exit(1 if n_fail else 0)


if __name__ == "__main__":
    main()
