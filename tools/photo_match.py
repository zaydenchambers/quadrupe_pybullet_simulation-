"""Fit a camera to the reference photo by maximising silhouette IoU (robot + stand),
then render the URDF from that camera. Used for STEP 6 visual sanity."""
import sys, json, itertools
from pathlib import Path
import numpy as np, pybullet as p, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT)); sys.path.insert(0, str(ROOT / "tools"))
from sm3_sim import load_robot, joint_map, set_pose
from sm3lib import load as load_stl

def photo_mask(img, W, H):
    from matplotlib.colors import rgb_to_hsv
    im = img[..., :3]
    hsv = rgb_to_hsv(im)
    m = (hsv[..., 1] > 0.28) | (hsv[..., 2] < 0.16)
    # downsample to W x H by block averaging
    ys = np.linspace(0, im.shape[0], H + 1).astype(int); xs = np.linspace(0, im.shape[1], W + 1).astype(int)
    out = np.zeros((H, W))
    for i in range(H):
        for j in range(W):
            out[i, j] = m[ys[i]:ys[i+1], xs[j]:xs[j+1]].mean()
    return out > 0.5

def scene(hip_pitch=1.05, knee=-1.5):
    p.resetSimulation()
    for stem, z0, yaw in [("Stand_ConeBase", 0.0, 0.0), ("Stand_Cone", 11.0, 0.0), ("Stand_ConeCap", 214.0, 90.0)]:
        m = load_stl(stem); c = m.bounds.mean(0)
        vs = p.createVisualShape(p.GEOM_MESH, fileName=str(ROOT / "source_stl" / f"SM3_{stem}.stl"), meshScale=[1e-3]*3,
                                 rgbaColor=[0.35, 0.5, 0.8, 1], visualFramePosition=[-c[0]*1e-3, -c[1]*1e-3, -m.bounds[0][2]*1e-3])
        p.createMultiBody(0, -1, vs, basePosition=[0, 0, z0*1e-3], baseOrientation=p.getQuaternionFromEuler([0, 0, np.radians(yaw)]))
    s = json.loads((ROOT / "reports" / "build_summary.json").read_text())
    belly = -s["body.bounding_box_mm"]["value"][0][2]
    global RID, Z0
    Z0 = (214.0 + 42.0 + belly) * 1e-3
    rid = RID = load_robot(base_pos=(0, 0, Z0), fixed=True)
    jm = joint_map(rid)
    set_pose(rid, jm, {"hip_roll": 0.0, "hip_pitch": hip_pitch, "knee": knee})
    return rid

RID, Z0 = None, 0.0


def cam(x, W, H, rgb=False):
    yaw, pitch, dist, tx, ty, tz, fov, dx = x
    p.resetBasePositionAndOrientation(RID, [dx, 0, Z0], [0, 0, 0, 1])   # robot slid along the cradle
    V = p.computeViewMatrixFromYawPitchRoll([tx, ty, tz], dist, yaw, pitch, 0, 2)
    P = p.computeProjectionMatrixFOV(fov, W / H, 0.01, 5)
    r = p.getCameraImage(W, H, V, P, renderer=p.ER_TINY_RENDERER, shadow=1 if rgb else 0, lightDirection=[1, -1, 2],
                         lightAmbientCoeff=0.55, lightDiffuseCoeff=0.55, lightSpecularCoeff=0.1)
    seg = np.reshape(r[4], (H, W)) >= 0
    return (np.reshape(r[2], (H, W, 4))[..., :3] if rgb else None), seg

def iou(a, b):
    return (a & b).sum() / max(1, (a | b).sum())

def fit(photo, W=148, H=100):
    pm = photo_mask(plt.imread(photo), W, H)
    best = (-1, None)
    for yaw, pitch, dist in itertools.product(range(100, 171, 10), range(-50, -19, 6), (0.6, 0.75, 0.9)):
        for tx, tz in itertools.product((-0.03, 0.03), (0.22, 0.28)):
            x = [yaw, pitch, dist, tx, 0.0, tz, 40, 0.0]
            s = iou(cam(x, W, H)[1], pm)
            if s > best[0]: best = (s, x)
    x = list(best[1]); step = [5, 3, 0.05, 0.02, 0.02, 0.02, 4, 0.03]
    for it in range(10):
        improved = False
        for k in range(8):
            for sgn in (-1, 1):
                y = list(x); y[k] += sgn * step[k]
                s = iou(cam(y, W, H)[1], pm)
                if s > best[0]: best = (s, y); x = y; improved = True
        if not improved: step = [v / 2 for v in step]
    return best, pm

if __name__ == "__main__":
    photo = sys.argv[1]
    p.connect(p.DIRECT)
    scene()
    (score, x), pm = fit(photo)
    print("best IoU %.3f camera yaw %.1f pitch %.1f dist %.3f target (%.3f, %.3f, %.3f) fov %.1f robot dx %.3f" % (score, *x))
    im, seg = cam(x, 1184, 798, rgb=True)
    img = plt.imread(photo)
    pmf = photo_mask(img, 296, 200); _, segf = cam(x, 296, 200)
    fig, ax = plt.subplots(1, 3, figsize=(20, 5.4))
    ax[0].imshow(img); ax[0].set_title("reference photo")
    ax[1].imshow(im); ax[1].set_title("URDF render from the fitted camera")
    ov = np.zeros((200, 296, 3)); ov[..., 0] = pmf; ov[..., 1] = segf; ov[..., 2] = 0
    ax[2].imshow(ov); ax[2].set_title(f"silhouettes: red = photo only, green = render only, yellow = both (IoU {score:.2f})")
    for a in ax: a.axis("off")
    plt.tight_layout(); plt.savefig(ROOT / "reports" / "render_vs_photo.png", dpi=70)
    json.dump(dict(iou=score, camera=dict(zip(["yaw", "pitch", "dist", "tx", "ty", "tz", "fov", "robot_dx_on_stand"], x))),
              open(ROOT / "reports" / "photo_match.json", "w"), indent=2)
