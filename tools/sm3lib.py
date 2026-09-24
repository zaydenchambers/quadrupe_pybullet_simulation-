"""Geometry helpers shared by measure_parts.py and build_urdf.py.

All lengths are millimetres (the STL units) unless a name ends in _m.
Paths are resolved relative to the project root (the folder that holds this
`tools/` directory), so the project can be moved anywhere.
"""
from __future__ import annotations

import json
import os
from pathlib import Path

import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parent.parent
SRC = ROOT / "source_stl"
MESH_OUT = ROOT / "meshes"
REPORTS = ROOT / "reports"


def load(name: str) -> trimesh.Trimesh:
    """Load an STL from source_stl/ by file stem (with or without the SM3_ prefix)."""
    stem = name if name.startswith("SM3_") else "SM3_" + name
    m = trimesh.load(SRC / f"{stem}.stl", force="mesh")
    return m


# --------------------------------------------------------------------------- #
# rotations / transforms
# --------------------------------------------------------------------------- #
_AX = {"X": np.array([1.0, 0, 0]), "Y": np.array([0, 1.0, 0]), "Z": np.array([0, 0, 1.0])}


def axis_map(spec: str) -> np.ndarray:
    """Rotation from a string like 'X>+Y Y>+X Z>-Z' (source axis > target axis).

    Returns R such that target = R @ source. Raises if the mapping is not a proper
    rotation (det must be +1; mirror images are handled separately)."""
    R = np.zeros((3, 3))
    for tok in spec.split():
        s, t = tok.split(">")
        sign = -1.0 if t[0] == "-" else 1.0
        R[:, "XYZ".index(s)] = sign * _AX[t[-1]]
    if not np.isclose(np.linalg.det(R), 1.0):
        raise ValueError(f"axis map '{spec}' is not a proper rotation (det={np.linalg.det(R)})")
    return R


def rot_y(deg: float) -> np.ndarray:
    a = np.radians(deg)
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def make_T(R: np.ndarray, p_src, p_dst=(0, 0, 0)) -> np.ndarray:
    """4x4 transform with rotation R that sends point p_src to p_dst."""
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(p_dst, float) - R @ np.asarray(p_src, float)
    return T


def orthonormal_from_rows(X, Z) -> np.ndarray:
    """Build a rotation (rows = target axes expressed in source coords) from X and Z guesses."""
    Z = np.asarray(Z, float) / np.linalg.norm(Z)
    X = np.asarray(X, float)
    X = X - Z * (X @ Z)
    X /= np.linalg.norm(X)
    Y = np.cross(Z, X)
    return np.vstack([X, Y, Z])


MIRROR_Y = np.diag([1.0, -1.0, 1.0, 1.0])


# --------------------------------------------------------------------------- #
# feature finding
# --------------------------------------------------------------------------- #
def ring_stats(xy: np.ndarray):
    A = abs(0.5 * np.sum(xy[:-1, 0] * xy[1:, 1] - xy[1:, 0] * xy[:-1, 1]))
    L = np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1))
    return A, L, (4 * np.pi * A / L**2 if L > 0 else 0.0), np.sqrt(A / np.pi)


def circular_holes(mesh, normal, origin, rmin, rmax, offsets=(0.0,), min_circ=0.9):
    """Circular interior rings in planar sections of `mesh`.

    A bore is only circular when the cutting plane is perpendicular to its true
    axis, so a high circularity score is itself evidence the axis is right.
    Returns list of dicts: circ, center(3D), r, offset."""
    n = np.asarray(normal, float)
    n /= np.linalg.norm(n)
    o = np.asarray(origin, float)
    out = []
    for off in offsets:
        s = mesh.section(plane_origin=o + n * off, plane_normal=n)
        if s is None:
            continue
        p2, T = s.to_2D()
        try:
            polys = p2.polygons_full
        except Exception:
            continue
        for poly in polys:
            for ring in poly.interiors:
                xy = np.array(ring.coords)
                A, L, c, r = ring_stats(xy)
                if rmin < r < rmax and c >= min_circ:
                    cen = (T @ np.r_[xy[:-1].mean(0), 0, 1])[:3]
                    out.append(dict(circ=float(c), center=cen, r=float(r), offset=float(off)))
    return out


def axis_bores(mesh, axis: str, step=1.0, rmin=0.8, rmax=40, min_circ=0.9):
    """Scan sections perpendicular to a print axis; cluster rings into bores."""
    d = _AX[axis]
    proj = mesh.vertices @ d
    hits = []
    for h in np.arange(proj.min() + 0.3, proj.max() - 0.3, step):
        for hh in circular_holes(mesh, d, d * h, rmin, rmax, (0.0,), min_circ):
            hits.append((h, hh))
    clusters = []
    for h, hh in hits:
        q = hh["center"] - d * (hh["center"] @ d)
        for C in clusters:
            if np.linalg.norm(C["q"] - q) < 1.0 and abs(C["r"] - hh["r"]) < 0.6:
                C["span"][0] = min(C["span"][0], h)
                C["span"][1] = max(C["span"][1], h)
                break
        else:
            clusters.append(dict(q=q, r=hh["r"], span=[h, h], axis=axis))
    return clusters


def find_bores(clusters, near=None, r=None, tol=1.0, rtol=0.3):
    out = []
    for c in clusters:
        if r is not None and abs(c["r"] - r) > rtol:
            continue
        if near is not None and np.linalg.norm(c["q"] - np.asarray(near, float)) > tol:
            continue
        out.append(c)
    return out


def section_center(mesh, origin, normal):
    """Bounding-box centre and size of a planar section (used for pegs/pins)."""
    s = mesh.section(plane_origin=origin, plane_normal=normal)
    v = s.vertices
    return (v.min(0) + v.max(0)) / 2, v.max(0) - v.min(0)


def fit_circle_2d(p):
    from scipy.optimize import least_squares

    f = lambda c: np.linalg.norm(p - c[:2], axis=1) - c[2]
    r = least_squares(f, [p[:, 0].mean(), p[:, 1].mean(), 10.0])
    return r.x


def plate_normal(mesh, face_mask):
    """Normal of a thin tab = smallest principal axis of its vertices."""
    sub = mesh.submesh([np.where(face_mask)[0]], append=True)
    v = sub.vertices - sub.vertices.mean(0)
    w, V = np.linalg.eigh(np.cov(v.T))
    return V[:, 0], sub.vertices.mean(0), np.sqrt(w)


def pca(v):
    c = v.mean(0)
    w, V = np.linalg.eigh(np.cov((v - c).T))
    V = V[:, ::-1]
    if np.linalg.det(V) < 0:
        V[:, 2] *= -1
    return c, V, np.sqrt(w[::-1])


def register_mirror(src: trimesh.Trimesh, ref: trimesh.Trimesh, n=6000):
    """4x4 A (improper: includes a mirror) mapping src print coords onto ref print coords.

    Right-side parts are exported as mirror images of left-side parts; A lets the
    right leg reuse every feature measured on the left part."""
    from trimesh.registration import icp

    M = MIRROR_Y
    ps = trimesh.transform_points(trimesh.sample.sample_surface(src, n, seed=0)[0], M)
    pr = trimesh.sample.sample_surface(ref, n, seed=0)[0]
    cs, Vs, _ = pca(ps)
    cr, Vr, _ = pca(pr)
    best = None
    for sx in (1, -1):
        for sy in (1, -1):
            D = np.diag([sx, sy, sx * sy])
            R0 = Vr @ D @ Vs.T
            T0 = make_T(R0, cs, cr)
            T, _, cost = icp(ps, ref, initial=T0, threshold=1e-7, max_iterations=60)
            if best is None or cost < best[1]:
                best = (T, cost)
    A = best[0] @ M
    q = trimesh.transform_points(trimesh.sample.sample_surface(src, 3000, seed=1)[0], A)
    d = np.abs(trimesh.proximity.signed_distance(ref, q))
    return A, float(d.mean()), float(np.percentile(d, 95))


def register_rigid(src: trimesh.Trimesh, ref: trimesh.Trimesh, n=6000):
    """Same as register_mirror but without the mirror (for re-oriented print copies)."""
    from trimesh.registration import icp

    ps = trimesh.sample.sample_surface(src, n, seed=0)[0]
    cs, Vs, _ = pca(ps)
    cr, Vr, _ = pca(trimesh.sample.sample_surface(ref, n, seed=0)[0])
    best = None
    for sx in (1, -1):
        for sy in (1, -1):
            D = np.diag([sx, sy, sx * sy])
            T, _, cost = icp(ps, ref, initial=make_T(Vr @ D @ Vs.T, cs, cr), threshold=1e-7, max_iterations=60)
            if best is None or cost < best[1]:
                best = (T, cost)
    q = trimesh.transform_points(trimesh.sample.sample_surface(src, 3000, seed=1)[0], best[0])
    d = np.abs(trimesh.proximity.signed_distance(ref, q))
    return best[0], float(d.mean()), float(np.percentile(d, 95))


# --------------------------------------------------------------------------- #
# inertia
# --------------------------------------------------------------------------- #
def box_inertia(m, dims):
    a, b, c = dims
    return np.diag([m * (b * b + c * c), m * (a * a + c * c), m * (a * a + b * b)]) / 12.0


def combine_boxes(components):
    """components: list of (mass_kg, com_m(3), dims_m(3)). Returns mass, com, I about com."""
    M = sum(c[0] for c in components)
    com = sum(c[0] * np.asarray(c[1]) for c in components) / M
    I = np.zeros((3, 3))
    for m, p, dims in components:
        d = np.asarray(p) - com
        I += box_inertia(m, dims) + m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    return M, com, I


class NumpyEncoder(json.JSONEncoder):
    def default(self, o):
        if isinstance(o, np.ndarray):
            return o.tolist()
        if isinstance(o, (np.floating,)):
            return float(o)
        if isinstance(o, (np.integer,)):
            return int(o)
        return super().default(o)


def dump_json(obj, path):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        json.dump(obj, f, indent=2, cls=NumpyEncoder)


def rel(p) -> str:
    return os.path.relpath(p, ROOT)
