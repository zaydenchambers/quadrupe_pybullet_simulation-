"""Small helper module shared by the loader example and the validation script."""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
import pybullet as p

ROOT = Path(__file__).resolve().parent
URDF = ROOT / "spotmicro_sm3.urdf"
LEGS = ("FL", "FR", "RL", "RR")


def summary():
    return json.loads((ROOT / "reports" / "build_summary.json").read_text())


def leg_lengths():
    s = summary()
    return (s["femur.length_hip_to_knee"]["value"] / 1000.0,
            s["tibia.length_knee_to_foot_centre"]["value"] / 1000.0,
            s["foot.tyre_outer_radius"]["value"] / 1000.0,
            s["coax.roll_to_pitch_offset_z"]["value"] / 1000.0)


def stance_angles(extension_m: float):
    """Planar IK: tyre centre directly below the hip-pitch axis at distance `extension_m`.
    Returns (hip_pitch, knee) in rad; knee < 0 means the knee points backwards (Spot-style)."""
    L1, L2, _, _ = leg_lengths()
    c2 = (extension_m**2 - L1**2 - L2**2) / (2 * L1 * L2)
    if abs(c2) > 1:
        raise ValueError("extension out of reach")
    q2 = -math.acos(c2)
    q1 = math.atan2(-L2 * math.sin(q2), L1 + L2 * math.cos(q2))
    return q1, q2


def stance_height(extension_m: float):
    """Height of base_link origin (hip-roll axis) above the ground in that stance."""
    _, _, r, dz = leg_lengths()
    return dz + extension_m + r


def joint_map(robot):
    out = {}
    for j in range(p.getNumJoints(robot)):
        info = p.getJointInfo(robot, j)
        out[info[1].decode()] = dict(index=j, type=info[2], link=info[12].decode(), parent=info[16],
                                     lower=info[8], upper=info[9], effort=info[10], axis=info[13])
    return out


def load_robot(base_pos=(0, 0, 0.3), fixed=False, flags=0):
    return p.loadURDF(str(URDF), basePosition=base_pos, useFixedBase=fixed,
                      flags=flags | p.URDF_USE_INERTIA_FROM_FILE)


def set_pose(robot, jm, q_by_type: dict, reset=True, force=None):
    """q_by_type: {"hip_roll": a, "hip_pitch": b, "knee": c} applied to every leg."""
    for leg in LEGS:
        for jt, q in q_by_type.items():
            j = jm[f"{leg}_{jt}"]["index"]
            if reset:
                p.resetJointState(robot, j, q)
            p.setJointMotorControl2(robot, j, p.POSITION_CONTROL, targetPosition=q,
                                    force=force if force is not None else jm[f"{leg}_{jt}"]["effort"],
                                    positionGain=0.3, velocityGain=1.0)


def whole_body_com(robot):
    ms, ps = [], []
    m = p.getDynamicsInfo(robot, -1)[0]
    pos, _ = p.getBasePositionAndOrientation(robot)   # already the base's COM (inertial frame)
    ms.append(m)
    ps.append(np.array(pos))
    for j in range(p.getNumJoints(robot)):
        ms.append(p.getDynamicsInfo(robot, j)[0])
        ps.append(np.array(p.getLinkState(robot, j, computeForwardKinematics=True)[0]))  # link COM (inertial frame)
    ms = np.array(ms)
    return (ms[:, None] * np.array(ps)).sum(0) / ms.sum(), ms.sum()
