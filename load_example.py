"""Minimal PyBullet example: drop the SM3 SpotMicro on the floor, stand, then do slow squats.

    python load_example.py            # opens the PyBullet GUI if a display is available
    python load_example.py --direct   # headless; prints base height / foot loads

Joint naming: {FL,FR,RL,RR}_{hip_roll,hip_pitch,knee}. Axes: roll about +X, pitch and knee
about +Y for all four legs. hip_pitch > 0 swings the knee backwards; knee < 0 folds the
tibia forward (Spot-style, knees point back).
"""
import argparse
import math
import time

import pybullet as p
import pybullet_data

import params as P
from sm3_sim import LEGS, joint_map, load_robot, set_pose, stance_angles, stance_height


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--direct", action="store_true", help="run headless")
    ap.add_argument("--seconds", type=float, default=20.0)
    args = ap.parse_args()

    p.connect(p.DIRECT if args.direct else p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.loadURDF("plane.urdf")

    ext = P.STANCE_LEG_EXTENSION / 1000.0
    robot = load_robot(base_pos=(0, 0, stance_height(ext) + 0.02))
    jm = joint_map(robot)
    for leg in LEGS:
        p.changeDynamics(robot, jm[f"{leg}_foot_fixed"]["index"], lateralFriction=P.FOOT_LATERAL_FRICTION)
    print("revolute joints:", [n for n, j in jm.items() if j["type"] == p.JOINT_REVOLUTE])

    q1, q2 = stance_angles(ext)
    set_pose(robot, jm, {"hip_roll": 0.0, "hip_pitch": q1, "knee": q2})
    if not args.direct:
        p.resetDebugVisualizerCamera(0.7, 40, -25, [0, 0, 0.15])

    dt = 1 / 240.0
    t = 0.0
    while t < args.seconds:
        if t > 2.0:  # gentle squat: leg extension 175 mm +/- 25 mm
            e = ext + 0.025 * math.sin(2 * math.pi * 0.25 * (t - 2.0))
            a, b = stance_angles(e)
            set_pose(robot, jm, {"hip_roll": 0.0, "hip_pitch": a, "knee": b}, reset=False)
        p.stepSimulation()
        t += dt
        if args.direct and abs(t % 1.0) < dt:
            pos, _ = p.getBasePositionAndOrientation(robot)
            print(f"t={t:4.1f}s  base COM z={pos[2]*1000:6.1f} mm")
        if not args.direct:
            time.sleep(dt)


if __name__ == "__main__":
    main()
