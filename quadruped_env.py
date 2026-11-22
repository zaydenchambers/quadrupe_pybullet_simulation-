"""Gymnasium environment: teach the SM3 SpotMicro to walk forward (+X) in PyBullet.

    python quadruped_env.py              # smoke test: random actions, headless
    python quadruped_env.py --gui        # same, in the PyBullet GUI

Action (12): joint position offsets from the standing pose, in [-1, 1] * ACTION_SCALE rad,
    ordered FL, FR, RL, RR x (hip_roll, hip_pitch, knee). Low-pass filtered (ACTION_FILTER_ALPHA)
    before being sent to the servos.
Observation (50):
    3  gravity direction in the body frame (body orientation without yaw)
    3  base angular velocity in the body frame, rad/s
    12 joint angles relative to the stance, rad
    12 joint velocities, rad/s (x 0.1)
    12 previous filtered action (what the servos are currently tracking)
    3  base linear velocity in the body frame, m/s
    2  heading error as (sin, cos)
    1  sideways offset from the start line, m (clipped to +/-1)
    2  trot clock as (sin, cos)
Reward, termination and all tuning knobs live in params.py (RL section).
"""
from __future__ import annotations

import argparse

import gymnasium as gym
import numpy as np
import pybullet as p
import pybullet_data
from gymnasium import spaces
from pybullet_utils import bullet_client

import params as P
from sm3_sim import LEGS, URDF, stance_angles, stance_height

JOINT_TYPES = ("hip_roll", "hip_pitch", "knee")


class QuadrupedEnv(gym.Env):
    metadata = {"render_modes": ["human"], "render_fps": int(round(1 / (P.SIM_TIMESTEP * P.ACTION_REPEAT)))}

    def __init__(self, render_mode: str | None = None):
        self.render_mode = render_mode
        # One BulletClient per env so several envs can run side by side (SubprocVecEnv).
        self._p = bullet_client.BulletClient(p.GUI if render_mode == "human" else p.DIRECT)
        pc = self._p
        pc.setAdditionalSearchPath(pybullet_data.getDataPath())
        pc.setGravity(0, 0, -9.81)
        pc.setTimeStep(P.SIM_TIMESTEP)
        if render_mode == "human":
            pc.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        self._plane = pc.loadURDF("plane.urdf")

        ext = P.STANCE_LEG_EXTENSION / 1000.0
        self._stand_h = stance_height(ext)
        self._robot = pc.loadURDF(str(URDF), basePosition=(0, 0, self._stand_h),
                                  flags=p.URDF_USE_INERTIA_FROM_FILE)

        names = {pc.getJointInfo(self._robot, j)[1].decode(): j for j in range(pc.getNumJoints(self._robot))}
        self._joints = [names[f"{leg}_{jt}"] for leg in LEGS for jt in JOINT_TYPES]
        info = [pc.getJointInfo(self._robot, j) for j in self._joints]
        self._lower = np.array([i[8] for i in info])
        self._upper = np.array([i[9] for i in info])
        self._effort = np.array([i[10] for i in info])
        self._feet = [names[f"{leg}_foot_fixed"] for leg in LEGS]
        for foot in self._feet:
            pc.changeDynamics(self._robot, foot, lateralFriction=P.FOOT_LATERAL_FRICTION,
                              spinningFriction=P.FOOT_SPINNING_FRICTION, rollingFriction=P.FOOT_ROLLING_FRICTION,
                              mass=P.SIM_FOOT_MASS, contactStiffness=P.SIM_FOOT_CONTACT_STIFFNESS,
                              contactDamping=P.SIM_FOOT_CONTACT_DAMPING)
        self._phase0 = 0.0

        q1, q2 = stance_angles(ext)
        self._stance = np.tile([0.0, q1, q2], len(LEGS))

        self.action_space = spaces.Box(-1.0, 1.0, shape=(12,), dtype=np.float32)
        self.observation_space = spaces.Box(-np.inf, np.inf, shape=(50,), dtype=np.float32)
        self._max_steps = int(P.EPISODE_SECONDS / (P.SIM_TIMESTEP * P.ACTION_REPEAT))
        self._last_action = np.zeros(12)
        self._filtered_action = np.zeros(12)
        self._steps = 0

    # ------------------------------------------------------------------ helpers
    def _apply_targets(self, q_target):
        self._p.setJointMotorControlArray(
            self._robot, self._joints, p.POSITION_CONTROL, targetPositions=q_target.tolist(),
            forces=self._effort.tolist(), positionGains=[P.MOTOR_POSITION_GAIN] * 12,
            velocityGains=[P.MOTOR_VELOCITY_GAIN] * 12)

    def _joint_states(self):
        s = self._p.getJointStates(self._robot, self._joints)
        return (np.array([x[0] for x in s]), np.array([x[1] for x in s]), np.array([x[3] for x in s]))

    def _foot_contacts(self):
        # getContactPoints can return None instead of an empty result when running with the GUI.
        return np.array([bool(self._p.getContactPoints(self._robot, self._plane, f)) for f in self._feet])

    def _body_contacts(self):
        """Number of non-foot links touching the ground."""
        links = {c[3] for c in self._p.getContactPoints(self._robot, self._plane) or ()}
        return len(links - set(self._feet))

    def _base_state(self):
        pc = self._p
        pos, orn = pc.getBasePositionAndOrientation(self._robot)
        lin, ang = pc.getBaseVelocity(self._robot)
        rot = np.array(pc.getMatrixFromQuaternion(orn)).reshape(3, 3)  # body -> world
        roll, pitch, _ = pc.getEulerFromQuaternion(orn)
        return np.array(pos), rot, np.array(lin), np.array(ang), roll, pitch

    def _gait_phase(self, t):
        return 2 * np.pi * P.GAIT_FREQUENCY_HZ * t + self._phase0

    def _desired_contacts(self, t):
        """Trot: FL+RR are in stance for half the cycle, FR+RL for the other half (order follows LEGS)."""
        first_pair = np.sin(self._gait_phase(t)) >= 0
        return np.array([first_pair, not first_pair, not first_pair, first_pair])

    def _obs(self):
        pos, rot, lin, ang, _, _ = self._base_state()
        q, qd, _ = self._joint_states()
        gravity_body = rot.T @ np.array([0.0, 0.0, -1.0])
        yaw = np.arctan2(rot[1, 0], rot[0, 0])
        phase = self._gait_phase(self._steps * P.SIM_TIMESTEP * P.ACTION_REPEAT)
        return np.concatenate([gravity_body, rot.T @ ang, q - self._stance, 0.1 * qd, self._filtered_action,
                               rot.T @ lin, [np.sin(yaw), np.cos(yaw)], [np.clip(pos[1], -1.0, 1.0)],
                               [np.sin(phase), np.cos(phase)]]).astype(np.float32)

    # --------------------------------------------------------------- gym API
    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        pc, rng = self._p, self.np_random
        rpy = rng.uniform(-P.RESET_ORIENT_NOISE, P.RESET_ORIENT_NOISE, 3)
        z = self._stand_h + 0.005 + rng.uniform(-P.RESET_HEIGHT_NOISE, P.RESET_HEIGHT_NOISE)
        pc.resetBasePositionAndOrientation(self._robot, (0, 0, max(z, self._stand_h - 0.003)),
                                           pc.getQuaternionFromEuler(rpy.tolist()))
        pc.resetBaseVelocity(self._robot, (0, 0, 0), (0, 0, 0))
        q0 = np.clip(self._stance + rng.uniform(-P.RESET_JOINT_NOISE, P.RESET_JOINT_NOISE, 12),
                     self._lower, self._upper)
        for j, q in zip(self._joints, q0):
            pc.resetJointState(self._robot, j, q, 0.0)
        # Let it settle onto its feet holding the stance before the policy takes over.
        self._apply_targets(self._stance)
        for _ in range(int(0.25 / P.SIM_TIMESTEP)):
            pc.stepSimulation()
        self._last_action = np.zeros(12)
        self._filtered_action = np.zeros(12)
        self._phase0 = float(rng.uniform(0.0, 2 * np.pi))
        self._steps = 0
        return self._obs(), {}

    def step(self, action):
        action = np.clip(np.asarray(action, dtype=np.float64), -1.0, 1.0)
        a = P.ACTION_FILTER_ALPHA
        self._filtered_action = a * action + (1 - a) * self._filtered_action
        target = np.clip(self._stance + P.ACTION_SCALE * self._filtered_action, self._lower, self._upper)
        self._apply_targets(target)

        energy = 0.0
        joint_vel_sq = 0.0
        body_contacts = 0.0
        gait_match = 0.0
        flight = 0.0
        slip = 0.0
        t0 = self._steps * P.SIM_TIMESTEP * P.ACTION_REPEAT
        for i in range(P.ACTION_REPEAT):
            self._p.stepSimulation()
            _, qd, tau = self._joint_states()
            energy += np.abs(tau * qd).sum() * P.SIM_TIMESTEP
            joint_vel_sq += np.square(qd).sum() / P.ACTION_REPEAT
            body_contacts += self._body_contacts() / P.ACTION_REPEAT
            raw_contact = self._foot_contacts()
            desired = self._desired_contacts(t0 + (i + 1) * P.SIM_TIMESTEP)
            gait_match += float(np.mean(raw_contact == desired)) / P.ACTION_REPEAT
            flight += float(not raw_contact.any()) / P.ACTION_REPEAT
            for foot in np.array(self._feet)[raw_contact]:
                v = self._p.getLinkState(self._robot, foot, computeLinkVelocity=1)[6]
                slip += (v[0]**2 + v[1]**2) / P.ACTION_REPEAT
        if self.render_mode == "human":
            self._follow_camera()

        pos, rot, lin, _, roll, pitch = self._base_state()
        body_vel = rot.T @ lin
        yaw = np.arctan2(rot[1, 0], rot[0, 0])
        speed_error = (body_vel[0] - P.TARGET_FORWARD_SPEED) / P.FORWARD_SPEED_SIGMA
        action_rate = float(np.square(action - self._last_action).sum())
        terms = {
            "forward": P.REWARD_FORWARD_VEL * np.exp(-speed_error**2),
            "alive": P.REWARD_ALIVE,
            "tilt": -P.PENALTY_TILT * (roll**2 + pitch**2),
            "energy": -P.PENALTY_ENERGY * energy,
            "action_rate": -P.PENALTY_ACTION_RATE * action_rate,
            "drift": -P.PENALTY_LATERAL_DRIFT * abs(body_vel[1]),
            "lateral_offset": -P.PENALTY_LATERAL_OFFSET * pos[1]**2,
            "yaw": -P.PENALTY_YAW * yaw**2,
            "gait": P.REWARD_GAIT_MATCH * gait_match,
            "flight": -P.PENALTY_FLIGHT * flight,
            "vertical": -P.PENALTY_VERTICAL_VEL * lin[2]**2,
            "slip": -P.PENALTY_FOOT_SLIP * slip,
            "joint_vel": -P.PENALTY_JOINT_VEL * joint_vel_sq,
            "body_contact": -P.PENALTY_BODY_CONTACT * body_contacts,
        }
        reward = float(sum(terms.values()))

        self._last_action = action
        self._steps += 1
        fell = pos[2] < P.MIN_BASE_HEIGHT_FRAC * self._stand_h or max(abs(roll), abs(pitch)) > P.MAX_TILT
        terminated = bool(fell)
        truncated = self._steps >= self._max_steps
        info = {"x": pos[0], "height": pos[2], "vx": lin[0], "energy": energy, **{f"r_{k}": v for k, v in terms.items()}}
        return self._obs(), reward, terminated, truncated, info

    def _follow_camera(self):
        pos, _ = self._p.getBasePositionAndOrientation(self._robot)
        self._p.resetDebugVisualizerCamera(0.8, 40, -25, [pos[0], pos[1], 0.15])

    def render(self):
        pass  # the GUI draws itself while stepping in "human" mode

    def close(self):
        if self._p is not None:
            self._p.disconnect()
            self._p = None


def _smoke_test(episodes: int, gui: bool):
    """Random actions: checks shapes, resets, termination and the time limit."""
    from gymnasium.utils.env_checker import check_env

    env = QuadrupedEnv(render_mode="human" if gui else None)
    check_env(env, skip_render_check=True)
    obs, _ = env.reset(seed=0)
    print(f"standing: base z={env._base_state()[0][2]*1000:.1f} mm  (fall below "
          f"{P.MIN_BASE_HEIGHT_FRAC*env._stand_h*1000:.1f} mm), obs shape {obs.shape}")

    # Zero action should stand still for the whole episode (truncated, not terminated).
    obs, _ = env.reset(seed=1)
    ret, n = 0.0, 0
    while True:
        obs, r, term, trunc, info = env.step(np.zeros(12, dtype=np.float32))
        ret, n = ret + r, n + 1
        assert np.all(np.isfinite(obs)), "non-finite observation"
        if term or trunc:
            break
    print(f"zero action: {n} steps, terminated={term} truncated={trunc} return={ret:.2f} "
          f"height={info['height']*1000:.1f} mm")

    for ep in range(episodes):
        obs, _ = env.reset(seed=100 + ep)
        ret, n = 0.0, 0
        while True:
            obs, r, term, trunc, info = env.step(env.action_space.sample())
            ret, n = ret + r, n + 1
            assert np.all(np.isfinite(obs)), "non-finite observation"
            if term or trunc:
                break
        print(f"random ep {ep}: {n:4d} steps  {'FELL' if term else 'time limit'}  return={ret:7.2f}  "
              f"x={info['x']:+.3f} m  height={info['height']*1000:.1f} mm")
    env.close()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--gui", action="store_true")
    ap.add_argument("--episodes", type=int, default=5)
    a = ap.parse_args()
    _smoke_test(a.episodes, a.gui)
