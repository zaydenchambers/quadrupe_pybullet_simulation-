"""Watch a trained policy walk in the PyBullet GUI.

    python evaluate.py                                    # newest run's final model
    python evaluate.py runs/ppo_<stamp>/checkpoints/model_2000000_steps.zip
    python evaluate.py --episodes 3 --direct              # headless; just print the scores
"""
import argparse
import time
from pathlib import Path

import numpy as np
import pybullet
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

import params as P
from quadruped_env import QuadrupedEnv
from train import RUNS, vecnormalize_for


def latest_model() -> Path:
    finals = sorted(RUNS.glob("ppo_*/final_model.zip"))
    if not finals:
        raise SystemExit("no trained model in runs/ - run train.py first or pass a .zip path")
    return finals[-1]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("model", type=Path, nargs="?", help="model .zip (default: newest runs/*/final_model.zip)")
    ap.add_argument("--episodes", type=int, default=5)
    ap.add_argument("--direct", action="store_true", help="no GUI, run as fast as possible")
    ap.add_argument("--stochastic", action="store_true", help="sample actions instead of using the mean")
    args = ap.parse_args()

    model_path = args.model or latest_model()
    print(f"model: {model_path}")
    env = QuadrupedEnv(render_mode=None if args.direct else "human")
    # Observation normalisation statistics from training; frozen for evaluation.
    norm = VecNormalize.load(str(vecnormalize_for(model_path)), DummyVecEnv([lambda: env]))
    norm.training = False
    norm.norm_reward = False
    model = PPO.load(model_path, device="cpu")

    control_dt = P.SIM_TIMESTEP * P.ACTION_REPEAT
    for ep in range(args.episodes):
        obs, _ = env.reset(seed=1000 + ep)
        ret, n = 0.0, 0
        while True:
            t0 = time.perf_counter()
            action, _ = model.predict(norm.normalize_obs(obs), deterministic=not args.stochastic)
            obs, r, term, trunc, info = env.step(action)
            ret, n = ret + r, n + 1
            if term or trunc:
                break
            if not args.direct:
                time.sleep(max(0.0, control_dt - (time.perf_counter() - t0)))  # real time
        secs = n * control_dt
        print(f"episode {ep}: {secs:5.1f} s  {'FELL' if term else 'time limit'}  x={info['x']:+.2f} m  "
              f"avg {info['x'] / secs:+.2f} m/s  return={ret:.1f}")
    env.close()


if __name__ == "__main__":
    try:
        main()
    except pybullet.error as e:
        if "Not connected" not in str(e):
            raise
        print("PyBullet window closed - stopping.")
