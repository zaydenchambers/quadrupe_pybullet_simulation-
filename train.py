"""Train a PPO walking policy on QuadrupedEnv.

    python train.py                         # full run (TRAIN_TOTAL_STEPS from params.py)
    python train.py --steps 5000 --n-envs 2 # quick check that training starts
    python train.py --resume runs/ppo_<stamp>/checkpoints/model_1000000_steps.zip
    tensorboard --logdir runs               # watch the curves

Each run writes to runs/ppo_<stamp>/:
    checkpoints/model_<N>_steps.zip + model_vecnormalize_<N>_steps.pkl   every TRAIN_CHECKPOINT_EVERY steps
    final_model.zip + final_vecnormalize.pkl                         at the end (or on Ctrl+C)
    tb/                                                              tensorboard logs
"""
import argparse
import signal
import time
from pathlib import Path

from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import SubprocVecEnv, VecNormalize

import params as P
from quadruped_env import QuadrupedEnv

RUNS = Path(__file__).resolve().parent / "runs"


def vecnormalize_for(model_path: Path) -> Path:
    """checkpoints/model_N_steps.zip -> checkpoints/model_vecnormalize_N_steps.pkl (CheckpointCallback's naming),
    final_model.zip -> final_vecnormalize.pkl."""
    stem = model_path.stem
    if stem == "final_model":
        return model_path.with_name("final_vecnormalize.pkl")
    return model_path.with_name(stem.replace("model_", "model_vecnormalize_", 1) + ".pkl")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--steps", type=int, default=P.TRAIN_TOTAL_STEPS)
    ap.add_argument("--n-envs", type=int, default=P.TRAIN_N_ENVS)
    ap.add_argument("--resume", type=Path, help="checkpoint .zip to continue from")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    run_dir = RUNS / f"ppo_{time.strftime('%Y%m%d_%H%M%S')}"
    ckpt_dir = run_dir / "checkpoints"
    ckpt_dir.mkdir(parents=True)

    venv = make_vec_env(QuadrupedEnv, n_envs=args.n_envs, seed=args.seed,
                        vec_env_cls=SubprocVecEnv if args.n_envs > 1 else None)
    if args.resume:
        venv = VecNormalize.load(str(vecnormalize_for(args.resume)), venv)
        model = PPO.load(args.resume, env=venv, tensorboard_log=str(run_dir / "tb"))
    else:
        venv = VecNormalize(venv, norm_obs=True, norm_reward=True, clip_obs=10.0, gamma=P.PPO_KWARGS["gamma"])
        model = PPO("MlpPolicy", venv, seed=args.seed, verbose=1, tensorboard_log=str(run_dir / "tb"),
                    **P.PPO_KWARGS)

    checkpoints = CheckpointCallback(save_freq=max(P.TRAIN_CHECKPOINT_EVERY // args.n_envs, 1),
                                     save_path=str(ckpt_dir), name_prefix="model", save_vecnormalize=True)
    print(f"run dir: {run_dir}")
    # `kill <pid>` (SIGTERM) stops training like Ctrl+C, so the final model is still saved.
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt))
    try:
        model.learn(total_timesteps=args.steps, callback=checkpoints, reset_num_timesteps=not args.resume)
    except KeyboardInterrupt:
        print("interrupted - saving what we have")
    finally:
        model.save(run_dir / "final_model")
        venv.save(str(run_dir / "final_vecnormalize.pkl"))
        venv.close()
    print(f"saved {run_dir / 'final_model.zip'}")


if __name__ == "__main__":
    main()
