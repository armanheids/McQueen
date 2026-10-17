"""Train the end-to-end PPO planner (nominal scenario categories only)."""
import sys, time
sys.path.insert(0, __file__.rsplit("/scripts/", 1)[0])
import numpy as np, torch
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv, DummyVecEnv, VecMonitor
from stable_baselines3.common.callbacks import CheckpointCallback
from mqg.config import NOMINAL
from mqg.baselines import CrossingEnv

steps = int(sys.argv[1]) if len(sys.argv) > 1 else 1_500_000
cats = [c.name for c in NOMINAL]
torch.set_num_threads(1)
if __name__ == "__main__":
    env = VecMonitor(DummyVecEnv([lambda i=i: CrossingEnv(cats, seed=100 + i) for i in range(8)]))
    import os
    done = int(open("models/ppo_steps.txt").read()) if os.path.exists("models/ppo_steps.txt") and os.path.exists("models/ppo_planner.zip") else 0
    if done > 0:
        model = PPO.load("models/ppo_planner", env=env, device="cpu")
        print(f"resuming from {done} steps", flush=True)
    else:
      model = PPO("MlpPolicy", env, n_steps=512, batch_size=256, n_epochs=10, gamma=0.99, gae_lambda=0.95,
                learning_rate=3e-4, clip_range=0.2, ent_coef=0.0, policy_kwargs=dict(net_arch=[128, 128]),
                seed=0, verbose=0, device="cpu")
    t0 = time.time()
    chunk = 100_000
    while done < steps:
        model.learn(total_timesteps=chunk, reset_num_timesteps=False)
        done += chunk
        ep_info = [e["r"] for e in model.ep_info_buffer] if model.ep_info_buffer else [0]
        print(f"{done} steps  ep_return {np.mean(ep_info):.2f}  {time.time()-t0:.0f}s", flush=True)
        model.save("models/ppo_planner")
        open("models/ppo_steps.txt", "w").write(str(done))
    print("saved")
