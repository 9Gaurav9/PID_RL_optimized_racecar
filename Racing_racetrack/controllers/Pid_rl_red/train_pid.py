import os
from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
from log_dataset import PIDLogDatasetEnv
import pandas as pd
import matplotlib.pyplot as plt
from stable_baselines3.common.vec_env import DummyVecEnv
from webots_racing_env import WebotsRacingEnv


# === Directories ===
os.makedirs("saved_model", exist_ok=True)
os.makedirs("logs_pid", exist_ok=True)
os.makedirs("logs_webots", exist_ok=True) 

# === Phase 1: Train on PID logs ===
print("🚗 Phase 1: Training PPO on PID logs")
pid_env = PIDLogDatasetEnv("pid_data_log.csv")
model = PPO("MlpPolicy", pid_env, verbose=1, tensorboard_log="./logs_pid")
model.learn(total_timesteps=500_000)
model.save("saved_model/ppo_model_pretrained")
print("✅ PPO pretrained on PID data")

