import os
from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
from log_dataset import PIDLogDatasetEnv
import pandas as pd
import matplotlib.pyplot as plt
from stable_baselines3.common.vec_env import DummyVecEnv
from webots_racing_env import WebotsRacingEnv




# === Directories ===
# os.makedirs("saved_model", exist_ok=True)
# os.makedirs("logs_pid", exist_ok=True)
# os.makedirs("logs_webots", exist_ok=True) 

# # === Phase 1: Train on PID logs ===
# # print(" Phase 1: Training PPO on PID logs")
pid_env = PIDLogDatasetEnv("pid_data_log.csv")
# model = PPO("MlpPolicy", pid_env, verbose=1, tensorboard_log="./logs_pid")
model = PPO(
    "MlpPolicy",
    pid_env,
    verbose=1,
    tensorboard_log="./logs_pid",
    learning_rate=3e-4,
    batch_size=64
)

# # model.learn(total_timesteps=500_000)
# # model.save("saved_model/ppo_model_pretrained")
# # print(" PPO pretrained on PID data")

# # === Phase 2: Fine-tune in Webots simulation ===
# print("\n Phase 2: Fine-tuning PPO in Webots simulation")
sim_env = DummyVecEnv([lambda: WebotsRacingEnv()])
model.set_env(sim_env)
# model.set_tensorboard_log("logs_webots")
model.learn(total_timesteps=1000000)
model.save("saved_model/ppo_model_final")
print(" PPO fine-tuned in simulation")


# To view PPO stats tensorboard --logdir=./logs_pid
