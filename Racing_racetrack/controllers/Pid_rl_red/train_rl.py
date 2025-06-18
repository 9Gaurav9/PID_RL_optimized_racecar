import os
from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
from log_dataset import PIDLogDatasetEnv
import pandas as pd
import matplotlib.pyplot as plt
from stable_baselines3.common.vec_env import DummyVecEnv
from webots_racing_env import WebotsRacingEnv


# sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# from stable_baselines3 import PPO
# from webots_racing_env import WebotsRacingEnv
# from sim_interface import SimInterface

# # Create output directory
# os.makedirs("saved_model", exist_ok=True)

# # Create simulation interface and environment
# sim = SimInterface()
# env = WebotsRacingEnv(sim)

# # Train PPO
# model = PPO("MlpPolicy", env, verbose=1)
# model.learn(total_timesteps=100000)

# # Save model
# model.save("saved_model/ppo_model")
# print("Model saved to saved_model/ppo_model.zip")


"""2nd Try """
# import os
# from stable_baselines3 import PPO
# from webots_racing_env import WebotsRacingEnv
# from stable_baselines3.common.callbacks import EvalCallback

# os.makedirs("saved_model", exist_ok=True)
# log_dir = "./logs"
# os.makedirs(log_dir, exist_ok=True)


# env = WebotsRacingEnv()
# model = PPO("MlpPolicy", env, verbose=1, tensorboard_log=log_dir)
# model.learn(total_timesteps=1000000)  
# model.save("saved_model/ppo_model")

# print("Model saved to saved_model/ppo_model.zip")

# To view PPO stats tensorboard --logdir=./logs


"""3nd try - Working """
# import os
# from stable_baselines3 import PPO
# from stable_baselines3.common.vec_env import DummyVecEnv
# from log_dataset import PIDLogDatasetEnv
# from stable_baselines3.common.monitor import Monitor
# from webots_racing_env import WebotsRacingEnv

# os.makedirs("saved_model", exist_ok=True)
# log_dir = "./logs"
# os.makedirs(log_dir, exist_ok=True)

# # Wrap Webots env
# env = DummyVecEnv([lambda: Monitor(WebotsRacingEnv(), log_dir)])

# # Load env from PID logs
# env = PIDLogDatasetEnv("pid_data_log.csv")

# # PPO model
# model = PPO("MlpPolicy", env, verbose=1, tensorboard_log=log_dir)


# # Train
# model.learn(total_timesteps=1000000) #timesteps = 1000000

    
# # Save
# model.save("saved_model/ppo_model")
# print("✅ Model saved to saved_model/ppo_model.zip")




# === Directories ===
# os.makedirs("saved_model", exist_ok=True)
# os.makedirs("logs_pid", exist_ok=True)
# os.makedirs("logs_webots", exist_ok=True) 

# # === Phase 1: Train on PID logs ===
# # print(" Phase 1: Training PPO on PID logs")
pid_env = PIDLogDatasetEnv("pid_data_log.csv")
model = PPO("MlpPolicy", pid_env, verbose=1, tensorboard_log="./logs_pid")
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


# import os
# from stable_baselines3 import PPO
# from stable_baselines3.common.vec_env import DummyVecEnv
# from webots_racing_env import WebotsRacingEnv

# === Setup log directory for online training ===
# log_dir = "./logs_webots"
# os.makedirs(log_dir, exist_ok=True)

# # === Wrap Webots simulator env ===
# env = DummyVecEnv([lambda: WebotsRacingEnv()])

# # === Load the pretrained PPO model from Phase 1 ===
# model = PPO.load("saved_model/ppo_model_pretrained", env=env, verbose=1, tensorboard_log=log_dir)

# # === Continue training in Webots simulator ===
# model.learn(total_timesteps=500_000)

# # === Save final model ===
# model.save("saved_model/ppo_model_final")
# print("✅ Phase 2: PPO fine-tuned on Webots. Model saved.")
