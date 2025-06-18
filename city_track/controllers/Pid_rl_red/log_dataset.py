# log_dataset.py
# import pandas as pd
# import numpy as np
# import gym
# from gym import spaces

# class PIDLogDatasetEnv(gym.Env):
#     def __init__(self, csv_path):
#         self.data = pd.read_csv(csv_path).dropna()
#         self.states = self.data[['x', 'z', 'speed']].values.astype(np.float32)
#         self.actions = self.data[['steering', 'throttle']].values.astype(np.float32)
#         self.index = 0

#         self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(3,), dtype=np.float32)
#         self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(1,), dtype=np.float32)

#     def reset(self):
#         self.index = 0
#         return self.states[self.index]

#     def step(self, action):
#         obs = self.states[self.index]
#         true_action = self.actions[self.index]
#         reward = -np.abs(true_action - action).sum()  # imitation loss as negative reward
#         self.index += 1
#         done = self.index >= len(self.states) - 1
#         return obs, reward, done, {}


import gym
from gym import spaces
import pandas as pd
import numpy as np

class PIDLogDatasetEnv(gym.Env):
    def __init__(self, csv_path="pid_data_log.csv"):
        super(PIDLogDatasetEnv, self).__init__()
        self.data = pd.read_csv(csv_path)

        print("Before cleaning:", self.data.shape)

        # Fill missing throttle values with constant (e.g., PID cruise speed = 30)
        if "throttle" in self.data.columns:
            self.data["throttle"] = self.data["throttle"].fillna(30.0)
        else:
            self.data["throttle"] = 30.0  # create it if missing

        self.data = self.data.dropna()  # Drop rows with remaining NaN
        print("After cleaning:", self.data.shape)

        # Extract state-action pairs
        self.states = self.data[["line_offset", "speed", "yaw"]].values.astype(np.float32)
        # self.states = np.stack([
        #     self.data["line_offset"].values.astype(np.float32),
        #     (self.data["speed"].values / 50.0).astype(np.float32),
        #     self.data["yaw"].values.astype(np.float32),
        # ], axis=1)
        self.actions = self.data[["steer", "throttle"]].values.astype(np.float32)
 
        if len(self.states) == 0:
            raise RuntimeError(" No valid data after cleaning the log file.")

        self.index = 0

        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(3,), dtype=np.float32)
        self.action_space = spaces.Box(low=np.array([-0.5, 0.0]), high=np.array([0.5, 50.0]), dtype=np.float32)

    def reset(self):
        self.index = 0
        return self.states[self.index]

    def step(self, action):
        self.index += 1
        done = self.index >= len(self.states) - 1
        reward = self.data.iloc[self.index]["reward"]
        return self.states[self.index], reward, done, {}

    def render(self, mode="human"):
        pass
