# import gym
# from gym import spaces
# # import gymnasium as gym
# # from gymnasium import spaces
# import numpy as np

# class WebotsRacingEnv(gym.Env):
#     def __init__(self, sim_interface):
#         super(WebotsRacingEnv, self).__init__()
#         self.sim = sim_interface
#         self.action_space = spaces.Box(low=np.array([-0.5, 0.0]), high=np.array([0.5, 50.0]), dtype=np.float32)
#         self.observation_space = spaces.Box(low=-1.0, high=1.0, shape=(4,), dtype=np.float32)

#     def reset(self):
#         self.sim.reset_simulation()
#         obs = self.sim.get_observation()
#         return obs

#     def step(self, action):
#         steer, throttle = action
#         obs = self.sim.apply_action(steer, throttle)
#         reward = self.sim.compute_reward()
#         done = self.sim.check_done()
#         return obs, reward, done, {}

""" 2nd Try"""
import gym
from gym import spaces
import numpy as np
from sim_interface import SimInterface

class WebotsRacingEnv(gym.Env):
    def __init__(self):
        super(WebotsRacingEnv, self).__init__()
        self.sim = SimInterface()
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(3,), dtype=np.float32)
        # self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(6,), dtype=np.float32)

        self.action_space = spaces.Box(low=np.array([-0.5, 0.0]), high=np.array([0.5, 50.0]), dtype=np.float32)


    def reset(self):
        self.sim.reset_simulation()
        return self.sim.get_observation()

    # def step(self, action):
    #     steer, throttle = action
    #     obs = self.sim.apply_action(steer, throttle)
    #     reward = self.sim.compute_reward()
    #     done = self.sim.check_done()
    #     return obs, reward, done, {}
    
    # def step(self, action):
    #     steer, throttle = action
    #     obs = self.sim.apply_action(steer, throttle)

    #     # === Custom Reward Shaping ===
    #     base_reward = 1.0  # constant positive reward for surviving a step

    #     #  Encourage speed — penalize long laps
    #     elapsed_time = self.sim.get_elapsed_time()  # must be tracked inside sim
    #     time_penalty = 0.01 * elapsed_time

    #     #  Keep car on yellow line center
    #     line_offset = self.sim.get_line_offset()  # must return offset in [-1, 1]
    #     alignment_penalty = abs(line_offset)  # more off-center → more penalty

    #     #  Optional: detect crash or going off track
    #     crash_penalty = 0.0
    #     if self.sim.is_off_road():  # optional logic
    #         crash_penalty = 5.0

    #     #  Total reward
    #     reward = base_reward - time_penalty - alignment_penalty - crash_penalty

    #     done = self.sim.check_done()

    #     return obs, reward, done, {}

    # def step(self, action):
    #     steer, throttle = action
    #     obs = self.sim.apply_action(steer, throttle)

    #     # === Track current state ===
    #     pos = self.sim.get_position()          # Must implement get_position() in SimInterface
    #     speed = self.sim.get_speed()          # Must implement get_speed() in SimInterface
    #     line_offset = self.sim.get_line_offset()
    #     self.sim.last_pos = getattr(self.sim, "last_pos", pos)  # init if not present

    #     # === Reward Components ===
    #     forward_progress = pos[0] - self.sim.last_pos[0]   # assumes x-axis is forward
    #     center_alignment_penalty = abs(line_offset)
    #     throttle_bonus = throttle
    #     oversteering_penalty = abs(steer)
    #     idle_penalty = 1.0 if speed < 1.0 else 0.0
    #     crash_penalty = 5.0 if self.sim.is_off_road() else 0.0
    #     survival_bonus = 1.0

    #     # Update for next step
    #     self.sim.last_pos = pos

    #     # === Final Reward Calculation ===
    #     reward = 0.0
    #     reward += 2.0 * forward_progress                    # encourage progress
    #     reward -= 0.5 * center_alignment_penalty            # stay centered
    #     reward += 0.1 * throttle_bonus                      # drive faster
    #     reward -= 0.1 * oversteering_penalty                # smoother control
    #     reward -= 0.5 * idle_penalty                        # penalize stopping
    #     reward -= crash_penalty                             # harsh crash punishment
    #     reward += survival_bonus                            # always alive bonus
    #     if speed < 1.0:
    #         reward -= 1.0                         # Penalize idling

    #     if self.is_off_road():
    #         reward -= 5.0                         # Harsh crash penalty
        
    #     done = self.sim.check_done()
    #     return obs, reward, done, {}

    def step(self, action):
        steer, throttle = action
        obs = self.sim.apply_action(steer, throttle)

        pos = self.sim.get_position()
        speed = self.sim.get_speed()
        line_offset = self.sim.get_line_offset()
        elapsed_time = self.sim.get_elapsed_time()  # implement inside SimInterface

        self.sim.last_pos = getattr(self.sim, "last_pos", pos)
        forward_progress = pos[0] - self.sim.last_pos[0]
        self.sim.last_pos = pos

        # === Base step-wise shaping ===
        reward = 0.0
        reward += 2.0 * forward_progress
        reward -= 0.5 * abs(line_offset)
        reward += 0.1 * throttle
        reward -= 0.1 * abs(steer)
        reward -= 1.0 if speed < 1.0 else 0.0
        reward -= 5.0 if self.sim.is_off_road() else 0.0
        reward += 1.0  # survival bonus

        # === Lap completion bonus ===
        if self.sim.lap_completed():  # you need to implement this
            lap_time = elapsed_time
            reward += 100.0 - lap_time  # lower lap time = higher reward

        done = self.sim.check_done()
        return obs, reward, done, {}


    # ======= Rewqard function with overtaking ==========
    # def step(self, action):
    #     steer, throttle = action
    #     obs = self.sim.apply_action(steer, throttle)

    #     pos = self.sim.get_position()
    #     speed = self.sim.get_speed()
    #     line_offset = self.sim.get_line_offset()
    #     elapsed_time = self.sim.get_elapsed_time()
    #     opponent_pos = self.sim.get_opponent_position()

    #     self.sim.last_pos = getattr(self.sim, "last_pos", pos)
    #     forward_progress = pos[0] - self.sim.last_pos[0]
    #     self.sim.last_pos = pos

    #     reward = 0.0
    #     reward += 2.0 * forward_progress
    #     reward -= 0.5 * abs(line_offset)
    #     reward += 0.1 * throttle
    #     reward -= 0.1 * abs(steer)
    #     reward -= 1.0 if speed < 1.0 else 0.0
    #     reward -= 5.0 if self.sim.is_off_road() else 0.0
    #     reward += 1.0  # survival bonus

    #     # Lap completion bonus
    #     if self.sim.lap_completed():
    #         reward += 100.0 - elapsed_time

    #     # Overtaking bonus
    #     # if hasattr(self.sim, "last_overtake"):
    #     #     rel_dx = pos[0] - opponent_pos[0]
    #     #     rel_dz = abs(pos[2] - opponent_pos[2])
    #     #     if rel_dx > 2.0 and rel_dz > 0.5 and elapsed_time - self.sim.last_overtake > 5.0:
    #     #         reward += 10.0
    #     #         self.sim.last_overtake = elapsed_time

    #     done = self.sim.check_done()
    #     return obs, reward, done, {}


   
    def render(self, mode="human"):
        """
        Render the current state of the environment.

        Parameters:
        - mode (str): The mode in which to render the environment. Default is "human".
                    Other modes (e.g., "rgb_array") can be added if needed.

        This function can be extended to:
        - Display the simulation state visually (e.g., via OpenCV)
        - Print debug information such as position, speed, reward
        - Log internal state to console or files
        """
        pass
