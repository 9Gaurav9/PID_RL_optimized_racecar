import numpy as np
import time

# class SimInterface:
#     def __init__(self):
#         self.step_count = 0

#     def reset_simulation(self):
#         self.step_count = 0
#         print("Simulation reset")

#     def get_observation(self):
#         return np.random.uniform(-1.0, 1.0, 4).astype(np.float32)

#     def apply_action(self, steer, throttle):
#         self.step_count += 1
#         # Just return new dummy state for now
#         return self.get_observation()

#     def compute_reward(self):
#         # Example reward function
#         return 1.0

#     def check_done(self):
#         return self.step_count >= 200

"""2nd Try"""
# import numpy as np
# import time

# class SimInterface:
#     def __init__(self):
#         self.reset_simulation()

#     def reset_simulation(self):
#         self.steps = 0
#         self.last_position = np.zeros(2)
#         self.start_time = time.time()  # Track when lap started
#         self.last_line_offset = 0.0     # Track alignment (placeholder)

#     def get_observation(self):
#         # Replace this with your actual state observation logic
#         return np.zeros(4, dtype=np.float32)

#     def apply_action(self, steer, throttle):
#         # You could simulate a physics model or return mock state
#         self.steps += 1
#         return self.get_observation()

#     def compute_reward(self):
#         # Very basic reward shaping for demo
#         return 1.0

#     def check_done(self):
#         return self.steps > 200
#     def get_elapsed_time(self):
#         return time.time() - self.start_time  # self.start_time should be set in reset()

#     def get_line_offset(self):
#         # Extract from camera image and return how far car is from line center (e.g., -1 to 1)
#         return self.last_line_offset

"""3nd Try"""
# import numpy as np
# import time

# class SimInterface:
#     def __init__(self):
#         self.reset_simulation()

#     def reset_simulation(self):
#         self.steps = 0
#         self.last_position = np.zeros(2)
#         self.start_time = time.time()  # Track when lap started
#         self.last_line_offset = 0.0     # Track alignment (placeholder)

#     # def get_observation(self):
#     #     # Replace with real observation vector (e.g., x, z, speed, steering)
#     #     # Here we return dummy state: x, z, speed, line_offset
#     #     return np.array([0.0, 0.0, 30.0, self.last_line_offset], dtype=np.float32)
#     def get_observation(self):
#         norm_speed = 30.0 / 50.0  # Normalize speed
#         return np.array([0.0, 0.0, norm_speed, self.last_line_offset], dtype=np.float32)
    
#     def apply_action(self, steer, throttle):
#         self.steps += 1

#         # Mock behavior — update line offset slightly (simulate car veering)
#         self.last_line_offset += np.random.uniform(-0.02, 0.02)  # drift simulation
#         self.last_line_offset = np.clip(self.last_line_offset, -1.0, 1.0)

#         return self.get_observation()

#     def compute_reward(self):
#         # Reward = 1.0 - penalty based on line offset
#         alignment_penalty = abs(self.last_line_offset)
#         return 1.0 - alignment_penalty  # Max = 1.0 when aligned

#     def check_done(self):
#         return self.steps > 200

#     def get_elapsed_time(self):
#         return time.time() - self.start_time

#     def is_off_road(self):
#         return abs(self.last_line_offset) > 0.9  # simulate going off track


"""4 try """
# class SimInterface:
#     def __init__(self):
#         self.reset_simulation()

#     def reset_simulation(self):
#         self.steps = 0
#         self.last_position = np.array([0.0, 0.0])  # x, z position
#         self.current_position = np.array([0.0, 0.0])
#         self.start_time = time.time()
#         self.last_line_offset = 0.0
#         self.speed = 30.0  # Starting speed

#     def get_observation(self):
#         norm_speed = self.speed / 50.0  # Normalize to [0,1] range
#         return np.array([self.current_position[0], self.current_position[1], norm_speed, self.last_line_offset], dtype=np.float32)

#     def apply_action(self, steer, throttle):
#         self.steps += 1

#         # Simulate forward motion and steer-induced drift
#         delta = throttle * 0.01
#         self.current_position[0] += delta
#         self.speed = throttle

#         # Simulate random drift for line offset
#         self.last_line_offset += np.random.uniform(-0.02, 0.02)
#         self.last_line_offset = np.clip(self.last_line_offset, -1.0, 1.0)

#         return self.get_observation()
    
#     def lap_completed(self):
#         if self.lap_done:
#             self.lap_done = False
#             self.start_time = time.time()  # reset for next lap
#             return True
#         return False

#     def get_position(self):
#         return self.current_position

#     def get_speed(self):
#         return self.speed

#     def get_elapsed_time(self):
#         return time.time() - self.start_time

#     def check_done(self):
#         return self.steps > 200  # or another condition

#     def is_off_road(self):
#         return abs(self.last_line_offset) > 0.9

"""5th try - added penalty for increase lap time """
class SimInterface:
    def __init__(self):
        self.reset_simulation()

    def reset_simulation(self):
        self.steps = 0
        self.last_position = np.array([0.0, 0.0])  # x, z position
        self.current_position = np.array([0.0, 0.0])
        self.start_time = time.time()
        self.last_line_offset = 0.0
        self.speed = 30.0
        self.lap_started = False
        self.lap_done = False
        self.last_overtake = -10.0
        # self.opponent_pos = np.array([self.current_position[0] - 3.0, 0.0])  # 3m behind initially
        # self.last_overtake = -10.0
        
    def get_observation(self):
        norm_speed = self.speed / 50.0
        return np.array([self.current_position[0], norm_speed, self.last_line_offset], dtype=np.float32)
        # return np.array([
        #     self.current_position[0], self.current_position[1],
        #     norm_speed, self.last_line_offset
        # ], dtype=np.float32)
    
    # def get_observation(self):
    #     norm_speed = self.speed / 50.0
    #     rel_dx = self.opponent_pos[0] - self.current_position[0]
    #     rel_dz = self.opponent_pos[1] - self.current_position[1]

    #     return np.array([
    #         self.current_position[0],  # forward
    #         self.current_position[1],  # lateral
    #         norm_speed,
    #         self.last_line_offset,
    #         rel_dx,
    #         rel_dz
    #     ], dtype=np.float32)


    def apply_action(self, steer, throttle):
        self.steps += 1

        # Update position
        delta = throttle * 0.01
        self.current_position[0] += delta  # forward
        self.speed = throttle

        # Simulate drift
        self.last_line_offset += np.random.uniform(-0.02, 0.02)
        self.last_line_offset = np.clip(self.last_line_offset, -1.0, 1.0)

        # Lap logic
        dx = self.current_position[0] - self.last_position[0]
        dist = abs(dx)

        if not self.lap_started and dist > 1.0:
            self.lap_started = True

        if self.lap_started and dist < 0.3 and not self.lap_done and self.get_elapsed_time() > 5.0:
            self.lap_done = True
            
        # self.current_position[1] = np.clip(self.current_position[1] + steer * 0.05, -1.0, 1.0)
        # self.opponent_pos[0] += 0.15  # Simulate constant-speed PID car

        return self.get_observation()

    def lap_completed(self):
        if self.lap_done:
            self.lap_done = False
            self.start_time = time.time()  # reset for next lap
            return True
        return False

    def get_position(self):
        return self.current_position

    def get_speed(self):
        return self.speed

    def get_elapsed_time(self):
        return time.time() - self.start_time

    def check_done(self):
        return self.steps > 200

    def is_off_road(self):
        return abs(self.last_line_offset) > 0.9

    def get_line_offset(self):
        return self.last_line_offset
