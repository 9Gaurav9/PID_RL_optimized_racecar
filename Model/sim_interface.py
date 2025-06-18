import numpy as np
import time

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
