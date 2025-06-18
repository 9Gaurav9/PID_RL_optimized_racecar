from controller import Camera, Keyboard, GPS, Display, InertialUnit
from vehicle import Driver
import math
import numpy as np
from stable_baselines3 import PPO
import time





# === Constants ===
TIME_STEP = 50
UNKNOWN = 99999.99
FILTER_SIZE = 3
KP, KI, KD = 0.25, 0.006, 2
min_speed =10
max_speed = 40
MIN_LAP_DURATION = 30  # seconds

# === Lap Tracking State ===
lap_cooldown = 0  # Frames to wait before another lap can be triggered
LAP_EXIT_DIST = 20   # Must go this far before eligible to count lap
LAP_REENTRY_DIST = 5  # Must return within this to count lap
LAP_COOLDOWN_FRAMES = 100  # ~5 seconds @ 50ms step
lap_completed = False

# === Webots Devices ===
driver = Driver()
keyboard = Keyboard()
keyboard.enable(TIME_STEP)

camera = driver.getDevice("camera")
camera.enable(TIME_STEP)
camera_width = camera.getWidth()
camera_height = camera.getHeight()
camera_fov = camera.getFov()

gps = driver.getDevice("gps")
gps.enable(TIME_STEP)

imu = driver.getDevice("inertial unit")
imu.enable(TIME_STEP)

START_POS = None
stationary_counter = 0
STATIONARY_THRESHOLD = 30  # 1.5 seconds (if TIME_STEP is 50 ms)
last_lap_time = 0.0

display = driver.getDevice("display")
display.setFont("Arial", 16, True)

# === PPO Model ===
ppo_model = PPO.load("saved_model/ppo_model")
print("PPO model loaded")

# === PID Controller ===
angle_history = [0.0] * FILTER_SIZE
integral = 0
old_value = 0
pid_reset = True

# === Lap Timer ===
lap_start_time = time.time()
lap_started = False
crash_counter = 0
best_lap_time = float('inf') 

# === Functions ===
def update_display_status(text):
    display.setColor(0xFFFFFF)
    display.fillRectangle(0, 0, 350, 250)
    display.setColor(0x000000)
    display.drawText(text, 10, 20)

def is_yellow(b, g, r):
    return abs(b - 95) + abs(g - 187) + abs(r - 203) < 30


def process_camera(image):
    sum_x = 0
    count = 0
    for y in range(camera_height // 2, camera_height):
        for x in range(camera_width):
            idx = (y * camera_width + x) * 4
            b, g, r = image[idx], image[idx + 1], image[idx + 2]
            if is_yellow(b, g, r):
                sum_x += x
                count += 1
    if count == 0:
        print(" Yellow line not detected")
        return UNKNOWN
    print(f" Yellow pixels: {count}")
    avg_x = sum_x / count
    return ((avg_x / camera_width) - 0.5) * camera_fov

def filter_angle(val):
    if val == UNKNOWN:
        return UNKNOWN
    angle_history.pop(0)
    angle_history.append(val)
    return sum(angle_history) / len(angle_history)

# def check_lap(pos):
    # global lap_started, lap_start_time, last_lap_time
    # if START_POS is None:
        # return
    # dx = pos[0] - START_POS[0]
    # dz = pos[2] - START_POS[1]
    # dist = math.sqrt(dx**2 + dz**2)
    # if not lap_started and dist > 20:
        # lap_started = True
    # elif lap_started and dist < 5:
        # current_time = time.time()
        # last_lap_time = current_time - lap_start_time
        # lap_start_time = current_time
        # lap_started = False
        # print(f" Lap completed in {last_lap_time:.2f}s")

def check_lap(pos, heading):
    global lap_started, lap_completed, best_lap_time, lap_start_time, last_lap_time, lap_cooldown

    if START_POS is None:
        return

    dx = pos[0] - START_POS[0]
    dz = pos[2] - START_POS[1]
    dist = math.sqrt(dx**2 + dz**2)
    current_time = time.time()

    if lap_cooldown > 0:
        lap_cooldown -= 1
        return

    # Consider direction (only reenter if heading roughly matches start direction)
    correct_heading = abs(heading) < 0.6  # Tweak threshold based on START zone

    if not lap_started and dist > LAP_EXIT_DIST:
        lap_started = True
        lap_completed = False
        print("[INFO] Lap started.")

    elif lap_started and dist < LAP_REENTRY_DIST and not lap_completed and \
         (current_time - lap_start_time) > MIN_LAP_DURATION and correct_heading:
        last_lap_time = current_time - lap_start_time
        lap_start_time = current_time
        lap_completed = True
        lap_started = False
        lap_cooldown = LAP_COOLDOWN_FRAMES
        print(f"[INFO] Lap completed in {last_lap_time:.2f}s")
    
        # Update best lap time after lap completion
        if last_lap_time < best_lap_time:
            best_lap_time = last_lap_time
            print(f"[INFO] New best lap time: {best_lap_time:.2f}s")
    
def apply_pid(angle):
    global pid_reset, integral, old_value
    if pid_reset:
        old_value = angle
        integral = 0
        pid_reset = False
    if math.copysign(1, angle) != math.copysign(1, old_value):
        integral = 0
    diff = angle - old_value
    if -30 < integral < 30:
        integral += angle
    old_value = angle
    return KP * angle + KI * integral + KD * diff


def reset_car_state():
    global crash_counter, lap_started, lap_start_time
    driver.setSteeringAngle(0.0)
    driver.setCruisingSpeed(0.0)
    driver.setBrakeIntensity(1.0)
    time.sleep(0.2)
    crash_counter = 0
    lap_started = False
    lap_start_time = time.time()
    driver.setBrakeIntensity(0.0)

def predict_rl(state):
    global ppo_model
    if not np.isfinite(state).all():
        print("Skipping PPO predict due to invalid input:", state)
        return 0.0, 0.0

    try:
        action, _ = ppo_model.predict(state, deterministic=True)
    except Exception as e:
        print("PPO prediction error:", e)
        return 0.0, 0.0

    print("PPO raw action:", action)
    if not np.isfinite(action).all():
        print("PPO returned NaN/Inf action:", action)
        return 0.0, 0.0

    return action[0], action[1]

# === Start Driving ===
driver.setCruisingSpeed(0)
driver.setDippedBeams(True)

lap_display = driver.getDevice("lapDisplay")
lap_display.setFont("Arial", 14, True)

lap_start_time = time.time()

# === Persistent control memory ===
last_valid_throttle = 15.0
last_valid_steer = 0.0

while driver.step() != -1:
    # === Manual Override ===
    key = keyboard.getKey()
    if key == keyboard.UP:
        driver.setCruisingSpeed(driver.getTargetCruisingSpeed() + 5)
    elif key == keyboard.DOWN:
        driver.setCruisingSpeed(driver.getTargetCruisingSpeed() - 5)
    


    pos = gps.getValues()
    
    # speed = driver.getCurrentSpeed()
    # if START_POS is None and speed > 1.0:
        # START_POS = (pos[0], pos[2])
        # print(f"[INFO] START_POS set to {START_POS}")
    # else:
        # check_lap(pos)
    
    
    # === Ensure proper starting position (with camera + orientation + stillness)
    pos = gps.getValues()
    speed = driver.getCurrentSpeed()
    orientation = imu.getRollPitchYaw()
    image = camera.getImage()
    
    if START_POS is None:
        angle = filter_angle(process_camera(image))
        heading = orientation[2]
    
        position_stable = abs(speed) < 1.0
        yellow_line_detected = angle != UNKNOWN
        heading_ok = abs(heading) < 0.5
    
        if yellow_line_detected and heading_ok and position_stable:
            stationary_counter += 1
            if stationary_counter >= 20:
                START_POS = (pos[0], pos[2])
                driver.setCruisingSpeed(40)
                print(f"[INFO] START_POS dynamically set to {START_POS}")
        else:
            stationary_counter = 0
        continue  # Wait until we lock START_POS
    else:
        check_lap(pos, orientation[2])
    
    orientation = imu.getRollPitchYaw()
    image = camera.getImage()
    angle = filter_angle(process_camera(image))
    line_offset = angle / camera_fov if angle != UNKNOWN else 0.0
    
    

    # obs = np.array([line_offset, speed / 50.0, orientation[2]], dtype=np.float32)
    obs = np.array([line_offset, speed / 50.0, orientation[2]], dtype=np.float32)


    if not np.isfinite(obs).all():
        print("Invalid observation:", obs)
        continue

    update_display_status("Hybrid PID Steering + PPO Throttle")
    
    # === Predict PPO ===
    steer_rl, raw_throttle = predict_rl(obs)
    
    # === Steering Decision: Hybrid (PID + PPO) or PPO-only fallback ===
    if angle == UNKNOWN:
        print("Yellow line lost → using PPO steering only")
        steer = steer_rl
        # raw_throttle = 20.0

        
    else:
        steer_pid = apply_pid(angle)
        steer = 0.9 * steer_pid + 0.1 * steer_rl
    # steer_abs = abs(steer_rl)
    # throttle = max_speed - (steer_abs / 0.5) * (max_speed - min_speed)
    # throttle = np.clip(throttle, min_speed, max_speed)  # Reduce speed on sharp turns
    raw_throttle = np.clip(raw_throttle, 0.0, 1.0)

    
    # === PID + PPO Steering ===
    steer_pid = apply_pid(angle) if angle != UNKNOWN else 0.0
    steer = 0.85 * steer_pid + 0.15 * steer_rl
    
    # === PPO Throttle ===
    # throttle = 5.0 + raw_throttle * 25.0
    # throttle = float(np.clip(throttle, 15.0, 30.0))
    
    # === Base PPO throttle ===
    throttle = 5.0 + raw_throttle * 15.0  # base range: [5,30]
    
    # === Scale throttle down based on steering sharpness
    steer_magnitude = abs(steer)
    # steer_penalty = 1.0 - min(steer_magnitude / 0.5, 1.0)  # 1.0 for straight, → 0.0 for sharp
    # throttle *= steer_penalty  # reduce speed on curves
    throttle -= steer_magnitude * 10.0  # scale down up to 10 units ma
    
    # === Final clip
    throttle = float(np.clip(throttle, 18.0, 30.0))

    


    # === Lap Timing ===
    current_time = time.time()
    lap_time = current_time - lap_start_time

    # Reset if car is stuck or off-road
    if abs(line_offset) > 0.9 or speed < 0.3 or abs(steer) > 0.9:
        crash_counter += 1
    else:
        crash_counter = 0

    if crash_counter > 3:
        last_lap_time = lap_time
        lap_start_time = time.time()
        update_display_status(" Crash! Resetting...")
        reset_car_state()
        continue
   

    
    
    
    # === Update both displays ===
    update_display_status("Hybrid PID Steering + PPO Throttle")
    lap_display.setColor(0xFFFFFF)
    lap_display.fillRectangle(0, 0, 256, 64)
    lap_display.setColor(0x000000)
    lap_display.drawText(f"Time: {lap_time:.2f}s", 10, 10)
    lap_display.drawText(f"Last Lap: {last_lap_time:.2f}s", 10, 25)
    lap_display.drawText(f"Best Lap: {best_lap_time:.2f}s", 10, 45)


    # === Apply to vehicle ===
    print(f"Steer: {steer:.3f}, Throttle: {throttle:.2f}, LineOffset: {line_offset:.2f}, Yaw: {orientation[2]:.2f}")
    driver.setSteeringAngle(steer)
    driver.setCruisingSpeed(throttle)
    driver.setBrakeIntensity(0.0) 

