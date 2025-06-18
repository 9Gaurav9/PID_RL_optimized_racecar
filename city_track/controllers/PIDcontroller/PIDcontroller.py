from controller import Camera, Keyboard, GPS, Display, InertialUnit
from vehicle import Driver
import math
import numpy as np
import csv
import os

# === Constants ===
TIME_STEP = 50
KP, KI, KD = 0.25, 0.006, 2
FILTER_SIZE = 3
UNKNOWN = 99999.99
STATIONARY_THRESHOLD = 30

# === Lap Detection ===
# START_POS = None
START_POS = (-10, -105)
LAP_START_THRESHOLD = 20
LAP_END_THRESHOLD = 5
lap_started = False
lap_completed = False
stationary_counter = 0

# === Dynamic Throttle Logic ===
max_speed = 40.0
min_speed = 10.0

# === PID State ===
pid_reset = True
integral = 0
old_value = 0
angle_history = [0.0] * FILTER_SIZE

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

display = driver.getDevice("display")
display.setFont("Arial", 16, True)

 # === Data Logger ===
# log_file = "pid_data_log.csv"
log_file = os.path.join(os.path.dirname(__file__), "..", "..", "..", "Model", "pid_data_log.csv")
log = open(log_file, "w", newline="")
csv_writer = csv.writer(log)
# csv_writer.writerow(["x", "z", "speed", "steering", "reward"])
csv_writer.writerow(["line_offset", "speed", "yaw", "steer", "throttle", "reward"])


# === Utilities ===
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
        return UNKNOWN
    avg_x = sum_x / count
    return ((avg_x / camera_width) - 0.5) * camera_fov

def filter_angle(val):
    if val == UNKNOWN:
        return UNKNOWN
    angle_history.pop(0)
    angle_history.append(val)
    return sum(angle_history) / len(angle_history)

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

def update_display_status(text):
    display.setColor(0xFFFFFF)
    display.fillRectangle(0, 0, 256, 150)
    display.setColor(0x000000)
    display.drawText(text, 10, 20)

def check_lap(pos):
    global lap_started, lap_completed
    dx = pos[0] - START_POS[0]
    dz = pos[2] - START_POS[1]
    dist = math.sqrt(dx**2 + dz**2)
    print(f"Distance from start: {dist:.2f}")
    if not lap_started and dist > LAP_START_THRESHOLD:
        lap_started = True
        print("Lap started!")
    if lap_started and dist < LAP_END_THRESHOLD and not lap_completed:
        lap_completed = True
        print("Lap completed!")

# === Start the Car ===
driver.setCruisingSpeed(30)
# driver.setCruisingSpeed(throttle)
driver.setDippedBeams(True)
update_display_status("Waiting for GPS...")

# === Main Loop ===
while driver.step() != -1:
    key = keyboard.getKey()
    if key == keyboard.UP:
        driver.setCruisingSpeed(driver.getTargetCruisingSpeed() + 5)
    elif key == keyboard.DOWN:
        driver.setCruisingSpeed(driver.getTargetCruisingSpeed() - 5)

    pos = gps.getValues()
    speed = driver.getCurrentSpeed()

    if START_POS is None:
        if speed < 0.1:
            stationary_counter += 1
        else:
            stationary_counter = 0
        if stationary_counter >= STATIONARY_THRESHOLD:
            START_POS = (pos[0], pos[2])
            print(f"START_POS set to: {START_POS}")
        continue

    check_lap(pos)

    if lap_completed:
        update_display_status("Lap complete. Logging stopped.")
        log.close()
        break

    update_display_status("PID Logging...")
    image = camera.getImage()
    angle = filter_angle(process_camera(image))
    steer = apply_pid(angle)
    
    if angle != UNKNOWN:
        
        driver.setSteeringAngle(steer)
        driver.setBrakeIntensity(0.0)
        
        orientation = imu.getRollPitchYaw()
        yaw = np.clip(orientation[2], -np.pi, np.pi)  # normalize
        line_offset = angle / camera_fov if angle != UNKNOWN else 0.0
        # Observation for logging
        obs = [line_offset, speed / 50.0, yaw]
        # Dynamic throttle calculation here
        steer_abs = abs(steer)
        throttle = max_speed - (steer_abs / 0.5) * (max_speed - min_speed)
        throttle = np.clip(throttle, min_speed, max_speed)  # Reduce speed on sharp turns
        driver.setCruisingSpeed(throttle)
        
        reward = 1.0  # or customize based on tracking quality
        # csv_writer.writerow([line_offset, speed, yaw, steer, throttle, reward])
    else:
        driver.setBrakeIntensity(0.4)
        driver.setSteeringAngle(0.0)
        throttle = driver.getTargetCruisingSpeed()
        pid_reset = True
       
        orientation = imu.getRollPitchYaw()
        yaw = np.clip(orientation[2], -np.pi, np.pi)  # normalize
        line_offset = angle / camera_fov if angle != UNKNOWN else 0.0
        # Observation for logging
        obs = [line_offset, speed / 50.0, yaw]
        # csv_writer.writerow([line_offset, speed, yaw, steer, throttle, reward])
    csv_writer.writerow([line_offset, speed, yaw, steer, throttle, reward])
   