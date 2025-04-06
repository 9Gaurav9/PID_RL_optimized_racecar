from controller import Camera, Keyboard
from vehicle import Driver
import math



TIME_STEP = 50
KP = 0.25
KI = 0.006
KD = 2
FILTER_SIZE = 3
UNKNOWN = 99999.99

driver = Driver()
keyboard = Keyboard()
keyboard.enable(TIME_STEP)

camera = driver.getDevice("camera")
camera.enable(TIME_STEP)
camera_width = camera.getWidth()
camera_height = camera.getHeight()
camera_fov = camera.getFov()

# PID
pid_reset = True
integral = 0
old_value = 0
angle_history = [0.0] * FILTER_SIZE

# Color matching (yellow line)
def is_yellow(b, g, r):
    return abs(b - 95) + abs(g - 187) + abs(r - 203) < 30

def process_camera(image):
    sum_x = 0
    count = 0
    for y in range(camera_height // 2, camera_height):  # only bottom half
        for x in range(camera_width):
            idx = (y * camera_width + x) * 4
            b = image[idx]
            g = image[idx + 1]
            r = image[idx + 2]
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

# Start
driver.setCruisingSpeed(30)
driver.setDippedBeams(True)

while driver.step() != -1:
    key = keyboard.getKey()
    if key == keyboard.UP:
        driver.setCruisingSpeed(driver.getTargetCruisingSpeed() + 5)
    elif key == keyboard.DOWN:
        driver.setCruisingSpeed(driver.getTargetCruisingSpeed() - 5)

    image = camera.getImage()
    angle = filter_angle(process_camera(image))

    if angle != UNKNOWN:
        steer = apply_pid(angle)
        driver.setBrakeIntensity(0.0)
        driver.setSteeringAngle(steer)
    else:
        print("Lost line — Braking")
        driver.setBrakeIntensity(0.4)
        driver.setSteeringAngle(0.0)
        pid_reset = True
