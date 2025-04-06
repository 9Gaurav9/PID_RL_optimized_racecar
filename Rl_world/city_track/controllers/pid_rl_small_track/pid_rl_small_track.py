from controller import Camera, GPS, Keyboard
from vehicle import Driver
import math

# Constants
TIME_STEP = 50
FILTER_SIZE = 3
KP = 0.2
KI = 0.001
KD = 0.7
UNKNOWN = 99999.99

# Initialize Webots driver and keyboard
driver = Driver()
keyboard = Keyboard()
keyboard.enable(TIME_STEP)

# Camera setup
camera = driver.getDevice("camera")
camera.enable(TIME_STEP)
camera_width = camera.getWidth()
camera_height = camera.getHeight()
camera_fov = camera.getFov()

# GPS setup
gps = driver.getDevice("gps")
gps.enable(TIME_STEP)

# PID state
pid_need_reset = True
old_value = 0.0
integral = 0.0
line_history = [0.0] * FILTER_SIZE

# Vehicle setup
driver.setCruisingSpeed(25)  # start slower for testing
driver.setDippedBeams(True)

# === PID Controller ===
def applyPID(angle):
    global old_value, integral, pid_need_reset
    if pid_need_reset:
        old_value = angle
        integral = 0.0
        pid_need_reset = False

    diff = angle - old_value
    if math.copysign(1, angle) != math.copysign(1, old_value):
        integral = 0.0
    if -30 < integral < 30:
        integral += angle
    old_value = angle
    return KP * angle + KI * integral + KD * diff

# === Pixel detection threshold ===
def is_white_pixel(b, g, r):
    return b + g + r > 450  # Adjust for slightly off-white/gray

# === Lane detection ===
def process_lane_center(image):
    left_sum = 0
    right_sum = 0
    left_count = 0
    right_count = 0

    for y in range(camera_height):
        for x in range(camera_width):
            pixel_index = 4 * (y * camera_width + x)
            b = image[pixel_index]
            g = image[pixel_index + 1]
            r = image[pixel_index + 2]

            if is_white_pixel(b, g, r):
                if x < camera_width / 2:
                    left_sum += x
                    left_count += 1
                else:
                    right_sum += x
                    right_count += 1

    print(f"Left: {left_count}, Right: {right_count}")  # Debug print

    if left_count > 0 and right_count > 0:
        left_avg = left_sum / left_count
        right_avg = right_sum / right_count
        lane_center = (left_avg + right_avg) / 2
        angle = ((lane_center / camera_width) - 0.5) * camera_fov
        return angle
    else:
        return UNKNOWN

# === Angle smoothing ===
def filter_angle(new_value):
    global line_history
    if new_value == UNKNOWN:
        return UNKNOWN
    line_history.pop(0)
    line_history.append(new_value)
    return sum(line_history) / len(line_history)

# === Keyboard interaction ===
def check_keyboard():
    key = keyboard.getKey()
    if key == keyboard.UP:
        driver.setCruisingSpeed(driver.getTargetCruisingSpeed() + 5)
    elif key == keyboard.DOWN:
        driver.setCruisingSpeed(driver.getTargetCruisingSpeed() - 5)

# === Main loop ===
while driver.step() != -1:
    check_keyboard()
    camera_image = camera.getImage()
    lane_angle = filter_angle(process_lane_center(camera_image))

    if lane_angle != UNKNOWN:
        driver.setBrakeIntensity(0.0)
        steer = applyPID(lane_angle)
        driver.setSteeringAngle(steer)
    else:
        print("❌ Lost lane — braking")
        driver.setBrakeIntensity(0.5)
        driver.setSteeringAngle(0.0)
        pid_need_reset = True
