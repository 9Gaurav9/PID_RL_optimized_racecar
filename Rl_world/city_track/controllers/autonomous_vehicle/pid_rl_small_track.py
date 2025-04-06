from controller import Robot, Camera, Lidar, GPS, Keyboard
from vehicle import Driver
import math

# Constants
TIME_STEP = 50
FILTER_SIZE = 3
KP = 0.25
KI = 0.006
KD = 2
UNKNOWN = 99999.99

# Robot initialization
robot = Robot()
driver = Driver()
keyboard = Keyboard()
keyboard.enable(TIME_STEP)

# Devices
camera = robot.getDevice("camera")
camera.enable(TIME_STEP)
camera_width = camera.getWidth()
camera_height = camera.getHeight()
camera_fov = camera.getFov()

gps = robot.getDevice("gps")
gps.enable(TIME_STEP)

# PID Control state
pid_need_reset = True
old_value = 0.0
integral = 0.0

# Line filter
line_history = [0.0] * FILTER_SIZE

# Set driving config
driver.setCruisingSpeed(50)
driver.setDippedBeams(True)
driver.setWiperMode(Driver.WIPER_SLOW)

# Helper functions
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

def color_diff(a, b):
    return sum(abs(int(a[i]) - b[i]) for i in range(3))

def process_camera_image(image):
    # ref = [203, 187, 95]  # Webots camera image is in BGRA (Yellow) 
    ref = [255, 255, 255]#White 
    sum_x = 0
    count = 0
    for y in range(camera_height):
        for x in range(camera_width):
            r = Camera.imageGetRed(image, camera_width, x, y)
            g = Camera.imageGetGreen(image, camera_width, x, y)
            b = Camera.imageGetBlue(image, camera_width, x, y)
            if color_diff([r, g, b], ref[::-1]) < 30:
                sum_x += x
                count += 1
    if count == 0:
        return UNKNOWN
    avg_x = sum_x / count
    return ((avg_x / camera_width) - 0.5) * camera_fov

def filter_angle(new_value):
    global line_history
    if new_value == UNKNOWN:
        return UNKNOWN
    line_history.pop(0)
    line_history.append(new_value)
    return sum(line_history) / len(line_history)

def check_keyboard():
    key = keyboard.getKey()
    if key == keyboard.UP:
        driver.setCruisingSpeed(driver.getTargetCruisingSpeed() + 5)
    elif key == keyboard.DOWN:
        driver.setCruisingSpeed(driver.getTargetCruisingSpeed() - 5)

# Main loop
while robot.step(TIME_STEP) != -1:
    check_keyboard()
    camera_image = camera.getImage()
    yellow_line_angle = filter_angle(process_camera_image(camera_image))

    if yellow_line_angle != UNKNOWN:
        driver.setBrakeIntensity(0.0)
        steer = applyPID(yellow_line_angle)
        driver.setSteeringAngle(steer)
    else:
        driver.setBrakeIntensity(0.4)
        pid_need_reset = True
