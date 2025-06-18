from controller import Supervisor

supervisor = Supervisor()
TIME_STEP = 50

pid_car = supervisor.getFromDef("Blue Car")
rl_car = supervisor.getFromDef("Red car")

pid_trans_field = pid_car.getField("translation")
rl_data_field = rl_car.getField("customData")

while supervisor.step(TIME_STEP) != -1:
    pos = pid_trans_field.getSFVec3f()  # [x, y, z]
    custom_string = f"{pos[0]:.2f},{pos[2]:.2f}"
    rl_data_field.setSFString(custom_string)
