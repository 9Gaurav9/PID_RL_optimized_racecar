#========= Webots Racing Simulation (PID + RL Hybrid Control) =================

This simulation project uses Webots to control a racing car using a hybrid approach first logging data via a PID controller, and then optimizing speed using a Reinforcement Learning (RL) model trained using Stable-Baselines3.

---

##  Requirements

 [Webots](httpscyberbotics.com) (R2022a or later)
 Python 3.8+
 Required Python packages

  ```bash
  pip install stable-baselines3==1.8.0 opencv-python-headless numpy matplotlib tqdm
  ```

---

##  Folder Structure

```
RL_world
├── City_track             # World for training and data logging
|	└── worlds
|		└── city.wbt
├── Test_racetrack         # World for model testing and evaluation
|	└── worlds
|		└── city_traffic.wbt
├── models                 # Includes the saved RL models and logs
│   └── saved_model
│       └── ppo_model.zip
├── controllers (Can be changed through Webots Model Tree)
│   ├── PIDcontroller      # Phase 1 Logs PID data
│   └── Pid_rl_red         # Phase 2 Hybrid controller with PPO + PID
└── README.md
```

---

## ================================ Steps to Run (No Docker) ==================

###  Phase 1 Log Training Data using PID

1. Open Webots and load the world

   ```
   RL_world/City_track//worlds/city.wbt
   ```

2. In the Scene Tree on the left, expand the car node (e.g. `BMWX5` or `Red Car`).

3. Set the Controller to

   ```
   PIDcontroller
   ```
	And Pid_rl_red for Rl model after Training

4. Run the simulation. This will log training data (to `models/pid_data_log.csv`).

---

###  Phase 2 Optimize with RL Model

1. After logging is complete, train your PPO model (optional if already trained)
	To train from PID logs initially

   ```bash
   python train_pid.py
   ```
	Then the remaining rl model with headless GUi WEbots 
	Run in One Terminal

   ```bash
   python train_rl.py

   ``` 
	In another Terminal 
   ```bash
   webots --batch --no-rendering "C:\Users\ugaur\Downloads\Rl_world\city_track\worlds\city.wbt"
   ```
 
   This uses the data from `pid_data_log.csv` and saves the model to `models/saved_model/ppo_model.zip`.

2. In Webots, reopen the same world

   ```
   RL_world/City_track/city.wbt
   ```

3. Set the Controller of the car to

   ```
   Pid_rl_red
   ```

4. Run simulation again. The car now starts with PID, transitions to RL, and attempts to optimize throttle and lap time.

---

###  Final Test Evaluate RL vs PID on Race Track

1. Open

   ```
   RL_world/Test_racetrack/city_traffic.wbt
   ```

2. Set the Controller to

   ```
   Pid_rl_red
   ```

3. Run the simulation. This world is configured for final model testing with PID car leading and RL car trying to overtake.

---

##  Notes

 `train_rl.py` uses `pid_data_log.csv` as input for RL training.
 `Pid_rl_red` loads the model from

  ```
  modelssaved_modelppo_model.zip
  ```
 Ensure all `.py` controllers are inside their correct `controllersname` folders as per Webots structure.

## =============================== Steps to Run ( Docker) ===========================


#---- Install Xming for Display (In Windows Display)
# Build the Docker image
docker build -t webots_rl .

# Run the container (Webots GUI should appear on host machine)
docker run -it \
  --env DISPLAY=$DISPLAY \
  --env QT_X11_NO_MITSHM=1 \
  --network host \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  --name webots_container \
  webots_rl


#------Note----
If any error comes up like 
""" docker: Error response from daemon: Conflict. The container name "/webots_container" is already in use by container """"

Use another container name such as 
docker run -it \
  --env DISPLAY=$DISPLAY \
  --env QT_X11_NO_MITSHM=1 \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  --name webots_rl_test \
  webots_rl

# After webots display shows up 
Got to Open World ----> /.app/ (Select any track) e.g city_track /worlds/ city.wbt

Then the world launches 
Select the ttpe of controller you need and run





