# PICO Body and Finger Teleoperation

This procedure uses the robot and workstation setup validated in testing: the shared image `ghcr.io/jsk-ros-pkg/g1_ros:latest`, `use_gear_sonic:=true`, and the original manager with optical-hand export and the zero-angle rotation fix.

**Startup order: A starts the manager → B starts the adapters → C enables body control → ABXY starts balance → AX starts body following → enable finger control separately.**

## Preparation

- Support the robot with the harness, with both feet on the ground.
- Connect the workstation and PICO to the G1 Wi-Fi.
- If a previous session is still running, follow the Stop section before starting another session.
- Prepare three terminals: A connects to G1; B and C run on the workstation.

## 1. Terminal A: Start the robot services and manager

```bash
ssh unitree@10.42.0.1
```

After logging into G1, run:

```bash
sudo systemctl start ros2-g1-gear-sonic-bringup.service

cd ~/GR00T-WholeBodyControl
.venv_teleop/bin/python -B gear_sonic/scripts/pico_manager_thread_server.py \
  --manager --port 5558 --optical-hand-port 5560
```

Open XRoboToolkit on PICO, set **PC Service IP to `10.42.0.1`**, and complete body and leg tracker calibration.

Wait for `Manager controls` in A. Keep A running. **Do not press ABXY yet; the manager must remain OFF.**

## 2. Workstation terminals B and C: Set up the environment

**Run this once in each terminal, B and C:**

```bash
source /opt/ros/jazzy/setup.bash
source /home/cici/projects/GrootReproucing/g1_ros/install/setup.bash
export ROS_DOMAIN_ID=164
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY ROS_DISCOVERY_SERVER
```

## 3. Terminal B: Start the body and hand adapters

```bash
ros2 launch g1_pico_teleop pico_body_hands.launch.py \
  body_zmq_endpoint:=tcp://10.42.0.1:5558 \
  hand_zmq_endpoint:=tcp://10.42.0.1:5560
```

Wait for this message in B:

```text
Manager mode: None -> 0
```

## 4. Terminal C: Enable body control

```bash
ros2 service call /pico_body_adapter/enable std_srvs/srv/SetBool '{data: true}'
```

Wait for **C to return `success=True` with `body armed` in the response**. Confirm that B has not subsequently reported `Body disarmed` before pressing controller buttons.

## 5. PICO controllers: ABXY → AX

The PICO operator should stand upright with feet together, upper arms close to the torso, elbows bent approximately 90 degrees, and palms facing each other.

1. Press **A+B+X+Y** together, then release all four buttons.
2. Wait for `OFF -> PLANNER` in A and **`SONIC balancing`** in B.
3. Wait at least **3 seconds** and confirm that the robot is stable.
4. Press **A+X** together, then release both buttons.
5. Wait for **`Body following active through ROS SmplMotion`** in B before testing small, slow arm movements.

Pressing AX again during body following returns to balance. This procedure uses only ABXY and AX; do not use BY, joystick locomotion, or other mode combinations.

**If `Body disarmed` appears or an expected success message is missing, stop at that step. Do not continue to AX or repeatedly press ABXY.**

## 6. Terminal C: Enable finger control

Gently put down the controllers and keep both hands in view of the headset cameras. Body following remains active while putting down the controllers.

Wait for **`left=ok right=ok`** in A, then run:

```bash
ros2 service call /pico_hand_adapter/enable std_srvs/srv/SetBool '{data: true}'
```

Wait for **`PICO hand output enabled`** in B, then slowly open and close your hands to test finger control.

## 7. Stop

In C, disable finger control first, then body control:

```bash
ros2 service call /pico_hand_adapter/enable std_srvs/srv/SetBool '{data: false}'
ros2 service call /pico_body_adapter/enable std_srvs/srv/SetBool '{data: false}'
```

Wait for **`Body disabled; ROS bridge returned to balance`** in B. SONIC is still running at this point.

Press `Ctrl+C` in A and B to exit the manager and adapters.

**Confirm that the harness supports the robot**, then run in the G1 shell in A:

```bash
sudo systemctl stop ros2-g1-gear-sonic-bringup.service gear-sonic.service inspire-g1.service
```

This ends SONIC balance control. For the next session, start again from step 1 and keep the manager OFF until body control is enabled.

## Build before first use or after local code changes

Run on the workstation. Rebuilding is not needed for routine startup or after rebooting the workstation.

```bash
cd /home/cici/projects/GrootReproucing/g1_ros
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to g1_pico_teleop \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```
