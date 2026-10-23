# The ROS driver for Unitree Robotics G1 Humanoid

## Setup

Please install ROS 2 and some development tools like rosdep, vcs2l at first.

Then build the packages.

```shell
source /opt/ros/<ROS DISTRO>/setup.bash
mkdir -p <colcon workspace>/src
cd <colcon workspace>/src
wget https://raw.githubusercontent.com/mqcmd196/g1_ros/refs/heads/master/jazzy.repos.yaml -O- | vcs import
sudo apt update && rosdep update && rosdep install -iqry --from-paths .
cd ..
colcon build --symlink-install --packages-up-to g1_bringup
```

## Execute

Please connect your computer to the robot following [official instruction](https://support.unitree.com/home/en/G1_developer/quick_development#heading-7). Please check the network interface name.

If the robot has Inspire RH56DFX hands and you want to move them, run the hand service:
```shell
ssh unitree@192.168.123.164  # Information required for ssh (IP address, user name, password): https://support.unitree.com/home/en/G1_developer/about_G1#heading-6
cd ~/dfx_inspire_service/build
sudo ./inspire_g1 -k -u  # This script must continue to run in the background. Required branch: https://github.com/pazeshun/dfx_inspire_service/tree/pazeshun-devel (https://github.com/unitreerobotics/dfx_inspire_service/pull/4 + https://github.com/unitreerobotics/dfx_inspire_service/pull/5
```

Then launch the nodes.

```shell
source <colcon workspace>/install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA # This requires for publishing/subscribing large data like PointCloud
ros2 launch g1_bringup g1_bringup.launch.py network_interface:=<network interface name> use_d435i:=true # if you want to use d435i on the G1
```

If you want to move Inspire RH56DFX hands, run the following instead:

```shell
ros2 launch g1_bringup g1_bringup.launch.py network_interface:=<network interface name> hand_type:=inspire_dfq
```

By default, the hands close when their hardware interface is deactivated.
To disable this behavior, run the following instead:

```shell
ros2 launch g1_bringup g1_bringup.launch.py network_interface:=<network interface name> hand_type:=inspire_dfq close_hand_on_deactivate:=false
```

To activate ROS controller

```shell
ros2 control set_controller_state upper_body_controller active
```

To run hand motion demo

```shell
ros2 run g1_bringup inspire_dfx_hand_open_close_demo --ros-args -p hand:=both
```

## Audio playback and TTS

Enable `use_audio:=true` on `g1_bringup.launch.py` or `hardware.launch.py`,
or run only audio (no motion controllers):

```shell
ros2 run g1_hardware g1_audio_interface_node --ros-args -p network_interface:=eth0
ros2 run g1_hardware play_audio.py /path/on/this/computer/speech.wav
ros2 topic pub --once /robotsound sound_play_msgs/msg/SoundRequest "{sound: -3, command: 1, volume: 0.5, arg: 'Hello from G1', arg2: '1'}"
# Stop this node's PCM playback and discard queued samples:
ros2 topic pub --once /robotsound sound_play_msgs/msg/SoundRequest "{sound: -1, command: 0}"
```

Install only the interface packages:
`sudo apt install ros-jazzy-audio-common-msgs ros-jazzy-sound-play-msgs`.
The playback implementation uses Unitree SDK, not `audio_play` or `sound_play`.

| Input topic | Type | Supported data |
| --- | --- | --- |
| `audio` | `audio_common_msgs/msg/AudioData` | Headerless **16000 Hz, mono, signed 16-bit little-endian PCM** |
| `audio_info` | `audio_common_msgs/msg/AudioInfo` | Optional: `channels: 1`, `sample_rate: 16000`, `sample_format: S16LE`, `coding_format: wave` |
| `robotsound` | `sound_play_msgs/msg/SoundRequest` | `SAY (-3)` / `PLAY_ONCE (1)`; `ALL (-1)` / `PLAY_STOP (0)` stops PCM only |

Following [audio_common ROS 2](https://github.com/ros-drivers/audio_common/tree/ros2),
`format: wave` means raw PCM on the topic, **not a RIFF/WAV file**.
The file publisher validates WAV and strips its header. Convert other formats first:
`ffmpeg -i input.mp3 -ar 16000 -ac 1 -c:a pcm_s16le speech.wav`.
Direct publishers must send complete two-byte samples, preferably 3200 bytes at 10 Hz.
The node paces transmission and rejects messages exceeding its 10-second pending buffer.
There is no resampling/MP3 decoding. Incompatible metadata blocks PCM until valid
metadata arrives; without metadata, the sender must ensure the format.
Metadata uses reliable, transient-local QoS; commands/data use reliable, volatile
QoS with depth 10. Use one audio producer at a time.

Read-only startup parameters are `format=wave`, `sample_format=S16LE`,
`sample_rate=16000`, `channels=1`, `depth=16` (other formats are rejected),
`app_name=g1_ros`, and `speaker_id=1`. Use distinct app names for separate nodes.
TTS `arg` is text; `arg2` is `0` (Chinese), `1` (English), or empty (use `speaker_id`).
For Japanese, synthesize externally and send PCM. `volume` is 0–1 and changes the
robot's **global** volume, including subsequent PCM. TTS clears pending PCM.
Avoid overlapping TTS and PCM: the SDK exposes neither TTS completion nor cancellation.

This supports a subset of `sound_play`: `PLAY_FILE`, looping, and built-in sounds
are rejected; no completion/cancellation action or custom service is provided.
Use `play_audio.py` for files. SDK errors are logged; topic delivery does not
acknowledge playback. A failed stop blocks PCM until another stop succeeds.
EOF does not stop playback, allowing the robot's buffered audio to finish.
See the [Unitree example](https://github.com/unitreerobotics/unitree_sdk2/blob/main/example/g1/audio/g1_audio_client_example.cpp)
and [client API](https://github.com/unitreerobotics/unitree_sdk2/blob/main/include/unitree/robot/g1/audio/g1_audio_client.hpp).

## gear_sonic (SONIC) locomotion

`g1_hardware` provides `gear_sonic_interface`, a bridge that sends locomotion, VR 3-point upper-body and SMPL-motion commands to the [GR00T-WholeBodyControl](https://github.com/NVlabs/GR00T-WholeBodyControl) (SONIC) deploy stack over ZMQ, and `gear_sonic_controller`, a `FollowJointTrajectory` action server that lets MoveIt execute trajectories through SONIC (FK → VR 3-point targets). Enable both with `use_gear_sonic:=true` on `hardware.launch.py` or `g1_bringup.launch.py`.

`zmq_host` is the address the interface **binds** its ZMQ XPUB socket to; the deploy stack connects to it. The default `127.0.0.1` only accepts a deploy stack running on the same machine (simulation). For the real robot, set `zmq_host:=0.0.0.0`.

> [!IMPORTANT]
> Run **all** ROS 2 nodes and CLI commands with a non-zero `ROS_DOMAIN_ID` (e.g. `export ROS_DOMAIN_ID=1`). unitree DDS (the sim / deploy stack / g1_hardware) is fixed to domain 0 and shares the RTPS port range with ROS 2 domain 0; with MoveIt + RViz running, domain 0 runs out of participant slots and the hardware interface fails with `Failed to find a free participant index`.

### Simulation (MuJoCo)

The MuJoCo simulator of GR00T-WholeBodyControl publishes the same DDS topics as the real robot (`rt/lowstate` etc.) on the loopback interface, so the ros2_control stack (`/joint_states`, TF) works unmodified with `network_interface:=lo`.

```shell
# Terminal 1: MuJoCo simulator (in GR00T-WholeBodyControl, .venv_sim)
python gear_sonic/scripts/run_sim_loop.py
# Terminal 2: SONIC controller (in gear_sonic_deploy)
./deploy.sh --input-type zmq_manager sim
# Terminal 3: ROS driver only
ros2 launch g1_hardware hardware.launch.py network_interface:=lo use_gear_sonic:=true
# ... or the full stack with MoveIt + RViz
ros2 launch g1_bringup g1_bringup.launch.py network_interface:=lo use_gear_sonic:=true
```

### Real robot

```shell
# On this computer
ros2 launch g1_bringup g1_bringup.launch.py network_interface:=<network interface name> hand_type:=inspire_dfq use_gear_sonic:=true zmq_host:=0.0.0.0
# On the robot (in gear_sonic_deploy)
./deploy.sh --input-type zmq_manager --zmq-host <this computer's IP> real
```

### Commanding the robot

Once the deploy stack has connected (see the `gear_sonic_interface` log), enable control and send velocity commands:

```shell
ros2 service call /gear_sonic_interface/enable_control std_srvs/srv/SetBool "{data: true}"
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}" -r 10
```

Locomotion modes (`~/mode/*` services), VR 3-point upper-body targets, base height (`~/target_height`) and full-body SMPL streaming are also available. See the header comment of `g1_hardware/include/g1_hardware/gear_sonic_interface.hpp` for the full interface. Pose targets are interpreted as URDF link poses (the `vr_3point_body_offset` keypoint offset is applied internally) and are tracked stiffly by default (`left_wrist_compliance` / `right_wrist_compliance` / `head_compliance` launch arguments, 0.0-1.0 each, 0.0 = exact tracking); targets are released when their topic stops for `pose_timeout` (0.5 s).

### MoveIt

With `use_gear_sonic:=true`, `g1_bringup.launch.py` routes MoveIt trajectory execution to `gear_sonic_controller` instead of the ros2_control `upper_body_controller`. Plan & Execute in RViz works as usual; only the wrist / torso poses are reproduced — the RL policy chooses its own whole-body joint angles.

The MoveIt model additionally gains a virtual prismatic joint `pelvis_height_joint` (world → pelvis), so IK can lower the whole body. Select the **`left_arm_with_waist`** or **`right_arm_with_waist`** planning group in RViz and drag the goal below the standing workspace: the plan uses the lift joint and, on execution, `gear_sonic_controller` converts it to the SONIC base-height command with an automatic `idle_squat`/`default` mode switch — the robot squats to reach, and stands back up when the goal returns high. Notes:

- `both_arms_with_waist` has no IK solver (two-tip group); its RViz markers fall back to the arm-only subgroups and cannot go low. Use it for joint-space goals only.
- The reachable wrist height is about 0.5 m below the standing pelvis (full lift + waist pitch limit).
- The RViz robot display keeps the legs straight while squatting (the lift joint is a planning-only dummy; the real legs are driven by the policy).

> [!WARNING]
> Do **not** activate `upper_body_controller` while SONIC is in control — its arm commands conflict with the whole-body policy. The Inspire hand controllers use separate DDS topics and can be used together with SONIC.

## Running as systemd services (on the robot)

The `etc/` directory mirrors the layout under `/etc` on the robot. `etc/systemd/` holds the unit files that run the stack as services:

| Unit | What it runs | Autostart |
|---|---|---|
| `inspire-g1.service` | Inspire RH56DFX hand service (`inspire_g1`) | disabled (pulled in by the bringup) |
| `gear-sonic.service` | SONIC deploy stack (`gear_sonic_deploy/deploy.sh`) | disabled (pulled in by the bringup) |
| `ros2-g1-gear-sonic-bringup.service` | `g1_bringup` in the `ghcr.io/mqcmd196/g1_ros` container via `rocker` | manual |

`etc/sysctl/` holds kernel tuning the DDS middleware needs:

| File | What it sets |
|---|---|
| `99-ros2-dds.conf` | large UDP socket buffers and netdev backlog (avoids dropped DDS packets, e.g. sensor data) |

### Install

An Ansible playbook installs both as **symlinks** back to the files in this repository (so editing a file here and reloading is enough — no copy to keep in sync): the units into `/etc/systemd/system` followed by `systemctl daemon-reload`, and the sysctl drop-ins into `/etc/sysctl.d` followed by `sysctl --system`.

Run on the robot itself:

```shell
ansible-playbook --ask-become-pass etc/install.yml
```

Or from another control node (override with the repo's absolute path **on the robot**):

```shell
ansible-playbook -i <robot-host>, -u unitree --become \
    -e g1_ros_dir=/home/unitree/ros/colcon_ws/src/g1_ros \
    etc/install.yml
```

No unit autostarts at boot. `ros2-g1-gear-sonic-bringup.service` pulls in `gear-sonic.service` and `inspire-g1.service` via `Requires=`, so starting the bringup starts all three.

### Start / stop

```shell
# Start SONIC + Inspire hand + the ROS bringup (pulled in via Requires=)
sudo systemctl start ros2-g1-gear-sonic-bringup
# Stop
sudo systemctl stop ros2-g1-gear-sonic-bringup gear-sonic inspire-g1
# Follow logs
journalctl -u ros2-g1-gear-sonic-bringup -f
journalctl -u gear-sonic -f
```

> [!NOTE]
> `ros2-g1-gear-sonic-bringup.service` runs the `ghcr.io/mqcmd196/g1_ros` container image (built from `master`), not your local workspace. Rebuild/pull the image to pick up changes that are merged to `master`.

## Contribution

### Prerequisite

```shell
sudo apt install black isort libxml2-utils pre-commit
```

### Pre-commit

Make sure `pre-commit` is run before commit.

If you want to execute `pre-commit` automatically in every `git commit` :

```shell
cd g1_ros
pre-commit install
```

If you want to execute `pre-commit` manually:

```shell
pre-commit run --all-files
```
