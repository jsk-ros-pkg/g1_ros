# Inspire Hand ROS 2

ROS 2 Jazzy driver for one physical Inspire RH56DFX hand per serial port (115200 baud, device ID 1). The driver exposes six actuated joints in radians through ros2_control.

## Build

Place this directory under a colcon workspace's `src` directory. From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src/inspire_hand_ros2 --ignore-src -r -y
colcon build --symlink-install --packages-up-to inspire_hand_driver
source install/setup.bash
```

## Run one hand

Stop any process using the port, give the launch user read/write access, then run:

```bash
ros2 launch inspire_hand_driver rh56dfx.launch.py device:=/dev/ttyUSB0 side:=right
```

Use `side:=left` for the left-hand kinematic frames. Both sides use the same six joint names. To run two standalone hands, launch each with its own `device` and `namespace` (default `inspire_hand`). The driver clears errors once on activation and starts from measured joint positions. To leave the hand in its last pose on shutdown, set `close_hand_on_deactivate:=false`.

To open the hand over two seconds and read its measured positions:

```bash
ros2 action send_goal /inspire_hand/position_joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory '{
    trajectory: {
      joint_names: [pinky_proximal_joint, ring_proximal_joint, middle_proximal_joint,
                    index_proximal_joint, thumb_proximal_pitch_joint, thumb_proximal_yaw_joint],
      points: [{positions: [0, 0, 0, 0, 0, 0], time_from_start: {sec: 2}}]
    }
  }'
ros2 topic echo --once /inspire_hand/joint_states
```

Stop trajectories and leave the fingers unloaded and free to move when calibrating the force sensors. The calibration action waits for the completion frame.

```bash
ros2 action send_goal /inspire_hand/clear_error inspire_hand_interfaces/action/HandMaintenance '{}'
ros2 action send_goal /inspire_hand/calibrate inspire_hand_interfaces/action/HandMaintenance '{}'
```
