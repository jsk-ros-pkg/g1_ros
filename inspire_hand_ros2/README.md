# Inspire Hand ROS 2

ROS 2 Jazzy driver for one physical Inspire RH56DFX hand per serial port (default 115200 baud, device ID 1). The driver exposes six actuated joints in radians through ros2_control.

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

## Hand maintenance

If a joint stops moving in response to commands, stop the current trajectory and clear the hand's errors. Set `hand` to the affected driver's namespace.

```bash
hand=/inspire_hand
ros2 action send_goal "$hand/clear_error" inspire_hand_interfaces/action/HandMaintenance '{}'
```

Retry the motion. If the joint still does not respond, stop trajectories again and calibrate the force sensors. The fingers move during calibration. Remove any grasped object and keep all fingers unloaded and clear of contact throughout the operation.

```bash
ros2 action send_goal "$hand/calibrate" inspire_hand_interfaces/action/HandMaintenance '{}'
```

Wait for the action to report success before retrying the motion.

## Serial settings

Set `baudrate` and `hand_id` on launch to match the connected hand. Supported rates are 115200, 57600 and 19200 baud, and IDs range from 1 to 254.

To change the hand itself, stop the driver or DDS bridge and connect one hand to the port. Pass its **current** connection settings and either or both new values. With no new values the command only reads the settings.

```bash
ros2 run inspire_hand_driver configure_rh56dfx --device /dev/ttyUSB0 \
  --baudrate 115200 --hand-id 1 --new-baudrate 57600 --new-hand-id 2
```

The new ID takes effect immediately. The tool saves the hand's current parameters to flash and waits for confirmation. Power-cycle the hand after a baud rate change, then check and launch with the new settings:

```bash
ros2 run inspire_hand_driver configure_rh56dfx --device /dev/ttyUSB0 --baudrate 57600 --hand-id 2
ros2 launch inspire_hand_driver rh56dfx.launch.py device:=/dev/ttyUSB0 baudrate:=57600 hand_id:=2
```

See the [RH56 manual, sections 2.4 and 3](https://inspire-robots.com/d/file/p/2023/11-17/%E5%9B%A0%E6%97%B6%E6%9C%BA%E5%99%A8%E4%BA%BA%E4%BB%BF%E4%BA%BA%E4%BA%94%E6%8C%87%E7%81%B5%E5%B7%A7%E6%89%8B--RH56%E7%94%A8%E6%88%B7%E6%89%8B%E5%86%8CV1.09cn%20.pdf) for the serial register definitions.
