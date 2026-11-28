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

Add `use_rviz:=true` to view the selected hand and its measured motion in RViz.
The standalone model comes from [inspire_hand_description](inspire_hand_description/NOTICE.md).
TF topics are scoped to the hand namespace so that two hands can use the same frame names.

Use `side:=left` for the left-hand kinematic frames. Both sides use the same six joint names. To run two standalone hands, launch each with its own `device` and `namespace` (default `inspire_hand`). The driver clears errors once on activation and starts from measured joint positions. To leave the hand in its last pose on shutdown, set `close_hand_on_deactivate:=false`.

On graceful shutdown (for example Ctrl-C), the driver first commands all six axes
open to help prevent the fingers from colliding as they close. After the hand
acknowledges the open command, the driver waits 0.8 s before sending the closing pose.
If the open command fails or calibration has not finished, the driver skips closing. It does not wait for
the closing motion to finish. SIGKILL and power loss cannot run this sequence.
Keep fingers clear; opening can release a held object.

An unplugged USB serial device does not stop the controllers or the other hand.
The disconnected hand ignores commands and rejects maintenance goals. Its joint
states retain the last measurement (the open pose if it was absent at startup).
After reconnecting USB, stop any active trajectory and reactivate the hand hardware
or relaunch to resume communication.

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

## Fingertip forces (direct serial only)

Use the hand's `set_force` action to change the closing force thresholds without
sending a trajectory. The six values are in newtons (0 to 9.80665 N), in order:
pinky, ring, middle, index, thumb bend, thumb rotation. A successful result means
the hand acknowledged the write, not that it reached the specified force.

```bash
hand=/inspire_hand  # On G1: /left_hand or /right_hand
ros2 action send_goal "$hand/set_force" inspire_hand_interfaces/action/SetForce \
  '{thresholds: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]}'
ros2 topic echo "$hand/index/wrench" --qos-reliability best_effort
```

Each successful serial sampling cycle publishes six `geometry_msgs/msg/WrenchStamped`
samples with the same timestamp on `$hand/{pinky,ring,middle,index,thumb,thumb_rotation}/wrench`.
Each finger has a fixed `<finger>_pad` frame on the contact side of its distal mesh
(`L_` or `R_` prefix on G1). Positive bending force is represented along the
outward pad normal (+Z), including thumb bending on `thumb/wrench`. The separate
`thumb_rotation/wrench` uses the transverse tangent (+X) of the same thumb pad
by convention; it is not a measured direction
or torque. All other components are zero, not independently measured. No stale
wrenches are published while disconnected or waiting for calibration.

The [RH56 manual](https://en.inspire-robots.com/wp-content/uploads/2023/11/INSPIRE-ROBOTS-THE-DEXTEROUS-HAND-RH56-SERIES-USER-MANUAL.pdf)
describes these values as fingertip gripping forces. They are **estimates**, not
calibrated contact forces or guaranteed maximum loads. Contact location and closing
speed matter; actual force can overshoot the threshold. See [Tan et al., RH56DFX
force characterization](https://arxiv.org/abs/2603.08988).

## Hand maintenance

If a joint stops responding to commands, first check the communication light on the affected hand's USB adapter.

1. If the light does not show normal activity, stop the current trajectory, make sure the USB adapter is firmly plugged in, and restart the launch.
2. If the light still does not show normal activity, stop the launch, unplug and reconnect the USB adapter, then start the launch again and recheck the light.

If the light shows normal activity but the joint still does not move, stop the current trajectory and clear the hand's errors. Set `hand` to the affected driver's namespace.

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
