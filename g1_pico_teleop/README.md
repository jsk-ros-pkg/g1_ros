# PICO optical-hand teleoperation

This package maps PICO optical hand tracking to the existing left/right Inspire
ros2_control controllers. Body control stays with the official PICO manager.
See [systemd setup](../README.md#pico-hand-teleoperation) for robot preparation and startup.

With G1 bringup and the patched manager already running:

```bash
ros2 launch g1_pico_teleop pico_hands.launch.py hand_zmq_endpoint:=tcp://10.42.0.1:5560
```

In another sourced ROS shell on the same domain, enable or disable hand following:

```bash
ros2 service call /pico_hand_adapter/enable std_srvs/srv/SetBool '{data: true}'
ros2 service call /pico_hand_adapter/enable std_srvs/srv/SetBool '{data: false}'
```

G1 bringup activates both hand controllers. The adapter starts with following
OFF and requires paired tracking and fresh hand joint states before sending
commands. Source timeout or restart requires enabling following again.
