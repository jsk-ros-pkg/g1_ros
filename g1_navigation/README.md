# g1_navigation

Nav2 localization + navigation for the Unitree G1 against a **2D occupancy grid**
map, driving the robot through `/cmd_vel` (the gear_sonic locomotion bridge).

## TF / architecture

```
map ──AMCL──> odom ──FAST-LIO2──> base_link ──static──> pelvis ──> (robot TF: torso, livox_frame, legs…)
 (2D scan match)     (LIO odometry)            (base_link->pelvis)
/livox/lidar (PointCloud2, unitree) ─pointcloud_to_laserscan─> /scan  (AMCL + costmap obstacle layer)
/livox/lidar_custom (CustomMsg, livox_driver) ──> FAST-LIO2
nav2 (costmaps[static=2D map, obstacle=/scan] + planner + RPP controller + bt) ──> /cmd_vel ──> gear_sonic
```

- **Odometry**: FAST-LIO2 (Livox mid360 + IMU) publishes `odom -> base_link`.
- **Global localization**: AMCL matches `/scan` against the 2D map, publishes `map -> odom`.
- **Map**: not bundled (site-specific). Pass a map_server-format 2D occupancy grid
  via `map:=/path/to/map.yaml` (required).

## Assumptions to check

- **Torso/waist held fixed** during navigation (mid360 is torso-mounted). Set the
  static `base_link -> pelvis` from the neutral pose:
  `ros2 run tf2_ros tf2_echo imu_in_torso pelvis`, then pass
  `base_to_pelvis_{x,y,z,yaw,pitch,roll}:=…` to `fast_lio.launch.py`.
- `pelvis` has roll/pitch/height while walking; nav2 uses `base_link` (x,y,yaw). Add a
  ground-projected `base_footprint` if 2D accuracy suffers.
- If the 2D scan projection is too noisy, switch to 3D `.pcd` localization (NDT/GLIM).

## Run

```bash
ros2 launch g1_navigation g1_navigation.launch.py map:=/path/to/map.yaml
ros2 launch g1_navigation g1_navigation.launch.py map:=/path/to/map.yaml use_fast_lio:=false  # external odom
```

Send goals from RViz2 (Nav2 panel) or the `navigate_to_pose` action.

## Livox CustomMsg (FAST-LIO input)

FAST-LIO uses `livox_ros_driver2/CustomMsg` (`lidar_type=1`), whose per-point
timestamps beat PointCloud2 for odometry. Publish it on dedicated topics
(`/livox/lidar_custom`, `/livox/imu_custom`) so it coexists with unitree's
PointCloud2 `/livox/lidar` (which still feeds the scan bridge):

```bash
ros2 launch g1_navigation livox_driver.launch.py   # edit config/mid360.json IPs first
```

## Deps

`nav2_*`, `pointcloud_to_laserscan` come from apt (rosdep). `fast_lio`,
`livox_ros_driver2` (tu-darmstadt fork) and `livox_sdk2` are source deps — see
the workspace `jazzy.repos.yaml` (and `.github/upstream.jazzy.repos` for CI).
