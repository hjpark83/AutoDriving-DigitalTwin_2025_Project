# DCAS Perception Simulation (ROS1) — Usage Guide

This package provides a minimal-yet-complete perception simulation framework for a 4-day DCAS course. It includes:
- Environment ground-truth publishers (lanes, traffic lights, guardrails, buildings, signs, vehicles)
- Simple sensor emulators (motion encoders, GNSS, IMU, LiDAR, Radar, Camera)
- Perception stub nodes (pass-through of GT to mimic detection/tracking blocks)
- RViz visualization via `visualization_msgs/MarkerArray`

The intent is to wire messages and topics end-to-end so learners can replace stubs with real logic.

## 1) Requirements
- ROS1 Noetic (Ubuntu 20.04) or compatible setup
- Python 3 (installed with Noetic)

## 2) Build & Source
```bash
cd /home/ailab/git/NGV_DCAS_SW
catkin_make
source devel/setup.bash
```

## 3) Launch
- Bring up all environment/sensor nodes:
```bash
roslaunch dcas_perception_sim bringup.launch
```
- Bring up with RViz visualization (recommended):
```bash
roslaunch dcas_perception_sim rviz.launch
```
Environment and vehicle nodes publish their own MarkerArray topics; RViz opens with a preconfigured view (`rviz/dcas_perception.rviz`).

## 4) Topics Overview
- Environment (GT)
  - `/env/lanes` — `dcas_msgs/LaneArray`
  - `/env/traffic_lights` — `dcas_msgs/TrafficLightArray`
  - `/env/guardrails`, `/env/buildings`, `/env/traffic_signs`, `/env/surrounding_vehicles` — `dcas_msgs/ObjectArray`
- Sensors
  - `/sensors/motion/steering_wheel_angle` — `std_msgs/Float32`
  - `/sensors/motion/wheel_speed_pulses` — `std_msgs/Int32MultiArray`
  - `/sensors/gnss/fix` — `sensor_msgs/NavSatFix`, `/sensors/gnss/odom` — `nav_msgs/Odometry`
  - `/sensors/imu/data` — `sensor_msgs/Imu`
  - `/sensors/lidar/points` — `sensor_msgs/PointCloud2`
  - `/sensors/radar/points` — `sensor_msgs/PointCloud2`
  - `/sensors/camera/image_raw` — `sensor_msgs/Image`
- Perception outputs
  - `/perception/lanes` — `dcas_msgs/LaneArray`
  - `/perception/traffic_lights` — `dcas_msgs/TrafficLightArray`
  - `/perception/traffic_signs`, `/perception/guardrails`, `/perception/buildings`, `/perception/vehicles` — `dcas_msgs/ObjectArray`
- Visualization
  - `/viz/markers` — `visualization_msgs/MarkerArray`

## 5) Nodes (by group)
- Environment
  - `env_lane_node.py`, `env_traffic_light_node.py`, `env_guardrail_node.py`, `env_building_node.py`, `env_sign_node.py`, `env_surrounding_vehicle_node.py`
- Sensors
  - `sensor_motion_node.py`, `sensor_gnss_node.py`, `sensor_imu_node.py`, `sensor_lidar_node.py`, `sensor_radar_node.py`, `sensor_camera_node.py`
- Perception stubs: removed to simplify the base package (use your own perception nodes if needed)
- Visualization
  - Marker topics are published by environment nodes and vehicle node

## 6) RViz
- Fixed Frame: `map`
- Displays:
  - `MarkerArray` on `/viz/markers`
  - (Optional) Add `PointCloud2` for `/sensors/lidar/points`
  - (Optional) Add `Image` for `/sensors/camera/image_raw`

## 7) Recording & Debugging
- Record a quick bag:
```bash
rosbag record -O dcas_perception.bag /env/lanes /perception/lanes /viz/markers
```
- Inspect graph/topics:
```bash
rqt_graph
rostopic list
rostopic echo /perception/lanes
```

## 8) Extending
- Replace perception stubs with your algorithms. Keep I/O topics unchanged to maintain launch compatibility.
- Add parameters (e.g., noise levels, spawn rates) using ROS params to control emulators.
- Add new environment objects by extending `dcas_msgs/Object` or creating new messages in `dcas_msgs`.

## 9) Package Layout
```
<repo-root>/src/
  dcas_msgs/                # Custom messages used across the framework
  dcas_perception_sim/
    launch/
      bringup.launch        # Starts env + sensors + vehicle
      rviz.launch           # Starts bringup + RViz + markers
    rviz/
      dcas_perception.rviz  # Preconfigured RViz layout
    scripts/                # Nodes (env, sensors, vehicle, viz)
```

## 10) Known Limits
- Environment publishers are simplified and deterministic.
- Perception nodes are pass-through stubs; no detection/tracking yet.
- No TF tree beyond `map` is enforced.

## 11) License
Apache-2.0
