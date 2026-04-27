# qcar_gazebo

ROS 2 package for simulating a QCar in Gazebo Sim (Ignition/GZ), including:

- Gazebo worlds and models
- `ros_gz_bridge` topic bridge configuration
- A kinematic `vehicle_controller` node (Ackermann steering + rear wheel velocities)
- A `joystick_controller` node for manual driving
- Launch files for single and multi-QCar simulation

## Package Structure

- `launch/`
  - `gz_sim.launch.py`: Single QCar simulation + bridge + controllers + EKF + RViz.
  - `gz_multi_qcar.launch.py`: Multi-QCar simulation with per-vehicle namespaces (Still experimental, needs adjustments).
  - `joystick.launch.py`: Joystick teleoperation pipeline (`joy_node` + `joystick_controller`).
  - `qcar2_mod_virtual_launch.py`: Virtual QCar2 sensor/hardware nodes from `qcar2_nodes`.
- `config/`
  - `ego_params.yaml`: Vehicle geometry and limits.
  - `gz_bridge.yaml`: ROS <-> Gazebo topic bridge mappings.
  - `ekf.yaml`: `robot_localization` EKF setup.
- `src/`
  - `vehicle_controller.cpp`
  - `joystick_controller.cpp`
- `worlds/`
  - `test_world.sdf`
  - `stop_test_world.sdf`
  - `stop_sign_cross_world.sdf`
  - `assessment_world.sdf`
- `models/`: Custom Gazebo assets.

## Prerequisites

This package is intended for ROS 2 + Gazebo Sim and expects companion packages in the same workspace (for example `qcar_description`, and optionally `qcar2_nodes`).

Typical dependencies used by launch files/nodes:

- ROS 2 core and build tools: `ament_cmake`, `rclcpp`
- Message packages: `std_msgs`, `sensor_msgs`, `geometry_msgs`, `nav_msgs`, `rosgraph_msgs`
- Simulation and bridge: `ros_gz_sim`, `ros_gz_bridge`
- Robot stack: `robot_state_publisher`, `xacro`, `controller_manager`, `robot_localization`, `rviz2`, `joy`

## Build

From workspace root (`movilidad_ws`):

```bash
colcon build --packages-select qcar_gazebo
source install/setup.bash
```

## Quick Start

### 1) Single QCar simulation

```bash
ros2 launch qcar_gazebo gz_sim.launch.py
```

Load a different world:

```bash
ros2 launch qcar_gazebo gz_sim.launch.py world:='assessment_world.sdf'
```

### 2) Joystick teleoperation

Run in another terminal (after sourcing workspace):

```bash
ros2 launch qcar_gazebo joystick.launch.py
```

`joystick_controller` publishes user command messages to `/qcar_sim/user_command`.

### 3) Multi-QCar simulation (Experimental, may fail)

Spawn multiple vehicles with auto-generated poses:

```bash
ros2 launch qcar_gazebo gz_multi_qcar.launch.py num_qcars:=3 base_name:=qcar
```

Spawn with explicit poses (`x,y,z,yaw;x,y,z,yaw;...`):

```bash
ros2 launch qcar_gazebo gz_multi_qcar.launch.py \
  poses:='0.0,0.0,0.0025,0.0;1.5,0.0,0.0025,0.0;3.0,0.0,0.0025,0.0'
```

Disable selected components (optional):

```bash
ros2 launch qcar_gazebo gz_multi_qcar.launch.py \
  spawn_bridge:=false spawn_vehicle_controller:=true spawn_ros2_controllers:=true
```

## Main Nodes

### `vehicle_controller`

- Subscribes: `/qcar_sim/user_command` (`geometry_msgs/msg/Vector3Stamped`)
  - `vector.x`: linear velocity command
  - `vector.y`: steering angle command
- Publishes:
  - `/forward_position_controller/commands` (`std_msgs/msg/Float64MultiArray`)
  - `/forward_velocity_controller/commands` (`std_msgs/msg/Float64MultiArray`)
- Behavior:
  - Clamps steering and speed by `max_steering_angle` and `max_velocity`
  - Computes Ackermann front steering angles
  - Computes differential rear wheel velocities
  - Stops vehicle if command timeout expires

### `joystick_controller`

- Subscribes: `joy` (`sensor_msgs/msg/Joy`)
- Publishes: `/qcar_sim/user_command` (`geometry_msgs/msg/Vector3Stamped`)
- Uses `max_steering_angle`, `max_velocity`, and `publish_topic` from `ego_params.yaml`

## Useful Topics

From `config/gz_bridge.yaml` and launch setup:

- `clock`
- `joint_states`
- `/qcar_sim/odom`
- `/qcar_sim/imu`
- `/qcar_sim/scan`
- `/qcar_sim/rgb/image_raw`
- `/qcar_sim/rgb/camera_info`
- `/qcar_sim/csi_front/image_raw`
- `/qcar_sim/csi_front/camera_info`
- `/qcar_sim/csi_back/image_raw`
- `/qcar_sim/csi_back/camera_info`
- `/qcar_sim/csi_left/image_raw`
- `/qcar_sim/csi_left/camera_info`
- `/qcar_sim/csi_right/image_raw`
- `/qcar_sim/csi_right/camera_info`

In multi-QCar mode, these are remapped under each namespace (for example `/qcar_0/...`, `/qcar_1/...`).

## Key Launch Arguments

### `gz_sim.launch.py`

- `world` (default: `test_world.sdf`)
- `use_sim_time` (default: `True`)
- `is_ign` (default: `false`)

### `gz_multi_qcar.launch.py`

- `world`, `is_ign`
- `base_name`, `num_qcars`
- `poses` (overrides auto-generation)
- `start_x`, `start_y`, `start_z`, `start_yaw`
- `x_spacing`, `y_spacing`
- `spawn_bridge`, `spawn_vehicle_controller`, `spawn_ros2_controllers`

## Sanity Checks

List active nodes:

```bash
ros2 node list
```

Inspect the user command topic:

```bash
ros2 topic echo /qcar_sim/user_command
```

Inspect odometry:

```bash
ros2 topic echo /qcar_sim/odom
```

## Troubleshooting

- If models/world resources are missing in Gazebo, ensure `GZ_SIM_RESOURCE_PATH` is set by launching via package launch files.
- If robot does not move, verify controllers are loaded:

```bash
ros2 control list_controllers
```

- If no joystick input arrives, verify joystick device and `joy` topic:

```bash
ros2 topic echo /joy
```

- If transforms/odometry look inconsistent, confirm `use_sim_time` and EKF settings in `config/ekf.yaml`.

## License

`Apache-2.0` (as declared in `package.xml`).
