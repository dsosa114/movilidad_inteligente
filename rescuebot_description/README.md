# rescuebot_description

ROS 2 description package for the RescueBot mobile base.

This package contains the robot model (`URDF`), meshes, RViz configuration, and a launch file to visualize the robot with `robot_state_publisher`, `joint_state_publisher_gui`, and `rviz2`.

## Package Contents

- `urdf/rescuebot_mobile_base.urdf`: RescueBot base model.
- `meshes/*.STL`: CAD meshes referenced by the URDF.
- `rviz/config.rviz`: RViz visualization setup.
- `launch/display.launch.py`: Launches state publisher, joint GUI, and RViz.
- `config/`: Extra config directory (currently available for future use).

## Requirements

Minimum runtime tools used by `display.launch.py`:

- `robot_state_publisher`
- `joint_state_publisher_gui`
- `rviz2`
- `xacro` (used by the launch file to process the model command)

On Ubuntu with ROS 2 Humble, you can install them with:

```bash
sudo apt update
sudo apt install \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-rviz2 \
  ros-humble-xacro
```

If you use another ROS 2 distro, replace `humble` with your distro name.

## Build

From your workspace root:

```bash
cd ~/Workspaces/movilidad_ws
colcon build --packages-select rescuebot_description
source install/setup.bash
```

## Run

Launch the visualization:

```bash
ros2 launch rescuebot_description display.launch.py
```

This starts:

- `robot_state_publisher`
- `joint_state_publisher_gui`
- `rviz2` with `rviz/config.rviz`

## Notes

- The URDF currently references meshes with `$(find rescuebot_description)` style paths.
- The launch file currently loads `rescuebot_mobile_base.urdf` through a `xacro` command. If needed, this can be switched to direct file loading for a pure URDF workflow.

## License

Apache-2.0 (see `LICENSE`).
