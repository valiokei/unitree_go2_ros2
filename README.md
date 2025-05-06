# Unitree Go2 ROS2

![Unitree Go2](https://oss-global-cdn.unitree.com/static/c487f93e06954100a44fac4442b94d94_288x238.png)

This package contains the configuration and integration of the Unitree Go2 robot with the CHAMP controller framework for ROS 2 Jazzy. It includes custom development of configuration packages and upgrades to the robot description model specifically adapted for ROS 2.

## About Unitree Go2

The Go2 is a quadrupedal robot developed by Unitree Robotics, designed for both research and commercial applications. It features powerful actuators, advanced sensor integration capabilities, and a robust mechanical design ideal for various terrain navigation.

## About CHAMP Controller

CHAMP (Coupled Hybrid Automata for Mobile Platforms) is an open-source development framework specifically designed for building new quadrupedal robots and developing novel control algorithms. The control framework is based on hierarchical control principles that combine pattern modulation and impedance control techniques.

![CHAMP Controller](https://raw.githubusercontent.com/chvmp/champ/master/docs/images/robots.gif)

## Features

- ✅ Complete ROS 2 Jazzy integration
- ✅ URDF model adapted to ROS 2 control framework
- ✅ Gazebo Harmonic simulation support
- ✅ Teleoperation using keyboard
- ✅ RVIZ visualization
- ✅ Integrated gait control and configuration
- ❌ Simulated sensors:(in progress)
  - ❌ IMU
  - ❌ 2D LiDAR (Hokuyo)
  - ❌ 3D LiDAR (Velodyne)
- ❌ Full SLAM functionality (Coming soon)
- ❌ Navigation 2 integration (Coming soon)

## System Requirements

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Sim Harmonic

## Installation

### 1. Install ROS 2 Dependencies

```bash
sudo apt update
sudo apt install ros-jazzy-gazebo-ros2-control
sudo apt install ros-jazzy-xacro
sudo apt install ros-jazzy-robot-localization
sudo apt install ros-jazzy-ros2-controllers
sudo apt install ros-jazzy-ros2-control
sudo apt install ros-jazzy-velodyne
sudo apt install ros-jazzy-velodyne-gazebo-plugins
sudo apt install ros-jazzy-velodyne-description
```

<!-- ### 2. Clone and Install CHAMP and Related Packages

First, ensure you have the core CHAMP packages installed:

```bash
cd ~/ros2_ws/src
git clone https://github.com/chvmp/champ
``` -->

### 3. Clone and Install CHAMP Controller and Go2 simulation robot Packages

```bash
cd ~/ros2_ws/src
git clone https://github.com/anujjain-dev/unitree-go2-ros2.git
```

### 4. Install Dependencies

```bash
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 5. Build the Workspace

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Usage

### Gazebo Simulation

Launch the Gazebo simulation:

```bash
ros2 launch unitree_go2_sim unitree_go2_launch.py
```

![unitree_go2](<unitree_go2_sim.png>)

[Watch on YouTube](https://youtu.be/NUu7TaZhaQM)

### RVIZ Visualization

Launch Gazebo with RVIZ:

```bash
ros2 launch unitree_go2_sim unitree_go2_launch.py rviz:=true
```

![alt text](<unitree_go2_vis.png>)

### Teleoperation

Control the robot using keyboard:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

<!-- ### Using the Velodyne Configuration

Launch the Gazebo simulation with Velodyne LiDAR:

```bash
ros2 launch go2_config gazebo_velodyne.launch.py
```

Launch with RVIZ (remember to set PointCloud2 topic to `/velodyne_points` in RVIZ):

```bash
ros2 launch go2_config gazebo_velodyne.launch.py rviz:=true
```

### Using 2D LiDAR Instead of 3D Velodyne

To use the Hokuyo 2D LiDAR instead of the 3D Velodyne:

1. Edit the `robot_VLP.xacro` file located in `go2_description/xacro/` folder
2. Comment out: `<xacro:include filename="$(find go2_description)/xacro/velodyne.xacro"/>`
3. Uncomment: `<xacro:include filename="$(find go2_description)/xacro/laser.xacro"/>`
4. Rebuild your workspace -->

## Tuning Gait Parameters

The gait configuration for the robot is found in `unitree_go2_sim/config/gait/gait.yaml`. You can modify the following parameters:

| Parameter | Description |
|-----------|-------------|
| Knee Orientation | How the knees should be bent (.>> .>< .<< .<>) |
| Max Linear Velocity X | Maximum forward/reverse speed (m/s) |
| Max Linear Velocity Y | Maximum sideways speed (m/s) |
| Max Angular Velocity Z | Maximum rotational speed (rad/s) |
| Stance Duration | How long each leg spends on the ground while walking |
| Leg Swing Height | Trajectory height during swing phase (m) |
| Leg Stance Height | Trajectory depth during stance phase (m) |
| Robot Walking Height | Distance from hip to ground while walking (m) |
| CoM X Translation | Offset to compensate for weight distribution |
| Odometry Scaler | Multiplier to calculated velocities for dead reckoning |

## Project Structure

- `champ_base/`: Core controllers and state estimation for CHAMP
- `unitree_go2_description/`: URDF models, meshes, and world files
- `unitree_go2_sim/`: Simulation launch files and configuration

<!-- ## Roadmap

- [ ] Implement SLAM functionality
- [ ] Add Navigation 2 integration
- [ ] Support for real hardware connection
- [ ] Improved terrain handling -->

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'feat: Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## Acknowledgements

This project builds upon and incorporates work from the following projects:

* [Unitree Robotics](https://github.com/unitreerobotics/unitree_ros) - For the Go2 robot description (URDF model)
* [CHAMP](https://github.com/chvmp/champ) - For the quadruped controller framework
* [CHAMP Robots](https://github.com/chvmp/robots) - For robot configurations and setup examples
* [unitree-go2-ros2](https://github.com/anujjain-dev/unitree-go2-ros2) - ROS 2 package with gazebo classic

## License

This project is licensed under the BSD 3-Clause License - see the LICENSE file for details.