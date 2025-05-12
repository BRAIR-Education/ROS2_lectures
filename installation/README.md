# Installation & Setup

## 1) Install
To install ROS2, we can simply follow this official [guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).

### 1.1) Source ROS2 in the `.bashrc` file
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

## 2) Workspace Setup
Create workspace directory typing
```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
```
To init the workspace, we need to do a initial building with `colcon`. To build, we must be in the `~/ros2_ws` directory.
```bash
cd .. && colcon build
```

### 2.1) Source the workspace in the `.bashrc` file
```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```