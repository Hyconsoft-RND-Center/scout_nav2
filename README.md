# scout_nav2

Navigation running package for SCOUT 2.0

Using ROS2 Humble
used ROS2 Package: Navigation 2, lidar_ocalization_ros2, lidarslam_ros2, patchwork++, ultralytic_ros

## Setup

Get packages from git

```bash
mkdir ~/scout_nav2/src
cd ~/scout_nav2/src/

git clone https://github.com/Hyconsoft-RND-Center/scout_nav2 --branch scout_ready
vcs import . < scout_nav2/deps.repo
```

Install dependency packages from rosdep

```bash
cd ~/scout_nav2
rosdep install --from-paths src --ignore-src -r -y
```

Build Package

```bash
colcon build
```  

## Run

Launch SCOUT 2.0 robot package

Launch SCOUT 2.0

```bash
source ~/colcon_ws/install/setup.bash
ros2 launch roas_brigup bringup.launch.py
```

Launch Ouster LiDAR

```bash
source ~/sensor_ws/install/setup.bash
ros2 launch ouster_ros driver.launch.py
```

Launch
