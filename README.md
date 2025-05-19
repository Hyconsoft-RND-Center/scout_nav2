# scout_nav2

navigation for SCOUT 2.0

## Install

`
git clone --branch test https://github.com/Hyconsoft-RND-Center/scout_nav2
`

## Installing Dependancy Packages

Underlay ROS2 package: rosdep

`
sudo apt install python3-rosdep # install rosdep if you don't have one
`
sudo rosdep init  # rosdep need initialize only once after installation
`
rosdep update     # after rosdep init, update rosdep update
`
install packages with rosdep
`
rosdep install --from-paths src --ignore-src -r -y
`

Overlay ROS2 packages: vcs-tools

`
cd src/
`
sudo apt install python3-vcs-tools # install vcs-tools if you don't have one
`
vcs import . < ../deps.repos
`

## Build Packages

`
cd ..
`
colcon build
`

## Launch Packages

`
source install/setup.bash
`
ros2 launch scout_nav2_bringup scout_bringup.launch.py
`
