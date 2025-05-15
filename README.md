# scout_nav2
navigation for SCOUT 2.0



# Install
<h3>Install</h3>
</code>
git clone --branch test https://github.com/Hyconsoft-RND-Center/scout_nav2
</code>

<h3>Installing Dependancy Packages</h3>

<h4>underlay ROS2 package: rosdep</h4>

<code>
sudo apt install python3-rosdep # install rosdep if you don't have one
sudo rosdep init  # rosdep need initialize only once after installation
rosdep update     # after rosdep init, update rosdep update

rosdep install --from-paths src --ignore-src -r -y   # install packages with rosdep
</code>


<h4>overlay packages: vcs-tools</h4>

<code>
cd src/
sudo apt install python3-vcs-tools # install vcs-tools if you don't have one
vcs import . < deps.repos
</code>

<h3>Build Packages</h3>
<code>
cd ..
colcon build
<code>

<h3>Launch Packages<h3>
<code>
source install/setup.bash

ros2 launch scout_nav2_bringup scout_bringup.launch.py
</code>
