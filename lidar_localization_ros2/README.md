# pcl_localization_ros2
A ROS2 package of 3D LIDAR-based Localization using PCL.

<img src="./images/path.png" width="640px">

Green: path, Red: map  
(the 5x5 grids in size of 50m × 50m)

## Requirements

- [ndt_omp_ros2](https://github.com/rsasaki0109/ndt_omp_ros2.git)

## IO
- input  
/velodyne_points  (sensor_msgs/PointCloud2)  
/map  (sensor_msgs/PointCloud2)  
/initialpose (geometry_msgs/PoseWithCovarianceStamped)(when `set_initial_pose` is false)  
/odom (nav_msgs/Odometry)(optional)   
/imu  (sensor_msgs/Imu)(optional)  

- output  
/pcl_pose (geometry_msgs/PoseWithCovarianceStamped)  
/path (nav_msgs/Path)  
/initial_map (sensor_msgs/PointCloud2)(when `use_pcd_map` is true)  

## params

|Name|Type|Default value|Description|
|---|---|---|---|
|registration_method|string|"GICP"|"NDT" or "GICP" of "NDT_OMP" of "GICP_OMP"|
|score_threshold|double|2.0|registration score threshold|
|ndt_resolution|double|2.0|resolution size of voxels[m]|
|ndt_step_size|double|0.1|step_size maximum step length[m]|
|ndt_num_threads|int|0|threads using NDT_OMP(if `0` is set, maximum alloawble threads are used.)|
|transform_epsilon|double|0.01|transform epsilon to stop iteration in registration|
|voxel_leaf_size|double|0.2|down sample size of input cloud[m]|
|scan_max_range|double|100.0|max range of input cloud[m]|
|scan_min_range|double|1.0|min range of input cloud[m]|
|scan_periad|double|0.1|scan period of input cloud[sec]|
|use_pcd_map|bool|false|whether pcd_map is used or not|
|map_path|string|"/map/map.pcd"|pcd_map file path|
|set_initial_pose|bool|false|whether or not to set the default value in the param file|
|initial_pose_x|double|0.0|x-coordinate of the initial pose value[m]|
|initial_pose_y|double|0.0|y-coordinate of the initial pose value[m]|
|initial_pose_z|double|0.0|z-coordinate of the initial pose value[m]|
|initial_pose_qx|double|0.0|Quaternion x of the initial pose value|
|initial_pose_qy|double|0.0|Quaternion y of the initial pose value|
|initial_pose_qz|double|0.0|Quaternion z of the initial pose value|
|initial_pose_qw|double|1.0|Quaternion w of the initial pose value|
|use_odom|bool|false|whether odom is used or not for initial attitude in point cloud registration|
|use_imu|bool|false|whether 9-axis imu is used or not for point cloud distortion correction|
|enable_debug|bool|false|whether debug is done or not|

## demo

demo data(ROS1) by Tier IV  
https://data.tier4.jp/rosbag_details/?id=212  
To use ros1 rosbag , use [rosbags](https://pypi.org/project/rosbags/).  
The Velodyne VLP-16 was used in this data.

Before running, put `bin_tc-2017-10-15-ndmap.pcd` into your `map` directory and  
edit the `map_path` parameter of `localization.yaml` in the `param` directory accordingly.
```
rviz2 -d src/pcl_localization_ros2/rviz/localization.rviz
ros2 launch pcl_localization_ros2 pcl_localization.launch.py
ros2 bag play tc_2017-10-15-15-34-02_free_download/
```

<img src="./images/path.png" width="640px">

Green: path, Red: map  
(the 5x5 grids in size of 50m × 50m)

## Changes from Original pcl_localization_ros2

### New Features
- **Multi-threading Support**: Added NDT_OMP and GICP_OMP registration methods for improved performance
- **Frame Transformation**: Added `convert_pose` functionality for transforming poses between different coordinate frames
- **Registration Quality Control**: Added `score_threshold` parameter to validate registration quality
- **TF Broadcasting Control**: Added `publish_tf` option to control transform publishing

### Message Type Changes
- **Input**: `geometry_msgs/PoseStamped` → `geometry_msgs/PoseWithCovarianceStamped` for `/initialpose`
- **Output**: `geometry_msgs/PoseStamped` → `geometry_msgs/PoseWithCovarianceStamped` for `/pcl_pose`

### Dependencies Added
- **ndt_omp_ros2**: Required for NDT_OMP functionality
- **OpenMP**: Added for multi-threading support
- **tf2_sensor_msgs**: Added for sensor frame transformations

### Parameter Optimizations
- **NDT Resolution**: Improved from 3.0m to 0.5m for higher precision
- **Voxel Leaf Size**: Reduced from 1.0m to 0.1m for finer detail processing
- **Scan Max Range**: Extended from 100.0m to 120.0m
- **Score Threshold**: Set to 1.0 for better registration validation

### Launch Configuration Changes
- **Topic Remapping**: Changed from `/points_raw` to `/velodyne_points`
- **Frame Configuration**: Updated frame relationships from `odom` to `base_link`

### New Parameters Added
|Name|Type|Default|Description|
|---|---|---|---|
|publish_tf|bool|false|Whether to publish TF transforms|
|convert_pose|bool|true|Whether to convert poses between frames|
|score_threshold|double|1.0|Registration score threshold for quality validation|
|ndt_num_threads|int|4|Number of threads for NDT_OMP processing|

### RViz Configuration Updates
- Added `PoseWithCovariance` visualization for pose uncertainty
- Changed camera view from Orbit to TopDownOrtho
- Updated topic configurations for new message types
- Enhanced visualization settings for better debugging
