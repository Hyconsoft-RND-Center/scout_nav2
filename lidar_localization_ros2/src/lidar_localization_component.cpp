#include <lidar_localization/lidar_localization_component.hpp>
PCLLocalization::PCLLocalization(const rclcpp::NodeOptions &options)
    : rclcpp_lifecycle::LifecycleNode("lidar_localization", options),
      clock_(RCL_ROS_TIME),
      tfbuffer_(std::make_shared<rclcpp::Clock>(clock_)),
      tflistener_(tfbuffer_),
      broadcaster_(this)
{
  declare_parameter("publish_tf", false);
  declare_parameter("global_frame_id", "map");
  declare_parameter("odom_frame_id", "odom");
  declare_parameter("base_frame_id", "base_link");
  declare_parameter("registration_method", "NDT");
  declare_parameter("score_threshold", 2.0);
  declare_parameter("ndt_resolution", 1.0);
  declare_parameter("ndt_step_size", 0.1);
  declare_parameter("ndt_max_iterations", 35);
  declare_parameter("ndt_num_threads", 4);
  declare_parameter("transform_epsilon", 0.01);
  declare_parameter("voxel_leaf_size", 0.2);
  declare_parameter("scan_max_range", 100.0);
  declare_parameter("scan_min_range", 1.0);
  declare_parameter("scan_period", 0.1);
  declare_parameter("use_pcd_map", false);
  declare_parameter("map_path", "/map/map.pcd");
  declare_parameter("map_offset_x", 0.0);
  declare_parameter("map_offset_y", 0.0);
  declare_parameter("map_offset_z", 0.0);
  declare_parameter("set_initial_pose", false);
  declare_parameter("initial_pose_x", 0.0);
  declare_parameter("initial_pose_y", 0.0);
  declare_parameter("initial_pose_z", 0.0);
  declare_parameter("initial_pose_qx", 0.0);
  declare_parameter("initial_pose_qy", 0.0);
  declare_parameter("initial_pose_qz", 0.0);
  declare_parameter("initial_pose_qw", 1.0);
  declare_parameter("use_odom", false);
  declare_parameter("use_imu", false);
  declare_parameter("enable_debug", false);
}

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturn PCLLocalization::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  initializeParameters();
  initializePubSub();
  initializeRegistration();

  path_ptr_ = std::make_shared<nav_msgs::msg::Path>();
  path_ptr_->header.frame_id = global_frame_id_;
  path_ptr_unoffset_ = std::make_shared<nav_msgs::msg::Path>();
  path_ptr_unoffset_->header.frame_id = global_frame_id_;

  RCLCPP_INFO(get_logger(), "Configuring end");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating");

  pose_pub_->on_activate();
  path_pub_->on_activate();
  odom_pub_->on_activate();
  initial_map_pub_->on_activate();

  if (set_initial_pose_)
  {
    auto msg = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();

    msg->header.stamp = now();
    msg->header.frame_id = global_frame_id_;
    msg->pose.pose.position.x = initial_pose_x_;
    msg->pose.pose.position.y = initial_pose_y_;
    msg->pose.pose.position.z = initial_pose_z_;
    msg->pose.pose.orientation.x = initial_pose_qx_;
    msg->pose.pose.orientation.y = initial_pose_qy_;
    msg->pose.pose.orientation.z = initial_pose_qz_;
    msg->pose.pose.orientation.w = initial_pose_qw_;

    geometry_msgs::msg::PoseStamped::SharedPtr pose_stamped(new geometry_msgs::msg::PoseStamped);
    pose_stamped->header.stamp = msg->header.stamp;
    pose_stamped->header.frame_id = global_frame_id_;
    pose_stamped->pose = msg->pose.pose;
    path_ptr_->poses.push_back(*pose_stamped);
    path_ptr_unoffset_ = path_ptr_;

    initialPoseReceived(msg);
  }

  if (use_pcd_map_)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile(map_path_, *map_cloud_ptr);
    RCLCPP_INFO(get_logger(), "Map Size %ld", map_cloud_ptr->size());

    // Offset the map
    // apply offset to the map
    for (auto &point : map_cloud_ptr->points)
    {
      point.x += map_offset_x_;
      point.y += map_offset_y_;
      point.z += map_offset_z_;
    }

    sensor_msgs::msg::PointCloud2::SharedPtr map_msg_ptr(new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(*map_cloud_ptr, *map_msg_ptr);
    map_msg_ptr->header.frame_id = global_frame_id_;
    initial_map_pub_->publish(*map_msg_ptr);
    RCLCPP_INFO(get_logger(), "Initial Map Published");

    if (registration_method_ == "GICP" || registration_method_ == "GICP_OMP")
    {
      pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
      voxel_grid_filter_.setInputCloud(map_cloud_ptr);
      voxel_grid_filter_.filter(*filtered_cloud_ptr);
      registration_->setInputTarget(filtered_cloud_ptr);
    }
    else
    {
      registration_->setInputTarget(map_cloud_ptr);
    }

    map_recieved_ = true;
  }

  RCLCPP_INFO(get_logger(), "Activating end");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  pose_pub_->on_deactivate();
  path_pub_->on_deactivate();
  odom_pub_->on_deactivate();
  initial_map_pub_->on_deactivate();

  RCLCPP_INFO(get_logger(), "Deactivating end");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning Up");
  initial_pose_sub_.reset();
  initial_map_pub_.reset();
  path_pub_.reset();
  pose_pub_.reset();
  odom_pub_.reset();
  odom_sub_.reset();
  cloud_sub_.reset();
  imu_sub_.reset();

  RCLCPP_INFO(get_logger(), "Cleaning Up end");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_shutdown(const rclcpp_lifecycle::State &state)
{
  RCLCPP_INFO(get_logger(), "Shutting Down from %s", state.label().c_str());

  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_error(const rclcpp_lifecycle::State &state)
{
  RCLCPP_FATAL(get_logger(), "Error Processing from %s", state.label().c_str());

  return CallbackReturn::SUCCESS;
}

void PCLLocalization::initializeParameters()
{
  RCLCPP_INFO(get_logger(), "initializeParameters");
  get_parameter("publish_tf", publish_tf_);
  get_parameter("global_frame_id", global_frame_id_);
  get_parameter("odom_frame_id", odom_frame_id_);
  get_parameter("base_frame_id", base_frame_id_);
  get_parameter("registration_method", registration_method_);
  get_parameter("score_threshold", score_threshold_);
  get_parameter("ndt_resolution", ndt_resolution_);
  get_parameter("ndt_step_size", ndt_step_size_);
  get_parameter("ndt_num_threads", ndt_num_threads_);
  get_parameter("ndt_max_iterations", ndt_max_iterations_);
  get_parameter("transform_epsilon", transform_epsilon_);
  get_parameter("voxel_leaf_size", voxel_leaf_size_);
  get_parameter("scan_max_range", scan_max_range_);
  get_parameter("scan_min_range", scan_min_range_);
  get_parameter("scan_period", scan_period_);
  get_parameter("use_pcd_map", use_pcd_map_);
  get_parameter("map_path", map_path_);
  get_parameter("map_offset_x", map_offset_x_);
  get_parameter("map_offset_y", map_offset_y_);
  get_parameter("map_offset_z", map_offset_z_);
  get_parameter("set_initial_pose", set_initial_pose_);
  get_parameter("initial_pose_x", initial_pose_x_);
  get_parameter("initial_pose_y", initial_pose_y_);
  get_parameter("initial_pose_z", initial_pose_z_);
  get_parameter("initial_pose_qx", initial_pose_qx_);
  get_parameter("initial_pose_qy", initial_pose_qy_);
  get_parameter("initial_pose_qz", initial_pose_qz_);
  get_parameter("initial_pose_qw", initial_pose_qw_);
  get_parameter("use_odom", use_odom_);
  get_parameter("use_imu", use_imu_);
  get_parameter("enable_debug", enable_debug_);

  RCLCPP_INFO(get_logger(), "publish_tf: %d", publish_tf_);
  RCLCPP_INFO(get_logger(), "global_frame_id: %s", global_frame_id_.c_str());
  RCLCPP_INFO(get_logger(), "odom_frame_id: %s", odom_frame_id_.c_str());
  RCLCPP_INFO(get_logger(), "base_frame_id: %s", base_frame_id_.c_str());
  RCLCPP_INFO(get_logger(), "registration_method: %s", registration_method_.c_str());
  RCLCPP_INFO(get_logger(), "ndt_resolution: %lf", ndt_resolution_);
  RCLCPP_INFO(get_logger(), "ndt_step_size: %lf", ndt_step_size_);
  RCLCPP_INFO(get_logger(), "ndt_num_threads: %d", ndt_num_threads_);
  RCLCPP_INFO(get_logger(), "transform_epsilon: %lf", transform_epsilon_);
  RCLCPP_INFO(get_logger(), "voxel_leaf_size: %lf", voxel_leaf_size_);
  RCLCPP_INFO(get_logger(), "scan_max_range: %lf", scan_max_range_);
  RCLCPP_INFO(get_logger(), "scan_min_range: %lf", scan_min_range_);
  RCLCPP_INFO(get_logger(), "scan_period: %lf", scan_period_);
  RCLCPP_INFO(get_logger(), "use_pcd_map: %d", use_pcd_map_);
  RCLCPP_INFO(get_logger(), "map_path: %s", map_path_.c_str());
  RCLCPP_INFO(get_logger(), "map_offset_x: %lf", map_offset_x_);
  RCLCPP_INFO(get_logger(), "map_offset_y: %lf", map_offset_y_);
  RCLCPP_INFO(get_logger(), "map_offset_z: %lf", map_offset_z_);
  RCLCPP_INFO(get_logger(), "set_initial_pose: %d", set_initial_pose_);
  RCLCPP_INFO(get_logger(), "use_odom: %d", use_odom_);
  RCLCPP_INFO(get_logger(), "use_imu: %d", use_imu_);
  RCLCPP_INFO(get_logger(), "enable_debug: %d", enable_debug_);
  RCLCPP_INFO(get_logger(), "initializeParameters end");
}

void PCLLocalization::initializePubSub()
{
  RCLCPP_INFO(get_logger(), "initializePubSub");

  pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "pcl_pose",
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "path",
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "pcl_odom",
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  initial_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "initial_map",
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose", rclcpp::SystemDefaultsQoS(),
      std::bind(&PCLLocalization::initialPoseReceived, this, std::placeholders::_1));

  map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      std::bind(&PCLLocalization::mapReceived, this, std::placeholders::_1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::SensorDataQoS(),
      std::bind(&PCLLocalization::odomReceived, this, std::placeholders::_1));

  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "ouster/points", rclcpp::SensorDataQoS(),
      std::bind(&PCLLocalization::cloudReceived, this, std::placeholders::_1));

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "imu/data", rclcpp::SensorDataQoS(),
      std::bind(&PCLLocalization::imuReceived, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "initializePubSub end");
}

void PCLLocalization::initializeRegistration()
{
  RCLCPP_INFO(get_logger(), "initializeRegistration");

  if (registration_method_ == "GICP")
  {
    boost::shared_ptr<pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>> gicp(
        new pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>());
    gicp->setTransformationEpsilon(transform_epsilon_);
    registration_ = gicp;
  }
  else if (registration_method_ == "NDT")
  {
    boost::shared_ptr<pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>> ndt(
        new pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
    ndt->setStepSize(ndt_step_size_);
    ndt->setResolution(ndt_resolution_);
    ndt->setTransformationEpsilon(transform_epsilon_);
    registration_ = ndt;
  }
  else if (registration_method_ == "NDT_OMP")
  {
    pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr ndt_omp(
        new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
    ndt_omp->setStepSize(ndt_step_size_);
    ndt_omp->setResolution(ndt_resolution_);
    ndt_omp->setTransformationEpsilon(transform_epsilon_);
    if (ndt_num_threads_ > 0)
    {
      ndt_omp->setNumThreads(ndt_num_threads_);
    }
    else
    {
      ndt_omp->setNumThreads(omp_get_max_threads());
    }
    registration_ = ndt_omp;
  }
  else if (registration_method_ == "GICP_OMP")
  {
    pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>::Ptr gicp_omp(
        new pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>());
    gicp_omp->setTransformationEpsilon(transform_epsilon_);
    registration_ = gicp_omp;
  }
  else
  {
    RCLCPP_ERROR(get_logger(), "Invalid registration method.");
    exit(EXIT_FAILURE);
  }
  registration_->setMaximumIterations(ndt_max_iterations_);

  voxel_grid_filter_.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
  RCLCPP_INFO(get_logger(), "initializeRegistration end");
}

void PCLLocalization::initialPoseReceived(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "initialPoseReceived");
  if (msg->header.frame_id != global_frame_id_)
  {
    RCLCPP_WARN(this->get_logger(), "initialpose_frame_id does not match global_frame_id");
    return;
  }
  initialpose_recieved_ = true;
  corrent_pose_with_cov_stamped_ptr_ = msg;
  pose_pub_->publish(*corrent_pose_with_cov_stamped_ptr_);

  cloudReceived(last_scan_ptr_);
  RCLCPP_INFO(get_logger(), "initialPoseReceived end");
}

void PCLLocalization::mapReceived(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "mapReceived");
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);

  if (msg->header.frame_id != global_frame_id_)
  {
    RCLCPP_WARN(this->get_logger(), "map_frame_id does not match global_frame_id");
    return;
  }

  pcl::fromROSMsg(*msg, *map_cloud_ptr);

  if (registration_method_ == "GICP" || registration_method_ == "GICP_OMP")
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    voxel_grid_filter_.setInputCloud(map_cloud_ptr);
    voxel_grid_filter_.filter(*filtered_cloud_ptr);
    registration_->setInputTarget(filtered_cloud_ptr);
  }
  else
  {
    registration_->setInputTarget(map_cloud_ptr);
  }

  map_recieved_ = true;
  RCLCPP_INFO(get_logger(), "mapReceived end");
}

void PCLLocalization::odomReceived(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  if (!use_odom_)
  {
    return;
  }
  RCLCPP_INFO(get_logger(), "odomReceived");

  double current_odom_received_time = msg->header.stamp.sec +
                                      msg->header.stamp.nanosec * 1e-9;
  double dt_odom = current_odom_received_time - last_odom_received_time_;
  last_odom_received_time_ = current_odom_received_time;
  if (dt_odom > 1.0 /* [sec] */)
  {
    RCLCPP_WARN(this->get_logger(), "odom time interval is too large");
    return;
  }
  if (dt_odom < 0.0 /* [sec] */)
  {
    RCLCPP_WARN(this->get_logger(), "odom time interval is negative");
    return;
  }

  tf2::Quaternion previous_quat_tf;
  double roll, pitch, yaw;
  tf2::fromMsg(corrent_pose_with_cov_stamped_ptr_->pose.pose.orientation, previous_quat_tf);

  tf2::Matrix3x3(previous_quat_tf).getRPY(roll, pitch, yaw);

  roll += msg->twist.twist.angular.x * dt_odom;
  pitch += msg->twist.twist.angular.y * dt_odom;
  yaw += msg->twist.twist.angular.z * dt_odom;

  Eigen::Quaterniond quat_eig =
      Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

  geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(quat_eig);

  Eigen::Vector3d odom{
      msg->twist.twist.linear.x,
      msg->twist.twist.linear.y,
      msg->twist.twist.linear.z};
  Eigen::Vector3d delta_position = quat_eig.matrix() * dt_odom * odom;

  corrent_pose_with_cov_stamped_ptr_->pose.pose.position.x += delta_position.x();
  corrent_pose_with_cov_stamped_ptr_->pose.pose.position.y += delta_position.y();
  corrent_pose_with_cov_stamped_ptr_->pose.pose.position.z += delta_position.z();
  corrent_pose_with_cov_stamped_ptr_->pose.pose.orientation = quat_msg;
}

void PCLLocalization::imuReceived(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  if (!use_imu_)
  {
    return;
  }

  sensor_msgs::msg::Imu tf_converted_imu;

  try
  {
    const geometry_msgs::msg::TransformStamped transform = tfbuffer_.lookupTransform(
        base_frame_id_, msg->header.frame_id, tf2::TimePointZero);

    geometry_msgs::msg::Vector3Stamped angular_velocity, linear_acceleration, transformed_angular_velocity, transformed_linear_acceleration;
    geometry_msgs::msg::Quaternion transformed_quaternion;

    angular_velocity.header = msg->header;
    angular_velocity.vector = msg->angular_velocity;
    linear_acceleration.header = msg->header;
    linear_acceleration.vector = msg->linear_acceleration;

    tf2::doTransform(angular_velocity, transformed_angular_velocity, transform);
    tf2::doTransform(linear_acceleration, transformed_linear_acceleration, transform);

    tf_converted_imu.angular_velocity = transformed_angular_velocity.vector;
    tf_converted_imu.linear_acceleration = transformed_linear_acceleration.vector;
    tf_converted_imu.orientation = transformed_quaternion;
  }
  catch (tf2::TransformException &ex)
  {
    std::cout << "Failed to lookup transform" << std::endl;
    RCLCPP_WARN(this->get_logger(), "Failed to lookup transform.");
    return;
  }

  Eigen::Vector3f angular_velo{tf_converted_imu.angular_velocity.x, tf_converted_imu.angular_velocity.y,
                               tf_converted_imu.angular_velocity.z};
  Eigen::Vector3f acc{tf_converted_imu.linear_acceleration.x, tf_converted_imu.linear_acceleration.y, tf_converted_imu.linear_acceleration.z};
  Eigen::Quaternionf quat{msg->orientation.w, msg->orientation.x, msg->orientation.y,
                          msg->orientation.z};
  double imu_time = msg->header.stamp.sec +
                    msg->header.stamp.nanosec * 1e-9;

  lidar_undistortion_.getImu(angular_velo, acc, quat, imu_time);
}

void PCLLocalization::cloudReceived(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
    if (!map_recieved_ || !initialpose_recieved_)
    {
        return;
    }

    // Convert PointCloud2 message to PCL PointCloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *cloud_ptr);

    // Apply IMU-based distortion adjustment if enabled
    if (use_imu_)
    {
        double received_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        lidar_undistortion_.adjustDistortion(cloud_ptr, received_time);
    }

    // Filter the cloud using a voxel grid
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    voxel_grid_filter_.setInputCloud(cloud_ptr);
    voxel_grid_filter_.filter(*filtered_cloud_ptr);

    // Filter points by range
    double r;
    pcl::PointCloud<pcl::PointXYZI> tmp;
    for (const auto &p : filtered_cloud_ptr->points)
    {
        r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
        if (scan_min_range_ < r && r < scan_max_range_)
        {
            tmp.push_back(p);
        }
    }

    // Transform the point cloud to the base_link frame
    sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg_ptr(new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(tmp, *cloud_msg_ptr);
    cloud_msg_ptr->header.frame_id = msg->header.frame_id;
    cloud_msg_ptr->header.stamp = msg->header.stamp;
    sensor_msgs::msg::PointCloud2::SharedPtr transformed_cloud_msg_ptr(new sensor_msgs::msg::PointCloud2);
    try
    {
        tf2::TimePoint time_point = tf2::TimePoint(std::chrono::nanoseconds((msg->header.stamp.sec * 1000000000) + msg->header.stamp.nanosec));
        geometry_msgs::msg::TransformStamped transform_stamped = tfbuffer_.lookupTransform(base_frame_id_, msg->header.frame_id, time_point);
        tf2::doTransform(*cloud_msg_ptr, *transformed_cloud_msg_ptr, transform_stamped);
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(this->get_logger(), "Failed to lookup transform.");
        return;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*transformed_cloud_msg_ptr, *transformed_cloud_ptr);
    registration_->setInputSource(transformed_cloud_ptr);

    // Perform point cloud registration
    Eigen::Affine3d affine;
    tf2::fromMsg(corrent_pose_with_cov_stamped_ptr_->pose.pose, affine);
    Eigen::Matrix4f init_guess = affine.matrix().cast<float>();
    pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    rclcpp::Clock system_clock;
    rclcpp::Time time_align_start = system_clock.now();
    registration_->align(*output_cloud, init_guess);
    rclcpp::Time time_align_end = system_clock.now();

    // Check alignment success and fitness score
    bool has_converged = registration_->hasConverged();
    double fitness_score = registration_->getFitnessScore();
    if (!has_converged)
    {
        RCLCPP_WARN(get_logger(), "The registration didn't converge.");
        return;
    }
    if (fitness_score > score_threshold_)
    {
        RCLCPP_WARN(get_logger(), "The fitness score is over %lf.", score_threshold_);
    }

    // // Update pose information
    // Eigen::Matrix4f final_transformation = registration_->getFinalTransformation();
    // Eigen::Matrix3d rot_mat = final_transformation.block<3, 3>(0, 0).cast<double>();
    // Eigen::Quaterniond quat_eig(rot_mat);

    // map to odom
    geometry_msgs::msg::TransformStamped map_to_odom;
    try
    {
        map_to_odom = tfbuffer_.lookupTransform(odom_frame_id_, global_frame_id_, tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(this->get_logger(), "Failed to lookup transform.");
        return;
    }

    // Update pose information
    Eigen::Matrix4f final_transformation = registration_->getFinalTransformation();
    Eigen::Matrix3d rot_mat = final_transformation.block<3, 3>(0, 0).cast<double>();
    Eigen::Quaterniond quat_eig(rot_mat);

    // lookup transform from map to odom
    geometry_msgs::msg::TransformStamped map_to_odom_transform;
    try
    {
        map_to_odom_transform = tfbuffer_.lookupTransform(odom_frame_id_, global_frame_id_, tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(this->get_logger(), "Failed to lookup transform.");
        return;
    }

    // Update pose information offset map_to_odom_transform
    final_transformation(0, 3) = final_transformation(0, 3) + map_to_odom_transform.transform.translation.x;
    final_transformation(1, 3) = final_transformation(1, 3) + map_to_odom_transform.transform.translation.y;
    final_transformation(2, 3) = final_transformation(2, 3) + map_to_odom_transform.transform.translation.z;


    geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(quat_eig);
    corrent_pose_with_cov_stamped_ptr_->header.stamp = msg->header.stamp;
    corrent_pose_with_cov_stamped_ptr_->header.frame_id = odom_frame_id_;
    // corrent_pose_with_cov_stamped_ptr_->header.frame_id = global_frame_id_;
    corrent_pose_with_cov_stamped_ptr_->pose.pose.position.x = final_transformation(0, 3);
    corrent_pose_with_cov_stamped_ptr_->pose.pose.position.y = final_transformation(1, 3);
    corrent_pose_with_cov_stamped_ptr_->pose.pose.position.z = final_transformation(2, 3);
    corrent_pose_with_cov_stamped_ptr_->pose.pose.orientation = quat_msg;
    pose_pub_->publish(*corrent_pose_with_cov_stamped_ptr_);

    // Publish the transform if required
    if (publish_tf_)
    {
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = msg->header.stamp;
        transform_stamped.header.frame_id = odom_frame_id_;
        transform_stamped.child_frame_id = base_frame_id_;
        transform_stamped.transform.translation.x = final_transformation(0, 3);
        transform_stamped.transform.translation.y = final_transformation(1, 3);
        transform_stamped.transform.translation.z = final_transformation(2, 3);
        transform_stamped.transform.rotation = quat_msg;
        broadcaster_.sendTransform(transform_stamped);
    }

    // Publish path
    geometry_msgs::msg::PoseStamped::SharedPtr pose_stamped_ptr(new geometry_msgs::msg::PoseStamped);
    pose_stamped_ptr->header.stamp = msg->header.stamp;
    pose_stamped_ptr->header.frame_id = base_frame_id_;
    pose_stamped_ptr->pose = corrent_pose_with_cov_stamped_ptr_->pose.pose;
    path_ptr_->poses.push_back(*pose_stamped_ptr);
    path_pub_->publish(*path_ptr_);

    geometry_msgs::msg::PoseStamped::SharedPtr pose_stamped_ptr_unoffset(new geometry_msgs::msg::PoseStamped);
    pose_stamped_ptr_unoffset->header.stamp = msg->header.stamp;
    pose_stamped_ptr_unoffset->header.frame_id = base_frame_id_;
    pose_stamped_ptr_unoffset->pose.position.x = final_transformation(0, 3) - map_to_odom_transform.transform.translation.x;
    pose_stamped_ptr_unoffset->pose.position.y = final_transformation(1, 3) - map_to_odom_transform.transform.translation.y;
    pose_stamped_ptr_unoffset->pose.position.z = final_transformation(2, 3) - map_to_odom_transform.transform.translation.z;
    // pose_stamped_ptr_unoffset->pose.position.x = final_transformation(0, 3) - map_to_odom.transform.translation.x;
    // pose_stamped_ptr_unoffset->pose.position.y = final_transformation(1, 3) - map_to_odom.transform.translation.y;
    // pose_stamped_ptr_unoffset->pose.position.z = final_transformation(2, 3) - map_to_odom.transform.translation.z;
    pose_stamped_ptr_unoffset->pose.orientation = quat_msg;
    path_ptr_unoffset_->poses.push_back(*pose_stamped_ptr_unoffset);

    last_scan_ptr_ = msg;

    // Debug information
    if (enable_debug_)
    {
        std::cout << "Filtered cloud points: " << filtered_cloud_ptr->size() << std::endl;
        std::cout << "Align time: " << time_align_end.seconds() - time_align_start.seconds() << " sec" << std::endl;
        std::cout << "Converged: " << has_converged << ", Fitness score: " << fitness_score << std::endl;
        std::cout << "Transformation matrix: " << final_transformation << std::endl;
    }

    // Compute velocity and angular velocity
    if (path_ptr_unoffset_->poses.size() >= 2)
    {
        double current_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        double previous_time = path_ptr_unoffset_->poses[path_ptr_unoffset_->poses.size() - 2].header.stamp.sec +
                               path_ptr_unoffset_->poses[path_ptr_unoffset_->poses.size() - 2].header.stamp.nanosec * 1e-9;
        double dt = current_time - previous_time;

        Eigen::Vector3d current_position(
            corrent_pose_with_cov_stamped_ptr_->pose.pose.position.x - map_to_odom_transform.transform.translation.x,
            corrent_pose_with_cov_stamped_ptr_->pose.pose.position.y - map_to_odom_transform.transform.translation.y,
            corrent_pose_with_cov_stamped_ptr_->pose.pose.position.z - map_to_odom_transform.transform.translation.z
            // corrent_pose_with_cov_stamped_ptr_->pose.pose.position.x - map_to_odom.transform.translation.x,
            // corrent_pose_with_cov_stamped_ptr_->pose.pose.position.y - map_to_odom.transform.translation.y,
            // corrent_pose_with_cov_stamped_ptr_->pose.pose.position.z - map_to_odom.transform.translation.z
          );
        Eigen::Vector3d previous_position(
            path_ptr_unoffset_->poses[path_ptr_unoffset_->poses.size() - 2].pose.position.x,
            path_ptr_unoffset_->poses[path_ptr_unoffset_->poses.size() - 2].pose.position.y,
            path_ptr_unoffset_->poses[path_ptr_unoffset_->poses.size() - 2].pose.position.z);

        Eigen::Vector3d velocity = (current_position - previous_position) / dt;

        nav_msgs::msg::Odometry odometry;
        odometry.header.stamp = msg->header.stamp;
        odometry.header.frame_id = odom_frame_id_;
        odometry.child_frame_id = base_frame_id_;
        odometry.pose.pose = corrent_pose_with_cov_stamped_ptr_->pose.pose;
        odometry.pose.pose.position.x = corrent_pose_with_cov_stamped_ptr_->pose.pose.position.x - map_to_odom_transform.transform.translation.x;
        odometry.pose.pose.position.y = corrent_pose_with_cov_stamped_ptr_->pose.pose.position.y - map_to_odom_transform.transform.translation.y;
        odometry.pose.pose.position.z = corrent_pose_with_cov_stamped_ptr_->pose.pose.position.z - map_to_odom_transform.transform.translation.z;
        // odometry.pose.pose.position.x = corrent_pose_with_cov_stamped_ptr_->pose.pose.position.x - map_to_odom.transform.translation.x;
        // odometry.pose.pose.position.y = corrent_pose_with_cov_stamped_ptr_->pose.pose.position.y - map_to_odom.transform.translation.y;
        // odometry.pose.pose.position.z = corrent_pose_with_cov_stamped_ptr_->pose.pose.position.z - map_to_odom.transform.translation.z;
        odometry.pose.pose.orientation = corrent_pose_with_cov_stamped_ptr_->pose.pose.orientation;
        odometry.twist.twist.linear.x = velocity.x();
        odometry.twist.twist.linear.y = velocity.y();
        odometry.twist.twist.linear.z = velocity.z();
        odom_pub_->publish(odometry);
    }
}
// void PCLLocalization::cloudReceived(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
// {
//     /* ---------- Early exits ---------- */
//     if (!map_recieved_ || !initialpose_recieved_)
//         return;

//     /* ---------- 1. Raw → filtered cloud ---------- */
//     pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
//     pcl::fromROSMsg(*msg, *cloud_ptr);

//     if (use_imu_)
//     {
//         const double t = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
//         lidar_undistortion_.adjustDistortion(cloud_ptr, t);
//     }

//     pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
//     voxel_grid_filter_.setInputCloud(cloud_ptr);
//     voxel_grid_filter_.filter(*filtered_cloud_ptr);

//     pcl::PointCloud<pcl::PointXYZI> in_range_cloud;
//     for (const auto &p : filtered_cloud_ptr->points)
//     {
//         const double r = std::hypot(p.x, p.y);
//         if (scan_min_range_ < r && r < scan_max_range_)
//             in_range_cloud.push_back(p);
//     }

//     /* ---------- 2. cloud → base_link frame ---------- */
//     sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg(new sensor_msgs::msg::PointCloud2);
//     pcl::toROSMsg(in_range_cloud, *cloud_msg);
//     cloud_msg->header = msg->header;

//     sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg_bl(new sensor_msgs::msg::PointCloud2);
//     try
//     {
//         const tf2::TimePoint tp{std::chrono::nanoseconds((msg->header.stamp.sec * 1'000'000'000ll) + msg->header.stamp.nanosec)};
//         const auto tf_lidar2base = tfbuffer_.lookupTransform(base_frame_id_, msg->header.frame_id, tp);
//         tf2::doTransform(*cloud_msg, *cloud_msg_bl, tf_lidar2base);
//     }
//     catch (const tf2::TransformException &ex)
//     {
//         RCLCPP_WARN(get_logger(), "Failed to lookup transform lidar→base_link: %s", ex.what());
//         return;
//     }

//     pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_bl_ptr(new pcl::PointCloud<pcl::PointXYZI>);
//     pcl::fromROSMsg(*cloud_msg_bl, *cloud_bl_ptr);
//     registration_->setInputSource(cloud_bl_ptr);

//     /* ---------- 3. Align ---------- */
//     Eigen::Affine3d T_prev;
//     tf2::fromMsg(corrent_pose_with_cov_stamped_ptr_->pose.pose, T_prev);
//     const Eigen::Matrix4f init_guess = T_prev.matrix().cast<float>();

//     pcl::PointCloud<pcl::PointXYZI> aligned;
//     const auto t_start = rclcpp::Clock().now();
//     registration_->align(aligned, init_guess);
//     const auto t_end   = rclcpp::Clock().now();

//     if (!registration_->hasConverged())
//     {
//         RCLCPP_WARN(get_logger(), "ICP did not converge");
//         return;
//     }
//     if (registration_->getFitnessScore() > score_threshold_)
//     {
//         RCLCPP_WARN(get_logger(), "ICP score %.3f exceeds threshold %.3f", registration_->getFitnessScore(), score_threshold_);
//     }

//     const Eigen::Matrix4f T_map_base_f = registration_->getFinalTransformation();
//     const Eigen::Matrix3d R_map_base   = T_map_base_f.block<3,3>(0,0).cast<double>();
//     const Eigen::Vector3d t_map_base   = T_map_base_f.block<3,1>(0,3).cast<double>();

//     const Eigen::Quaterniond q_map_base(R_map_base);

//     /* ---------- 4. TF map→odom (global EKF가 방송) ---------- */
//     geometry_msgs::msg::TransformStamped map_to_odom;
//     try
//     {
//         map_to_odom = tfbuffer_.lookupTransform(odom_frame_id_, global_frame_id_, tf2::TimePointZero);
//     }
//     catch (const tf2::TransformException &ex)
//     {
//         RCLCPP_WARN(get_logger(), "Failed to lookup map→odom: %s", ex.what());
//         return;
//     }

//     /* ---------- 5. Publish /pcl_pose (map frame) ---------- */
//     corrent_pose_with_cov_stamped_ptr_->header.stamp      = msg->header.stamp;
//     corrent_pose_with_cov_stamped_ptr_->header.frame_id   = global_frame_id_;   // "map"
//     corrent_pose_with_cov_stamped_ptr_->pose.pose.position.x = t_map_base.x();
//     corrent_pose_with_cov_stamped_ptr_->pose.pose.position.y = t_map_base.y();
//     corrent_pose_with_cov_stamped_ptr_->pose.pose.position.z = t_map_base.z();
//     corrent_pose_with_cov_stamped_ptr_->pose.pose.orientation = tf2::toMsg(q_map_base);
//     pose_pub_->publish(*corrent_pose_with_cov_stamped_ptr_);

//     /* ---------- 6. (optional) odom→base_link TF & /pcl_odom ---------- */
//     const Eigen::Vector3d t_odom_base = t_map_base - Eigen::Vector3d{
//         map_to_odom.transform.translation.x,
//         map_to_odom.transform.translation.y,
//         map_to_odom.transform.translation.z};

//     if (publish_tf_)
//     {
//         geometry_msgs::msg::TransformStamped tf_odom_base;
//         tf_odom_base.header.stamp            = msg->header.stamp;
//         tf_odom_base.header.frame_id         = odom_frame_id_;
//         tf_odom_base.child_frame_id          = base_frame_id_;
//         tf_odom_base.transform.translation.x = t_odom_base.x();
//         tf_odom_base.transform.translation.y = t_odom_base.y();
//         tf_odom_base.transform.translation.z = t_odom_base.z();
//         tf_odom_base.transform.rotation      = tf2::toMsg(q_map_base);
//         broadcaster_.sendTransform(tf_odom_base);
//     }

//     /* ---------- 7. nav_msgs/Path (map & odom) ---------- */
//     // Global path (map)
//     {
//         geometry_msgs::msg::PoseStamped p;
//         p.header.stamp    = msg->header.stamp;
//         p.header.frame_id = global_frame_id_;
//         p.pose            = corrent_pose_with_cov_stamped_ptr_->pose.pose;
//         path_ptr_->poses.push_back(p);
//         path_pub_->publish(*path_ptr_);
//     }
//     // Local path (odom)
//     {
//         if (path_ptr_unoffset_->poses.empty())
//             path_ptr_unoffset_->header.frame_id = odom_frame_id_;

//         geometry_msgs::msg::PoseStamped p;
//         p.header.stamp    = msg->header.stamp;
//         p.header.frame_id = odom_frame_id_;
//         p.pose.position.x = t_odom_base.x();
//         p.pose.position.y = t_odom_base.y();
//         p.pose.position.z = t_odom_base.z();
//         p.pose.orientation = tf2::toMsg(q_map_base);
//         path_ptr_unoffset_->poses.push_back(p);
//     }

//     /* ---------- 8. Derived odometry (odom frame) ---------- */
//     if (path_ptr_unoffset_->poses.size() >= 2)
//     {
//         const auto &p_curr = path_ptr_unoffset_->poses.back();
//         const auto &p_prev = path_ptr_unoffset_->poses[path_ptr_unoffset_->poses.size() - 2];
//         const double dt = (p_curr.header.stamp - p_prev.header.stamp).seconds();
//         if (dt > 0.0)
//         {
//             const Eigen::Vector3d curr{p_curr.pose.position.x, p_curr.pose.position.y, p_curr.pose.position.z};
//             const Eigen::Vector3d prev{p_prev.pose.position.x, p_prev.pose.position.y, p_prev.pose.position.z};
//             const Eigen::Vector3d vel  = (curr - prev) / dt;

//             nav_msgs::msg::Odometry odom_msg;
//             odom_msg.header            = p_curr.header; // stamp & frame=odom
//             odom_msg.child_frame_id    = base_frame_id_;
//             odom_msg.pose.pose         = p_curr.pose;
//             odom_msg.twist.twist.linear.x = vel.x();
//             odom_msg.twist.twist.linear.y = vel.y();
//             odom_msg.twist.twist.linear.z = vel.z();
//             odom_pub_->publish(odom_msg);
//         }
//     }
// }