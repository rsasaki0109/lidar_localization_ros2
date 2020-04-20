#include <pcl_localization/pcl_localization_component.hpp>
PCLLocalization::PCLLocalization(const rclcpp::NodeOptions & options)
      : rclcpp_lifecycle::LifecycleNode("pcl_localization", options),
        broadcaster_(this)
{
  declare_parameter("global_frame_id", "map");
  declare_parameter("odom_frame_id", "odom");
  declare_parameter("base_frame_id", "base_link");
  declare_parameter("ndt_resolution", 1.0);
  declare_parameter("voxel_leaf_size", 0.2);
  declare_parameter("scan_max_range", 100.0);
  declare_parameter("scan_min_range", 1.0);
  declare_parameter("use_pcd_map", false);
  declare_parameter("map_path", "/map/map.pcd");
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
}

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn PCLLocalization::on_configure(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Configuring");
    
    initializeParameters();
    initializePubSub();
    initializeRegistration();
    
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn PCLLocalization::on_activate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Activating");

    pose_pub_->on_activate();

    if (set_initial_pose_) {
      auto msg = std::make_shared<geometry_msgs::msg::PoseStamped>();

      msg->header.stamp = now();
      msg->header.frame_id = global_frame_id_;
      msg->pose.position.x = initial_pose_x_;
      msg->pose.position.y = initial_pose_y_;
      msg->pose.position.z = initial_pose_z_;
      msg->pose.orientation.x = initial_pose_qx_;
      msg->pose.orientation.y = initial_pose_qy_;
      msg->pose.orientation.z = initial_pose_qz_;
      msg->pose.orientation.w = initial_pose_qw_;

      initialPoseReceived(msg);
    }

    if (use_pcd_map_) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::io::loadPCDFile(map_path_, *map_cloud);
      ndt_.setInputTarget(map_cloud);
      map_recieved_ = true;
    }

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn PCLLocalization::on_deactivate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Deactivating");

    pose_pub_->on_deactivate();

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn PCLLocalization::on_cleanup(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Cleaning Up");
    initial_pose_sub_.reset();
    pose_pub_.reset();
    odom_sub_.reset();
    cloud_sub_.reset();

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
    get_parameter("global_frame_id", global_frame_id_);
    get_parameter("odom_frame_id", odom_frame_id_);
    get_parameter("ndt_resolution", ndt_resolution_);
    get_parameter("voxel_leaf_size", voxel_leaf_size_);
    get_parameter("scan_max_range", scan_max_range_);
    get_parameter("scan_min_range", scan_min_range_);
    get_parameter("use_pcd_map", use_pcd_map_);
    get_parameter("map_path", map_path_);
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
  }

  void PCLLocalization::initializePubSub()
  {
    RCLCPP_INFO(get_logger(), "initializePubSub");

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "pcl_pose",
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "initialpose", rclcpp::SystemDefaultsQoS(),
      std::bind(&PCLLocalization::initialPoseReceived, this, std::placeholders::_1));

    map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      std::bind(&PCLLocalization::mapReceived, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::SensorDataQoS(),
      std::bind(&PCLLocalization::odomReceived, this, std::placeholders::_1));

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "cloud", rclcpp::SensorDataQoS(),
      std::bind(&PCLLocalization::cloudReceived, this, std::placeholders::_1));
  }

  void PCLLocalization::initializeRegistration()
  {
    RCLCPP_INFO(get_logger(), "initializeRegistration");

    ndt_.setResolution(ndt_resolution_);
    ndt_.setTransformationEpsilon(0.01);

    voxel_grid_filter_.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
  }

  void PCLLocalization::initialPoseReceived(geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(), "initialPoseReceived");
    if(msg->header.frame_id != global_frame_id_){
      RCLCPP_WARN(this->get_logger(), "initialpose_frame_id does not match　global_frame_id");
      return;
    };
    initialpose_recieved_ = true;
    corrent_pose_stamped_ = *msg;
    pose_pub_->publish(corrent_pose_stamped_);
  }

  void PCLLocalization::mapReceived(sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if(map_recieved_) return;
    RCLCPP_INFO(get_logger(), "mapReceived");
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    if(msg->header.frame_id != global_frame_id_){
      RCLCPP_WARN(this->get_logger(), "map_frame_id does not match　global_frame_id");
      return;
    };

    pcl::fromROSMsg(*msg,*map_cloud);
    ndt_.setInputTarget(map_cloud);
    map_recieved_ = true;
  }

  void PCLLocalization::odomReceived(nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    if(!use_odom_) return;
    RCLCPP_INFO(get_logger(), "odomReceived");

    double current_odom_received_time = msg->header.stamp.sec +
      msg->header.stamp.nanosec * 1e-9;
    double dt_odom = current_odom_received_time - last_odom_received_time_;
    last_odom_received_time_ = current_odom_received_time;
    if (dt_odom > 5.0 /* [sec] */) {
      RCLCPP_WARN(this->get_logger(), "odom time interval is too large");
      return;
    }

    tf2::Quaternion previous_quat_tf;
    double roll, pitch, yaw;
    tf2::fromMsg(corrent_pose_stamped_.pose.orientation, previous_quat_tf);
    tf2::Matrix3x3(previous_quat_tf).getRPY(roll, pitch, yaw);

    roll += msg->twist.twist.angular.x * dt_odom;
    pitch += msg->twist.twist.angular.y * dt_odom;
    yaw += msg->twist.twist.angular.z * dt_odom;

    Eigen::Quaterniond quat_eig =
      Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

    geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(quat_eig);
    
    Eigen::Vector3d odom{msg->twist.twist.linear.x,
      msg->twist.twist.linear.y,
      msg->twist.twist.linear.z};
    Eigen::Vector3d delta_position = quat_eig.matrix() * dt_odom * odom;

    corrent_pose_stamped_.pose.position.x += delta_position.x();
    corrent_pose_stamped_.pose.position.y += delta_position.y();
    corrent_pose_stamped_.pose.position.z += delta_position.z();
    corrent_pose_stamped_.pose.orientation = quat_msg;
  }

  // void PCLLocalization::imuReceived(sensor_msgs::msg::Imu::ConstSharedPtr msg){}

  void PCLLocalization::cloudReceived(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    if(!map_recieved_ || !initialpose_recieved_) return;
    RCLCPP_INFO(get_logger(), "cloudReceived");
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg,*cloud_ptr);

    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    voxel_grid_filter_.setInputCloud(cloud_ptr);
    voxel_grid_filter_.filter(*filtered_cloud_ptr);
    // TODO:distortion
    double r;
    pcl::PointCloud<pcl::PointXYZI> tmp;
    for (const auto &p : filtered_cloud_ptr->points) {
      r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
      if (scan_min_range_ < r && r < scan_max_range_)
      {
      tmp.push_back(p);
      }
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_ptr(new pcl::PointCloud<pcl::PointXYZI>(tmp));
    ndt_.setInputSource(tmp_ptr);

    Eigen::Affine3d affine;
    tf2::fromMsg(corrent_pose_stamped_.pose, affine);
    Eigen::Matrix4f init_guess = affine.matrix().cast<float>();

    pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    ndt_.align(*output_cloud, init_guess);

    Eigen::Matrix4f final_transformation = ndt_.getFinalTransformation();
    Eigen::Matrix3d rot_mat = final_transformation.block<3, 3>(0, 0).cast<double>();
    Eigen::Quaterniond quat_eig(rot_mat);
    geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(quat_eig);

    corrent_pose_stamped_.header.stamp = msg->header.stamp;
    corrent_pose_stamped_.pose.position.x = static_cast<double>(final_transformation(0, 3));
    corrent_pose_stamped_.pose.position.y = static_cast<double>(final_transformation(1, 3));
    corrent_pose_stamped_.pose.position.z = static_cast<double>(final_transformation(2, 3));
    corrent_pose_stamped_.pose.orientation = quat_msg;
    pose_pub_->publish(corrent_pose_stamped_);

    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = msg->header.stamp;
    transform_stamped.header.frame_id = global_frame_id_;
    transform_stamped.child_frame_id = odom_frame_id_;
    transform_stamped.transform.translation.x = static_cast<double>(final_transformation(0, 3));
    transform_stamped.transform.translation.y = static_cast<double>(final_transformation(1, 3));
    transform_stamped.transform.translation.z = static_cast<double>(final_transformation(2, 3));
    transform_stamped.transform.rotation = quat_msg;
    broadcaster_.sendTransform(transform_stamped);
    #if 0
    std::cout << "number of filtered cloud points: " << filtered_cloud_ptr->size() << std::endl;
    std::cout << "has converged: " << ndt_.hasConverged() << std::endl;
    std::cout << "fitness score: " << ndt_.getFitnessScore() << std::endl;
    std::cout << "Number of iteration: " << ndt_.getFinalNumIteration() << std::endl;
    std::cout << "final transformation:" << std::endl;
    std::cout <<  final_transformation << std::endl;
    std::cout << "-----------------------------------------------------" << std::endl;
    #endif
  }
