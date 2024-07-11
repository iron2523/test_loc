#include "dlio/odom.h"

// dlio::OdomNode::OdomNode(ros::NodeHandle node_handle, ros::NodeHandle mt_node_handle) : nh(node_handle), mt_nh(mt_node_handle) {
//   this->initialization();
// }
dlio::OdomNode::OdomNode(ros::NodeHandle node_handle, ros::NodeHandle mt_node_handle)
{
  this->nh = node_handle;
  this->mt_nh = mt_node_handle;
  this->initialization();
}

dlio::OdomNode::~OdomNode() {}

void dlio::OdomNode::setInitialPose() {
  if (set_initial_pose_) {
    auto msg = 
        boost::make_shared<geometry_msgs::PoseWithCovarianceStamped>();
    msg->header.stamp = ros::Time::now();
    msg->header.frame_id = global_frame_id_;
    msg->pose.pose.position.x = initial_pose_x_;
    msg->pose.pose.position.y = initial_pose_y_;
    msg->pose.pose.position.z = initial_pose_z_;
    msg->pose.pose.orientation.x = initial_pose_qx_;
    msg->pose.pose.orientation.y = initial_pose_qy_;
    msg->pose.pose.orientation.z = initial_pose_qz_;
    msg->pose.pose.orientation.w = initial_pose_qw_;
    initialPoseReceived(msg);
  }
}

void dlio::OdomNode::declare_parameters() {
  ROS_INFO("declare_parameters");
  declare_global_param();
  declare_odom_param();
  declare_extrinsics_param();
  declare_imu_param();
  declare_frame_param();
}

void dlio::OdomNode::initializeParameters() {
  ROS_INFO("initializeParameters");
  this->num_threads_ = omp_get_max_threads();
  this->dlio_initialized = false;
  this->first_valid_scan = false;
  this->first_imu_received_ = false;
  if (this->imu_calibrate_) {
    this->imu_calibrated = false;
  } else {
    this->imu_calibrated = true;
  }
  this->deskew_status = false;
  this->deskew_size = 0;
  scan_stamp = 0.0;
  prev_scan_stamp = 0.0;
  scan_dt = 0.1;
  last_save_stamp = ros::Time::now();
  //
  tf2::Quaternion qua;
  qua.setRPY(initial_pose_roll_, initial_pose_pitch_, initial_pose_yaw_);
  initial_pose_qx_ = qua.x();
  initial_pose_qy_ = qua.y();
  initial_pose_qz_ = qua.z();
  initial_pose_qw_ = qua.w();

  //
  this->T = Eigen::Matrix4f::Identity();
  this->T_prior = Eigen::Matrix4f::Identity();
  this->T_corr = Eigen::Matrix4f::Identity();
  this->T_corr_prev = Eigen::Matrix4f::Identity();

  this->origin = Eigen::Vector3f(0., 0., 0.);
  this->state.p = Eigen::Vector3f(0., 0., 0.);
  this->state.q = Eigen::Quaternionf(1., 0., 0., 0.);
  this->state.v.lin.b = Eigen::Vector3f(0., 0., 0.);
  this->state.v.lin.w = Eigen::Vector3f(0., 0., 0.);
  this->state.v.ang.b = Eigen::Vector3f(0., 0., 0.);
  this->state.v.ang.w = Eigen::Vector3f(0., 0., 0.);
  this->lidarPose.p = Eigen::Vector3f(0., 0., 0.);
  this->lidarPose.q = Eigen::Quaternionf(1., 0., 0., 0.);

  this->imu_meas.stamp = 0.;
  this->imu_meas.ang_vel[0] = 0.;
  this->imu_meas.ang_vel[1] = 0.;
  this->imu_meas.ang_vel[2] = 0.;
  this->imu_meas.lin_accel[0] = 0.;
  this->imu_meas.lin_accel[1] = 0.;
  this->imu_meas.lin_accel[2] = 0.;

  this->imu_buffer.set_capacity(this->imu_buffer_size_);
  this->first_imu_stamp = 0.;
  this->prev_imu_stamp = 0.;

  this->original_scan = pcl::make_shared<pcl::PointCloud<PointType>>();
  this->deskewed_scan = pcl::make_shared<pcl::PointCloud<PointType>>();
  this->current_scan = pcl::make_shared<pcl::PointCloud<PointType>>();

  this->first_scan_stamp = 0.;
  this->elapsed_time = 0.;

  this->gicp.setCorrespondenceRandomness(this->gicp_k_correspondences_);
  this->gicp.setMaxCorrespondenceDistance(this->gicp_max_corr_dist_);
  this->gicp.setMaximumIterations(this->gicp_max_iter_);
  this->gicp.setTransformationEpsilon(this->gicp_transformation_ep_);
  this->gicp.setRotationEpsilon(this->gicp_rotation_ep_);
  this->gicp.setInitialLambdaFactor(this->gicp_init_lambda_factor_);

  pcl::Registration<PointType, PointType>::KdTreeReciprocalPtr temp;
  this->gicp.setSearchMethodSource(temp, true);
  this->gicp.setSearchMethodTarget(temp, true);
  this->geo.first_opt_done = false;
  this->geo.prev_vel = Eigen::Vector3f(0., 0., 0.);
  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
  this->crop.setNegative(true);
  this->crop.setMin(Eigen::Vector4f(-this->crop_size_, -this->crop_size_,
                                    -this->crop_size_, 1.0));
  this->crop.setMax(Eigen::Vector4f(this->crop_size_, this->crop_size_,
                                    this->crop_size_, 1.0));

  this->voxel.setLeafSize(this->vf_res_, this->vf_res_, this->vf_res_);
  this->metrics.spaciousness.push_back(0.);
  this->metrics.density.push_back(this->gicp_max_corr_dist_);
}

void dlio::OdomNode::loadMap() {
  this->global_map_ =
    pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
    pcl::io::loadPCDFile(map_path_, *global_map_);

  this->gicp.setInputTarget(this->global_map_);
  this->global_map_cov = this->gicp.getTargetCovariances();
  this->gicp.registerInputTarget(this->global_map_);
  ROS_INFO("***************map size: %lu  *************\n",
              global_map_->size());
  // sensor_msgs::PointCloud2Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
  this->map_msg_ptr.reset(new sensor_msgs::PointCloud2());
  pcl::toROSMsg(*global_map_, *map_msg_ptr);
  map_msg_ptr->header.frame_id = this->global_frame_id_;

  this->initial_map_pub_.publish(*map_msg_ptr);
  // initial_map_pub_->publish(*map_msg_ptr);
  ROS_INFO("Initil Map Published");

  map_recieved_ = true;
}

void dlio::OdomNode::loadFullMap() {
  // this->global_map_ =
  //   pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
  //   pcl::io::loadPCDFile(map_path_, *global_map_);

  this->full_map_ = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
  pcl::io::loadPCDFile(full_map_path_, *full_map_);
  // this->gicp.setInputTarget(this->global_map_);
  // this->global_map_cov = this->gicp.getTargetCovariances();
  // this->gicp.registerInputTarget(this->global_map_);
  ROS_INFO("***************map size: %lu  *************\n",
              full_map_->size());
  // sensor_msgs::PointCloud2Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
  this->full_map_msg_ptr.reset(new sensor_msgs::PointCloud2());
  pcl::toROSMsg(*full_map_, *full_map_msg_ptr);
  full_map_msg_ptr->header.frame_id = this->full_global_frame_id_;

  this->initial_full_map_pub_.publish(*full_map_msg_ptr);
  // initial_map_pub_->publish(*map_msg_ptr);
  ROS_INFO("Initil FullMap Published");
  // map_recieved_ = true;
}


void dlio::OdomNode::loadPoseFromServer() {
  try {
    YAML::Node doc = YAML::LoadFile(init_pose_path_);
    initial_pose_x_ = yaml_get_value<double>(doc, "initial_pose_x");
    initial_pose_y_ = yaml_get_value<double>(doc, "initial_pose_y");
    initial_pose_yaw_ = yaml_get_value<double>(doc, "initial_pose_yaw");
    tf2::Quaternion qua;
    qua.setRPY(0.0, 0.0, initial_pose_yaw_);
    initial_pose_qx_ = qua.x();
    initial_pose_qy_ = qua.y();
    initial_pose_qz_ = qua.z();
    initial_pose_qw_ = qua.w();
    set_initial_pose_ = true;
  } catch (YAML::ParserException& e) {
    ROS_ERROR("Failed to parse YAML for reason: %s",
                 e.msg.c_str());
  } catch (YAML::BadFile& e) {
    ROS_ERROR("Failed to load YAML for reason: %s",
                 e.msg.c_str());
  } catch (YAML::Exception& e) {
    ROS_ERROR("Failed to exeption YAML for reason: %s",
                 e.msg.c_str());
  }
}

void dlio::OdomNode::savePoseToServer(const geometry_msgs::Pose2D& pose) {
  if (abs(pose.x - last_save_pose.x) > 0.06 ||
    abs(pose.y - last_save_pose.y) > 0.06 ||
    abs(pose.theta - last_save_pose.theta) > 0.06 || 
    (ros::Time::now() - last_save_stamp) > 
      ros::Duration(10)) {
    YAML::Emitter e;
    e << YAML::Precision(5);
    e << YAML::BeginMap;
    e << YAML::Key << "initial_pose_x" << YAML::Value << pose.x;
    e << YAML::Key << "initial_pose_y" << YAML::Value << pose.y;
    e << YAML::Key << "initial_pose_yaw" << YAML::Value << pose.theta;
    if (!e.good()) {
      ROS_WARN_STREAM("YAML writer failed with an error "
                                           << e.GetLastError()
                                           << ". The pose may be invalid.");
    }
    std::ofstream(init_pose_path_) << e.c_str();
    last_save_pose = pose;
    // last_save_stamp = get_clock()->now();
    last_save_stamp = ros::Time::now();
  }
}

void dlio::OdomNode::createPublishers() {
  pose_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("pcl_pose", 1);
  // pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pcl_pose", 1);
  // path_pub_ = create_publisher<nav_msgs::msg::Path>("path", 1);
  initial_map_pub_ = nh.advertise<sensor_msgs::PointCloud2>("initial_map", true);
  initial_full_map_pub_ = nh.advertise<sensor_msgs::PointCloud2>("initial_full_map", true);
  gicp_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("gicp_pose", 10, true);
  // this->publish_timer = mt_nh.createWallTimer(ros::WallDuration(0.02), std::bind(&dlio::OdomNode::publishPoseThread, this));

}

void dlio::OdomNode::createSubscribers() {
  // this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  // auto lidar_sub_ = ros::Subscriber();
  initial_pose_sub_ = this->mt_nh.subscribe("initialpose", 1, &dlio::OdomNode::initialPoseReceived, this);
  map_sub_ = this->nh.subscribe("map", 1, &dlio::OdomNode::mapReceived, this);
  odom_sub_ = this->nh.subscribe("odom", 1, &dlio::OdomNode::odomReceived, this);
  // yyy
  // lio_state_sub_ = this->nh.subscribe("lio_sam_stateinfo", 10, &dlio::OdomNode::lioReceived, this);
  // livox_sub_ = this->nh.subscribe<livox_ros_driver::CustomMsg>("cloud", 1, boost::bind(&dlio::OdomNode::moveFromCustomMsgCallback, this, _1));


  // 有话题就会执行 cloudReceived回调函数
  // cloud_sub_ = this->nh.subscribe("cloud", 1, &dlio::OdomNode::cloudReceived, this);
  imu_sub_ = this->nh.subscribe("imu", 1, &dlio::OdomNode::imuReceived, this);


  // 设置定时器，调用一次timerCallback函数
  cloud_data_sub_ = this->nh.subscribe("cloud", 1, &dlio::OdomNode::cloudData, this);
  // 1s效果还比较好，设置为每帧都地图匹配效果很差，0.5也还可以
  cloud_timer_ = this->nh.createWallTimer(ros::WallDuration(1), &dlio::OdomNode::timerCallbackCloud, this);

  
}

void dlio::OdomNode::initializePubSub() {
  ROS_INFO("initializePubSub");
  createPublishers();
  createSubscribers();
  // this->broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
  // this->broadcaster_ = std::shared_ptr<tf2_ros::TransformBroadcaster>();
  // this->broadcaster_ = boost::make_shared<tf2_ros::TransformBroadcaster>();
  // this->broadcaster_ = std::shared_ptr<tf2_ros::TransformBroadcaster>(*this);
  // this->publish_timer = nh.createTimer(ros::Duration(0.02), std::bind(&dlio::OdomNode::publishPoseThread, this));
  // 发布tf位姿变换， 0.02为50Hz，可以和imu的频率相同
  this->publish_timer = mt_nh.createWallTimer(ros::WallDuration(0.02), std::bind(&dlio::OdomNode::publishPoseThread, this));
  // 最后一个 true表示只触发一次
  this->publish_timer_ = mt_nh.createWallTimer(ros::WallDuration(5), std::bind(&dlio::OdomNode::publishInitialMap, this), true);
  this->publish_full_map_ = mt_nh.createWallTimer(ros::WallDuration(5), std::bind(&dlio::OdomNode::publishFullMap, this), true);
  // this->publish_gicp_pose_ = mt_nh.createWallTimer(ros::WallDuration(5), std::bind(&dlio::OdomNode::publishGicpPose, this));
  // this->publish_gicp_pose_ = mt_nh.createWallTimer(ros::WallDuration(12), std::bind(&dlio::OdomNode::publishGicpPose, this), true);

  // publish_timer_ = nh.createTimer(ros::Duration(0.02), &OdomNode::publishPoseThread, this);
}

void dlio::OdomNode::initialization() {
  declare_parameters();
  initializeParameters();
  initializePubSub();
  loadMap();
  loadFullMap();
  loadPoseFromServer();
  // 根据上次保存的运行结果init_pose来初始化第一帧位姿
  setInitialPose();
  ROS_INFO("initialized!!!");
}

void dlio::OdomNode::resetImu() {
  this->first_imu_received_ = false;
  this->first_imu_stamp = 0.;
  this->imu_calibrated = false;
  this->calibrate_accel_ = false;
  this->calibrate_gyro_ = false;
  ROS_INFO("resetImu...");
}

void dlio::OdomNode::setInputSource() {
  this->gicp.setInputSource(this->current_scan);
  this->gicp.calculateSourceCovariances();
}

void dlio::OdomNode::propagateState() {
  std::lock_guard<std::mutex> lock(this->geo.mtx);
  double dt = this->imu_meas.dt;
  // double dt = this->imu_meas.dt > 0.012f ? 0.01f : this->imu_meas.dt;
  // RCLCPP_INFO("dt in propagateState: %.8f", dt);
  Eigen::Quaternionf qhat = this->state.q, omega;
  Eigen::Vector3f world_accel;

  // Transform accel from body to world frame
  // 这个imu_meas是否是imu转到lidar下的？
  world_accel = qhat._transformVector(this->imu_meas.lin_accel);

  // Accel propogation
  // float px = this->state.v.lin.w[0] * dt + 0.5 * dt * dt * world_accel[0];
  // float py = this->state.v.lin.w[1] * dt + 0.5 * dt * dt * world_accel[1];
  // float pz = this->state.v.lin.w[2] * dt +  0.5 * dt * dt * (world_accel[2] -
  // this->gravity_); this->state.p[0] +=
  //     px;
  // this->state.p[1] +=
  //     py;
  // this->state.p[2] +=
  //     pz;

  this->state.p[0] +=
      this->state.v.lin.w[0] * dt + 0.5 * dt * dt * world_accel[0];
  this->state.p[1] +=
      this->state.v.lin.w[1] * dt + 0.5 * dt * dt * world_accel[1];
  this->state.p[2] += this->state.v.lin.w[2] * dt +
                      0.5 * dt * dt * (world_accel[2] - this->gravity_);
  // ROS_INFO("px, py, pz, dt in propagateState:[%.8f,%.8f,%.8f,%.8f]", 
  // this->state.p[0], this->state.p[1], this->state.p[2], dt);
  this->state.v.lin.w[0] += world_accel[0] * dt;
  this->state.v.lin.w[1] += world_accel[1] * dt;
  this->state.v.lin.w[2] += (world_accel[2] - this->gravity_) * dt;
  this->state.v.lin.b = this->state.q.toRotationMatrix().inverse() * this->state.v.lin.w;
  // Gyro propogation
  omega.w() = 0;
  omega.vec() = this->imu_meas.ang_vel;
  Eigen::Quaternionf tmp = qhat * omega;
  this->state.q.w() += 0.5 * dt * tmp.w();
  this->state.q.vec() += 0.5 * dt * tmp.vec();

  // Ensure quaternion is properly normalized
  this->state.q.normalize();

  this->state.v.ang.b = this->imu_meas.ang_vel;
  this->state.v.ang.w = this->state.q.toRotationMatrix() * this->state.v.ang.b;
}

void dlio::OdomNode::deskewPointcloud() {
  // 创建一个新的空点云用于存放去斜后的点云数据
  pcl::PointCloud<PointType>::Ptr deskewed_scan_ =
      pcl::make_shared<pcl::PointCloud<PointType>>();
  // 调整新点云的大小，以匹配原始扫描的点数
  deskewed_scan_->points.resize(this->original_scan->points.size());

  // individual point timestamps should be relative to this time
  // sweep_ref_time = rclcpp::Time(this->scan_header_stamp).seconds();
  // 计算相对于此时间的每个点的时间戳
  sweep_ref_time = this->scan_header_stamp.toSec();
  // ROS_INFO("scan_header_stamp : %f", this->scan_header_stamp.toSec());

  // sort points by timestamp and build list of timestamps
  // 定义三个函数对象用于后续处理
  std::function<bool(const PointType&, const PointType&)> point_time_cmp;
  std::function<bool(boost::range::index_value<PointType&, long>,
                     boost::range::index_value<PointType&, long>)>
      point_time_neq;
  std::function<double(boost::range::index_value<PointType&, long>)>
      extract_point_time;

  // 调用函数设置上述函数对象
  pointTimeCallback(point_time_cmp, point_time_neq, extract_point_time);
  // copy points into deskewed_scan_ in order of timestamp

  // 根据时间戳顺序复制点到新的点云中
  std::partial_sort_copy(this->original_scan->points.begin(),
                         this->original_scan->points.end(),
                         deskewed_scan_->points.begin(),
                         deskewed_scan_->points.end(), point_time_cmp);

  // filter unique timestamps
  // 过滤出具有唯一时间戳的点
  auto points_unique_timestamps =
      deskewed_scan_->points | boost::adaptors::indexed() |
      boost::adaptors::adjacent_filtered(point_time_neq);

  // extract timestamps from points and put them in their own list
  // 从点中提取时间戳，并将它们存入一个列表中
  std::vector<double> timestamps;
  std::vector<int> unique_time_indices;
  for (auto it = points_unique_timestamps.begin();
       it != points_unique_timestamps.end(); it++) {
    timestamps.push_back(extract_point_time(*it));
    unique_time_indices.push_back(it->index());
  }
  unique_time_indices.push_back(deskewed_scan_->points.size());

  // 计算中位点的索引，并设置扫描时间戳
  int median_pt_index = timestamps.size() / 2;
  this->scan_stamp = timestamps[median_pt_index];  // set this->scan_stamp to the timestamp of
                                    // the median point

  // don't process scans until IMU data is present
  // 如果没有有效的IMU数据，则不处理扫描
  if (!this->first_valid_scan) {
    if (this->imu_buffer.empty() || this->scan_stamp <= this->imu_buffer.back().stamp) {
      if (this->imu_buffer.empty()) {
        ROS_INFO("empty");
      }
      if (this->scan_stamp <= this->imu_buffer.back().stamp) {
        ROS_INFO("low");
      }
      // ROS_INFO("scan_stamp:%f, imu:%f", this->scan_stamp, this->imu_buffer.back().stamp);
      ROS_WARN_ONCE("Waiting First Scan Valid....");
      //RCLCPP_WARN_ONCE(get_logger(), "Waiting First Scan Valid....");
      return;
    }

    // RCLCPP_INFO(get_logger(), "First Scan is Valid");
    ROS_INFO("First Scan is Valid");
    this->first_valid_scan = true;
    this->T_prior = this->T;  // assume no motion for the first scan
    pcl::transformPointCloud(*deskewed_scan_, *deskewed_scan_,
                             this->T_prior * this->extrinsics.baselink2lidar_T);
    this->deskewed_scan = deskewed_scan_;
    this->deskew_status = true;
    return;
  }
  // 计算扫描时间差
  scan_dt = this->scan_stamp - this->prev_scan_stamp;
  if (scan_dt < 1e-6) {
    return;
  }

  // IMU prior & deskewing for second scan onwards
  // 对第二次及之后的扫描进行IMU先验和去斜处理
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
      frames;
  frames = this->integrateImu(this->prev_scan_stamp, this->lidarPose.q,
                              this->lidarPose.p,
                              this->geo.prev_vel.cast<float>(), timestamps);

  // if there are no frames between the start and end of the sweep
  // that probably means that there's a sync issue
  if (frames.size() != timestamps.size()) {
    
    // RCLCPP_FATAL(this->get_logger(),"Bad time sync between LiDAR and IMU! frames: %d, timestamp: %d",frames.size(), timestamps.size());
    ROS_FATAL("Bad time sync between LiDAR and IMU! frames: %ld, timestamp: %ld", frames.size(), timestamps.size());
    this->T_prior = this->T;
    pcl::transformPointCloud(*deskewed_scan_, *deskewed_scan_,
                             this->T_prior * this->extrinsics.baselink2lidar_T);
    this->deskewed_scan = deskewed_scan_;
    this->deskew_status = false;
    return;
  }

  // update prior to be the estimated pose at the median time of the scan
  // (corresponds to this->scan_stamp)
  // 更新先验为扫描中位时间的估计姿态
  this->T_prior = frames[median_pt_index];

// 并行处理每个时间戳对应的点
#pragma omp parallel for num_threads(this->num_threads_)
  for (unsigned int i = 0; i < timestamps.size(); i++) {
    Eigen::Matrix4f T = frames[i] * this->extrinsics.baselink2lidar_T;
    // transform point to world frame
    for (int k = unique_time_indices[i]; k < unique_time_indices[i + 1]; k++) {
      auto& pt = deskewed_scan_->points[k];
      pt.getVector4fMap()[3] = 1.;
      pt.getVector4fMap() = T * pt.getVector4fMap();
    }
  }
  // 更新去斜后的点云和状态
  this->deskewed_scan = deskewed_scan_;
  this->deskew_status = true;
}

void dlio::OdomNode::computeMetrics() {
  this->computeSpaciousness();
  this->computeDensity();
}

void dlio::OdomNode::computeSpaciousness() {
  std::vector<float> ds;

  for (unsigned int i = 0; i <= this->original_scan->points.size(); i++) {
    float d = std::sqrt(pow(this->original_scan->points[i].x, 2) +
                        pow(this->original_scan->points[i].y, 2));
    ds.push_back(d);
  }

  // median
  std::nth_element(ds.begin(), ds.begin() + ds.size() / 2, ds.end());
  float median_curr = ds[ds.size() / 2];
  static float median_prev = median_curr;
  float median_lpf = 0.95 * median_prev + 0.05 * median_curr;
  median_prev = median_lpf;

  // push
  this->metrics.spaciousness.push_back(median_lpf);
}

void dlio::OdomNode::computeDensity() {
  float density;

  if (!this->geo.first_opt_done) {
    density = 0.;
  } else {
    density = this->gicp.source_density_;
  }

  static float density_prev = density;
  float density_lpf = 0.95 * density_prev + 0.05 * density;
  density_prev = density_lpf;

  this->metrics.density.push_back(density_lpf);
}

// 从变换矩阵T中提取位姿 位置和旋转
void dlio::OdomNode::propagateGICP() {
  this->lidarPose.p << this->T(0, 3), this->T(1, 3), this->T(2, 3);

  Eigen::Matrix3f rotSO3;
  rotSO3 << this->T(0, 0), this->T(0, 1), this->T(0, 2), this->T(1, 0),
      this->T(1, 1), this->T(1, 2), this->T(2, 0), this->T(2, 1), this->T(2, 2);

  Eigen::Quaternionf q(rotSO3);

  // Normalize quaternion
  double norm =
      sqrt(q.w() * q.w() + q.x() * q.x() + q.y() * q.y() + q.z() * q.z());
  q.w() /= norm;
  q.x() /= norm;
  q.y() /= norm;
  q.z() /= norm;
  this->lidarPose.q = q;
}

void dlio::OdomNode::updateState() {
  // Lock thread to prevent state from being accessed by PropagateState
  std::lock_guard<std::mutex> lock(this->geo.mtx);

  Eigen::Vector3f pin = this->lidarPose.p;
  Eigen::Quaternionf qin = this->lidarPose.q;
  double dt = scan_dt;  // this->scan_stamp - this->prev_scan_stamp;
  if (dt < 1e-6) {
    ROS_ERROR("scan dt is invalid: %f", dt);
    return;
  }
  // if (dt > 0.065) {
  //   ROS_INFO("scan dt is too large: %f", dt);
  //   dt = 0.05;
  // }
  Eigen::Quaternionf qe, qhat, qcorr;
  qhat = this->state.q;

  // Constuct error quaternion
  qe = qhat.conjugate() * qin;

  double sgn = 1.;
  if (qe.w() < 0) {
    sgn = -1;
  }

  // Construct quaternion correction
  qcorr.w() = 1 - abs(qe.w());
  qcorr.vec() = sgn * qe.vec();
  qcorr = qhat * qcorr;

  Eigen::Vector3f err = pin - this->state.p;
  Eigen::Vector3f err_body;

  err_body = qhat.conjugate()._transformVector(err);

  double abias_max = this->geo_abias_max_;
  double gbias_max = this->geo_gbias_max_;
  //yyyjf new callback func
  // Update accel bias
  this->state.b.accel -= dt * this->geo_Kab_ * err_body;
  this->state.b.accel = this->state.b.accel.array().min(abias_max).max(-abias_max);

  // Update gyro bias
  this->state.b.gyro[0] -= dt * this->geo_Kgb_ * qe.w() * qe.x();
  this->state.b.gyro[1] -= dt * this->geo_Kgb_ * qe.w() * qe.y();
  this->state.b.gyro[2] -= dt * this->geo_Kgb_ * qe.w() * qe.z();
  this->state.b.gyro = this->state.b.gyro.array().min(gbias_max).max(-gbias_max);

  // Update state
  this->state.p += dt * this->geo_Kp_ * err;
  this->state.v.lin.w += dt * this->geo_Kv_ * err;
  this->state.q.w() += dt * this->geo_Kq_ * qcorr.w();
  this->state.q.x() += dt * this->geo_Kq_ * qcorr.x();
  this->state.q.y() += dt * this->geo_Kq_ * qcorr.y();
  this->state.q.z() += dt * this->geo_Kq_ * qcorr.z();
  this->state.q.normalize();
  
  // @yjf
  // 发布每次地图匹配后的位姿
  geometry_msgs::PoseStamped update_pose_msg;
  // 最新imubuffer时间
  update_pose_msg.header.stamp = ros::Time(this->imu_buffer.front().stamp);
  update_pose_msg.header.frame_id = "map_11";
  // pose
  update_pose_msg.pose.position.x = this->state.p[0];
  update_pose_msg.pose.position.y = this->state.p[1];
  update_pose_msg.pose.position.z = this->state.p[2];
  // qua
  update_pose_msg.pose.orientation.x = this->state.q.x();
  update_pose_msg.pose.orientation.y = this->state.q.y();
  update_pose_msg.pose.orientation.z = this->state.q.z();
  update_pose_msg.pose.orientation.w = this->state.q.w();
  gicp_pose_pub.publish(update_pose_msg);

  // store previous pose, orientation, and velocity
  this->geo.prev_p = this->state.p;
  this->geo.prev_q = this->state.q;
  this->geo.prev_vel = this->state.v.lin.w;

  this->geo.first_opt_done = true;
}




void dlio::OdomNode::getNextPose() {
  pcl::PointCloud<PointType>::Ptr aligned =
  pcl::make_shared<pcl::PointCloud<PointType>>();
  this->gicp.align(*aligned);
  aligned->header.frame_id = this->global_frame_id_;
  // hv::async(std::bind(&OdomNode::publishCloud, this, aligned));
  this->T_corr = this->gicp.getFinalTransformation();
  // this->T = this->T_corr * this->T_prior;

  // rclcpp::Time end = this->now();
  // rclcpp::Duration duration = end - start;
  // RCLCPP_INFO(this->get_logger(), "*****Time interval: %.6f s",
  //             duration.seconds());
  this->gicp_hasConverged = this->gicp.hasConverged();

  // //在这里添加发布对齐之后的点云
  if (!this->gicp_hasConverged) {
    ROS_WARN("The registration didn't converge.");
    // return;
    this->T = this->T_corr_prev * this->T_prior;
  } else {
    this->geo.first_opt_done = true;
    this->T = this->T_corr * this->T_prior;
    this->T_corr_prev = this->T_corr;
    //  RCLCPP_INFO(get_logger(),"after lidar propogate: x %.8f, y %.8f, z
    //  %.8f",
    //     this->T(0,3), this->T(1,3), this->T(2,3));
  }

  this->propagateGICP();

  this->updateState();
}
