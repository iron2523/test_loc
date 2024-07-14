#include <dlio/odom.h>

void dlio::OdomNode::declare_global_param() {
    use_pcd_map_ = false;
    declare_param(&nh, "use_pcd_map", use_pcd_map_, false);
    map_path_ = "/home/xc/test_loc_ws/src/lio_loc/map/GlobalMap.pcd";
    declare_param(&nh, "map_path", map_path_, map_path_);
    full_map_path_ = "/home/xc/test_loc_ws/src/lio_loc/map/FullMap.pcd";
    declare_param(&nh, "map_path", full_map_path_, full_map_path_);
    init_pose_path_ = "/home/xc/test_loc_ws/src/lio_loc/map/init_pose.yaml";
    declare_param(&nh, "init_pose_path", init_pose_path_, init_pose_path_);
    set_initial_pose_ = false;
    
    declare_param(&nh, "set_initial_pose", set_initial_pose_, set_initial_pose_);
    initial_pose_x_ = 0.0;
    declare_param(&nh, "initial_pose_x", initial_pose_x_, initial_pose_x_);
    initial_pose_y_ = 0.0;
    declare_param(&nh, "initial_pose_y", initial_pose_y_, initial_pose_y_);
    initial_pose_z_ = 0.0;
    declare_param(&nh, "initial_pose_z", initial_pose_z_, initial_pose_z_);
    initial_pose_yaw_ = 0.0;
    declare_param(&nh, "initial_pose_yaw", initial_pose_yaw_, initial_pose_yaw_);
    initial_pose_roll_ = 0.0;
    declare_param(&nh, "initial_pose_roll", initial_pose_roll_,
                  initial_pose_roll_);
    initial_pose_pitch_ = 0.0;
    declare_param(&nh, "initial_pose_pitch", initial_pose_pitch_,
                  initial_pose_pitch_);
    enable_debug_ = false;
    declare_param(&nh, "enable_debug", enable_debug_, enable_debug_);
    use_odom_ = false;
    declare_param(&nh, "use_odom", use_odom_, use_odom_);
    std::string version = "0.0.0";
    declare_param(&nh, "version", version, version);
}

void dlio::OdomNode::declare_odom_param() {
    // Deskew Flag
    this->deskew_ = true;
    declare_param(&nh, "pointcloud_deskew", deskew_, deskew_);
  
    // Dense map resolution
    this->densemap_filtered_ = true;
    declare_param(&nh, "map_dense_filtered", densemap_filtered_,
                  densemap_filtered_);
  
    // Wait until movement to publish map
    this->wait_until_move_ = false;
    declare_param(&nh, "map_waitUntilMove", wait_until_move_, wait_until_move_);
  
    // Adaptive Parameters
    this->adaptive_params_ = true;
    declare_param(&nh, "adaptive", adaptive_params_, adaptive_params_);
    // Gravity
    gravity_ = 9.80665;
    declare_param(&nh, "odom_gravity", gravity_, gravity_);
    // Keyframe Threshold
    keyframe_thresh_dist_ = 0.1;
    declare_param(&nh, "odom_keyframe_threshD", keyframe_thresh_dist_,
                  keyframe_thresh_dist_);
    keyframe_thresh_rot_ = 1.0;
    declare_param(&nh, "odom_keyframe_threshR", keyframe_thresh_rot_,
                  keyframe_thresh_rot_);
  
    // Submap
    submap_knn_ = 10;
    declare_param(&nh, "odom_submap_keyframe_knn", submap_knn_, submap_knn_);
    submap_kcv_ = 10;
    declare_param(&nh, "odom_submap_keyframe_kcv", submap_kcv_, submap_kcv_);
    submap_kcc_ = 10;
    declare_param(&nh, "odom_submap_keyframe_kcc", submap_kcc_, submap_kcc_);
  
    // Crop Box Filter
    crop_size_ = 1.0;
    declare_param(&nh, "odom_preprocessing_cropBoxFilter_size", crop_size_,
                  crop_size_);
  
    // Voxel Grid Filter
    vf_use_ = true;
    declare_param(&nh, "pointcloud_voxelize", vf_use_, vf_use_);
    vf_res_ = 0.05;
    declare_param(&nh, "odom_preprocessing_voxelFilter_res", vf_res_, vf_res_);
  
    // GICP
    // 两个点云之间必须找到的最小点数
    this->gicp_min_num_points_ = 100;
    declare_param(&nh, "odom_gicp_minNumPoints", this->gicp_min_num_points_,
                  this->gicp_min_num_points_);
    // 算法中每个点寻找对应点的最大数量。这个参数限制了算法在计算点对应关系时考虑的最大邻居点数，有助于平衡计算复杂度和配准精度              
    this->gicp_k_correspondences_ = 20;
    declare_param(&nh, "odom_gicp_kCorrespondences",
                  this->gicp_k_correspondences_, this->gicp_k_correspondences_);
    // 中点对应的最大距离。如果两个点之间的距离超过这个阈值，它们将不会被视为对应点
    this->gicp_max_corr_dist_ = std::sqrt(std::numeric_limits<double>::max());
    declare_param(&nh, "odom_gicp_maxCorrespondenceDistance",
                  gicp_max_corr_dist_, gicp_max_corr_dist_);
    // 法的最大迭代次数
    this->gicp_max_iter_ = 64;
    declare_param(&nh, "odom_gicp_maxIterations", this->gicp_max_iter_,
                  this->gicp_max_iter_);
    // 当连续两次迭代之间的变换小于这个值时，算法停止
    gicp_transformation_ep_ = 0.0005;
    declare_param(&nh, "odom_gicp_transformationEpsilon",
                  gicp_transformation_ep_, gicp_transformation_ep_);
    // 类似于变换阈值
    gicp_rotation_ep_ = 0.0005;
    declare_param(&nh, "odom_gicp_rotationEpsilon", gicp_rotation_ep_,
                  gicp_rotation_ep_);
    // GICP算法初始lambda因子。这个参数可能影响算法的优化过程，特别是在处理高度非线性问题时
    gicp_init_lambda_factor_ = 1e-9;
    declare_param(&nh, "odom_gicp_initLambdaFactor", gicp_init_lambda_factor_,
                  gicp_init_lambda_factor_);

    // Geometric Observer
    geo_Kp_ = 1.0;
    declare_param(&nh, "odom_geo_Kp", geo_Kp_, geo_Kp_);
    geo_Kv_ = 1.0;
    declare_param(&nh, "odom_geo_Kv", geo_Kv_, geo_Kv_);
    geo_Kq_ = 1.0;
    declare_param(&nh, "odom_geo_Kq", geo_Kq_, geo_Kq_);
    geo_Kab_ = 1.0;
    declare_param(&nh, "odom_geo_Kab", geo_Kab_, geo_Kab_);
    geo_Kgb_ = 1.0;
    declare_param(&nh, "odom_geo_Kgb", geo_Kgb_, geo_Kgb_);
    geo_abias_max_ = 1.0;
    declare_param(&nh, "odom_geo_abias_max", geo_abias_max_, geo_abias_max_);
    geo_gbias_max_ = 1.0;
    declare_param(&nh, "odom_geo_gbias_max", geo_gbias_max_, geo_gbias_max_);
}

void dlio::OdomNode::declare_extrinsics_param() {
    // Extrinsics
    std::vector<double> baselink2imu_t{0., 0., 0.},
        baselink2imu_R{1., 0., 0., 0., 1., 0., 0., 0., 1.};
    declare_param_vector(&nh, "extrinsics_baselink2imu_t", baselink2imu_t,
                         baselink2imu_t);
    declare_param_vector(&nh, "extrinsics_baselink2imu_R", baselink2imu_R,
                         baselink2imu_R);
  
    this->extrinsics.baselink2imu.t =
        Eigen::Vector3f(baselink2imu_t[0], baselink2imu_t[1], baselink2imu_t[2]);
    this->extrinsics.baselink2imu
        .R = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(
        std::vector<float>(baselink2imu_R.begin(), baselink2imu_R.end()).data(),
        3, 3);
  
    this->extrinsics.baselink2imu_T = Eigen::Matrix4f::Identity();
    this->extrinsics.baselink2imu_T.block(0, 3, 3, 1) =
        this->extrinsics.baselink2imu.t;
    this->extrinsics.baselink2imu_T.block(0, 0, 3, 3) =
        this->extrinsics.baselink2imu.R;
  
    std::vector<double> baselink2lidar_t{0., 0., 0.},
        baselink2lidar_R{1., 0., 0., 0., 1., 0., 0., 0., 1.};
  
    declare_param_vector(&nh, "extrinsics_baselink2lidar_t", baselink2lidar_t,
                         baselink2lidar_t);
    declare_param_vector(&nh, "extrinsics_baselink2lidar_R", baselink2lidar_R,
                         baselink2lidar_R);
    this->extrinsics.baselink2lidar.t = Eigen::Vector3f(
        baselink2lidar_t[0], baselink2lidar_t[1], baselink2lidar_t[2]);
    this->extrinsics.baselink2lidar.R =
        Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(
            std::vector<float>(baselink2lidar_R.begin(), baselink2lidar_R.end())
                .data(),
            3, 3);
    this->extrinsics.baselink2lidar_T = Eigen::Matrix4f::Identity();
    this->extrinsics.baselink2lidar_T.block(0, 3, 3, 1) = this->extrinsics.baselink2lidar.t;
    this->extrinsics.baselink2lidar_T.block(0, 0, 3, 3) = this->extrinsics.baselink2lidar.R;
}

void dlio::OdomNode::declare_imu_param() {
    // IMU
    calibrate_accel_ = true;
    declare_param(&nh, "odom_imu_calibration_accel", calibrate_accel_,
                  calibrate_accel_);
    calibrate_gyro_ = true;
    declare_param(&nh, "odom_imu_calibration_gyro", calibrate_gyro_,
                  calibrate_gyro_);
    imu_calib_time_ = 3.0;
    declare_param(&nh, "odom_imu_calibration_time", imu_calib_time_,
                  imu_calib_time_);
    imu_buffer_size_ = 2000;
    declare_param(&nh, "odom_imu_bufferSize", imu_buffer_size_,
                  imu_buffer_size_);
  
    std::vector<double> prior_accel_bias{0., 0., 0.};
    std::vector<double> prior_gyro_bias{0., 0., 0.};
  
    imu_calibrate_ = true;
    declare_param(&nh, "imu_calibration", imu_calibrate_, imu_calibrate_);
    declare_param_vector(&nh, "imu_intrinsics_accel_bias", prior_accel_bias,
                         prior_accel_bias);
    declare_param_vector(&nh, "imu_intrinsics_gyro_bias", prior_gyro_bias,
                         prior_gyro_bias);
  
    // scale-misalignment matrix
    std::vector<double> imu_sm{1., 0., 0., 0., 1., 0., 0., 0., 1.};
    declare_param_vector(&nh, "imu_intrinsics_accel_sm", imu_sm, imu_sm);
  
    if (!this->imu_calibrate_) {
        this->state.b.accel[0] = prior_accel_bias[0];
        this->state.b.accel[1] = prior_accel_bias[1];
        this->state.b.accel[2] = prior_accel_bias[2];
        this->state.b.gyro[0] = prior_gyro_bias[0];
        this->state.b.gyro[1] = prior_gyro_bias[1];
        this->state.b.gyro[2] = prior_gyro_bias[2];
        this->imu_accel_sm_ =
          Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(
              std::vector<float>(imu_sm.begin(), imu_sm.end()).data(), 3, 3);
    } else {
        this->state.b.accel = Eigen::Vector3f(0., 0., 0.);
        this->state.b.gyro = Eigen::Vector3f(0., 0., 0.);
        this->imu_accel_sm_ = Eigen::Matrix3f::Identity();
    }
}

void dlio::OdomNode::declare_frame_param() {
      // Frames
    global_frame_id_ = "map_1";
    declare_param(&nh, "frames_map", global_frame_id_, global_frame_id_);
    full_global_frame_id_ = "map_2";
    declare_param(&nh, "frames_map", full_global_frame_id_, full_global_frame_id_);
    odom_frame_id_ = "odom";
    declare_param(&nh, "frames_odom", odom_frame_id_, odom_frame_id_);
    base_frame_id_ = "base_link";
    declare_param(&nh, "frames_baselink", base_frame_id_, base_frame_id_);
    laser_frame_id_ = "rslidar";
    declare_param(&nh, "frames_lidar", laser_frame_id_, laser_frame_id_);
    imu_frame_id_ = "imu_link";
    declare_param(&nh, "frames_imu", imu_frame_id_, imu_frame_id_);
}