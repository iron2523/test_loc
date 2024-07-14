#include <dlio/odom.h>
#include <dlio_loc/state_info.h>

void dlio::OdomNode::initialPoseReceived(geometry_msgs::PoseWithCovarianceStampedPtr msg) {
    ROS_INFO("initialPoseRecdived");
    if ("map_1" != this->global_frame_id_) {
        ROS_WARN("initialpose_frame_id does not match global_frame_id");
        return;
    }

    initialpose_recieved_ = true;
    geometry_msgs::PoseWithCovarianceStamped msg_pose = *msg;
    pose_pub_.publish(msg_pose);

    Eigen::Quaternionf q_init(
        msg_pose.pose.pose.orientation.w, 
        msg_pose.pose.pose.orientation.x, 
        msg_pose.pose.pose.orientation.y, 
        msg_pose.pose.pose.orientation.z);
    q_init.normalize();

    this->T_init.setIdentity();
    this->T_init.block<3, 3>(0, 0) = q_init.toRotationMatrix();
    this->T_init.block<3, 1>(0, 3) = Eigen::Vector3f(
        msg_pose.pose.pose.position.x, msg_pose.pose.pose.position.y,
        msg_pose.pose.pose.position.z);
    // pose_pub_->publish(corrent_pose_stamped_);
    
    this->first_valid_scan = false;
    //让激光的回调去执行首祯配准 直接将当前的定位结果改成这个初始位姿
    this->T = this->T_init;
    this->T_prior = this->T_init;
    this->geo.mtx.lock();
    this->lidarPose.q = Eigen::Quaternionf(this->T_init.block<3, 3>(0, 0));
    this->lidarPose.p = Eigen::Vector3f(this->T_init.block<3, 1>(0, 3));
    // this->lidarPose = this->T_init;
    //如何清空geo的状态？
    this->geo.first_opt_done = false;
    //状态重传播
    this->state.q = lidarPose.q;
    this->state.p = lidarPose.p;
    this->state.q.normalize();
    // this->state.p.v = lidarPose.p;
    this->state.v.lin.b = Eigen::Vector3f(0., 0., 0.);
    this->state.v.lin.w = Eigen::Vector3f(0., 0., 0.);
    this->state.v.ang.b = Eigen::Vector3f(0., 0., 0.);
    this->state.v.ang.w = Eigen::Vector3f(0., 0., 0.);
    this->geo.mtx.unlock();
    //速度和角速度也要乘坐标变换吧？
    // this->state.v = this->state
    this->resetImu();
}

void dlio::OdomNode::mapReceived(sensor_msgs::PointCloud2ConstPtr msg) {
    ROS_INFO("mapReceived");
    if (msg->header.frame_id != global_frame_id_) {
        ROS_WARN("map_frame_id does not match global_frame_id");
        return;
    }
    map_recieved_ = true;
}
void dlio::OdomNode::odomReceived(nav_msgs::Odometry::ConstPtr msg) {
    if (!use_odom_) {
        return;
    }
    ROS_INFO("odomReceived");
    double current_odom_received_time =
    msg->header.stamp.sec + msg->header.stamp.nsec * 1e-9;
    double dt_odom = current_odom_received_time - last_odom_received_time_;
    last_odom_received_time_ = current_odom_received_time;
    if (dt_odom > 1.0 /* [sec] */) {
        ROS_WARN("odom time interval is too large");
        return;
    }
    if (dt_odom < 0.0 /* [sec] */) {
      ROS_WARN("odom time interval is negative");
      return;
    }
  
    tf2::Quaternion previous_quat_tf;
    double roll, pitch, yaw;
    tf2::fromMsg(corrent_pose_stamped_.pose.pose.orientation, previous_quat_tf);
    tf2::Matrix3x3(previous_quat_tf).getRPY(roll, pitch, yaw);
  
    //-------因此，当前地roll,pitch,yaw是相对于开始时刻的，也就是相对于map坐标系下的
    roll += msg->twist.twist.angular.x * dt_odom;
    pitch += msg->twist.twist.angular.y * dt_odom;
    yaw += msg->twist.twist.angular.z * dt_odom;
  
    //使用欧拉角转换为四元数
    Eigen::Quaterniond quat_eig =
        Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
    
    // geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(quat_eig);
    geometry_msgs::Quaternion quat_msg = tf2::toMsg(quat_eig);
  
    Eigen::Vector3d odom{msg->twist.twist.linear.x, msg->twist.twist.linear.y,
                         msg->twist.twist.linear.z};
  
    //旋转*平移 得到平移增量的适量矢量(时间上正确吗？不应该用中值吗？)
    Eigen::Vector3d delta_position = quat_eig.matrix() * dt_odom * odom;
  
    //加上增量，得到当前的position
    corrent_pose_stamped_.pose.pose.position.x += delta_position.x();
    corrent_pose_stamped_.pose.pose.position.y += delta_position.y();
    corrent_pose_stamped_.pose.pose.position.z += delta_position.z();
    corrent_pose_stamped_.pose.pose.orientation = quat_msg;
}

void dlio::OdomNode::lioReceived(nav_msgs::Odometry odometry) {
    // 接收位置信息
    std::lock_guard<std::mutex> lock(this->geo.mtx);

    this->state.p[0] = odometry.pose.pose.position.x;
    this->state.p[1] = odometry.pose.pose.position.y;
    this->state.p[2] = odometry.pose.pose.position.z;
    
    // 接收方向信息
    this->state.q.w() = odometry.pose.pose.orientation.w;
    this->state.q.x() = odometry.pose.pose.orientation.x;
    this->state.q.y() = odometry.pose.pose.orientation.y;
    this->state.q.z() = odometry.pose.pose.orientation.z;
    this->state.q.normalize();
    
    // 接收线速度信息
    this->state.v.lin.w[0] = odometry.twist.twist.linear.x;
    this->state.v.lin.w[1] = odometry.twist.twist.linear.y;
    this->state.v.lin.w[2] = odometry.twist.twist.linear.z;
    this->geo.prev_p = this->state.p;
    this->geo.prev_q = this->state.q;
    this->geo.prev_vel = this->state.v.lin.w;
}

void dlio::OdomNode::imuReceived(sensor_msgs::Imu::ConstPtr msg) {
    //接收到IMU数据
    this->first_imu_received_ = true;

    //转换IMU数据
    sensor_msgs::Imu::Ptr imu = transformImu(msg);
    imu_stamp = imu->header.stamp;
    double imu_stamp_secs = imu->header.stamp.toSec();

    Eigen::Vector3f lin_accel;
    Eigen::Vector3f ang_vel;

    // Get IMU samples
    ang_vel[0] = imu->angular_velocity.x;
    ang_vel[1] = imu->angular_velocity.y;
    ang_vel[2] = imu->angular_velocity.z;

    // lin_accel[0] = imu->linear_acceleration.x * 9.80665;
    // lin_accel[1] = imu->linear_acceleration.y * 9.80665;
    // lin_accel[2] = imu->linear_acceleration.z * 9.80665;

    lin_accel[0] = imu->linear_acceleration.x;
    lin_accel[1] = imu->linear_acceleration.y;
    lin_accel[2] = imu->linear_acceleration.z;

    if (this->first_imu_stamp == 0.) {
        this->first_imu_stamp = imu_stamp_secs;
    }

    // IMU calibration procedure - do for three seconds
    if (!this->imu_calibrated) {
        static int num_samples = 0;
        static Eigen::Vector3f gyro_avg(0., 0., 0.);
        static Eigen::Vector3f accel_avg(0., 0., 0.);
        static bool print = true;

        if ((imu_stamp_secs - this->first_imu_stamp) < this->imu_calib_time_) {
            num_samples++;
            gyro_avg[0] += ang_vel[0];
            gyro_avg[1] += ang_vel[1];
            gyro_avg[2] += ang_vel[2];

            accel_avg[0] += lin_accel[0];
            accel_avg[1] += lin_accel[1];
            accel_avg[2] += lin_accel[2];

        if (print) {
            std::cout << std::endl
                    << " Calibrating IMU for " << this->imu_calib_time_
                    << " seconds... ";
            std::cout.flush();
            print = false;
        }

      } else {
            std::cout << "done" << std::endl << std::endl;

            gyro_avg /= num_samples;
            accel_avg /= num_samples;

            Eigen::Vector3f grav_vec(0., 0., this->gravity_);

            if (this->calibrate_accel_) {
                
            // 将重力从平均加速度中减去以得到偏差
            this->state.b.accel = accel_avg - grav_vec;

            std::cout << " Accel biases [xyz]: "
                    << to_string_with_precision(this->state.b.accel[0], 8) << ", "
                    << to_string_with_precision(this->state.b.accel[1], 8) << ", "
                    << to_string_with_precision(this->state.b.accel[2], 8)
                    << std::endl;
        }

        if (this->calibrate_gyro_) {
            this->state.b.gyro = gyro_avg;

            std::cout << " Gyro biases  [xyz]: "
                    << to_string_with_precision(this->state.b.gyro[0], 8) << ", "
                    << to_string_with_precision(this->state.b.gyro[1], 8) << ", "
                    << to_string_with_precision(this->state.b.gyro[2], 8)
                    << std::endl;
        }

        this->imu_calibrated = true;
      }

    } else {
        double dt = imu_stamp_secs - this->prev_imu_stamp;
        if (dt == 0) {
            dt = 1.0 / 200.0;
      }
        // Apply the calibrated bias to the new IMU measurements
        this->imu_meas.stamp = imu_stamp_secs;
        this->imu_meas.dt = dt;
        this->prev_imu_stamp = this->imu_meas.stamp;

        //加速度计校准
        Eigen::Vector3f lin_accel_corrected = (this->imu_accel_sm_ * lin_accel) - this->state.b.accel;

        //陀螺仪校准
        Eigen::Vector3f ang_vel_corrected = ang_vel - this->state.b.gyro;

        this->imu_meas.lin_accel = lin_accel_corrected;
        this->imu_meas.ang_vel = ang_vel_corrected;

        // Store calibrated IMU measurements into imu buffer for manual integration
        this->mtx_imu.lock();
        this->imu_buffer.push_front(this->imu_meas);

        this->mtx_imu.unlock();

        // Notify the callbackPointCloud thread that IMU data exists for this time
        this->cv_imu_stamp.notify_one();

        //可以改成位姿是否初始化完成
        if (this->geo.first_opt_done) {
          //使用每祯IMU更新状态
          this->propagateState();
        }
    }
}

void dlio::OdomNode::publishInitialMap() {
    if (this->initial_map_pub_.getNumSubscribers() == 0) return;
    
    if (this->map_msg_ptr != nullptr) {
        this->initial_map_pub_.publish(*map_msg_ptr);
    }

}

void dlio::OdomNode::publishFullMap() {
    if (this->initial_full_map_pub_ != nullptr) {
        this->initial_full_map_pub_.publish(*full_map_msg_ptr);
    }
}

void dlio::OdomNode::publishGicpPose() {
    //@yjf
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
}

// 发布state 机器人位置
void dlio::OdomNode::publishPoseThread() {
    if (!this->first_valid_scan) {
        return;
    }

    geometry_msgs::PoseWithCovarianceStamped pose_msg;
    geometry_msgs::Quaternion quat_msg;
    {
        std::lock_guard<std::mutex> lock(this->geo.mtx);
        pose_msg.header.stamp = this->imu_stamp;
        pose_msg.pose.pose.position.x = static_cast<double>(this->state.p.x());
        pose_msg.pose.pose.position.y = static_cast<double>(this->state.p.y());
        pose_msg.pose.pose.position.z = static_cast<double>(this->state.p.z());
        Eigen::Quaterniond quat_eig(this->state.q);
        quat_msg = tf2::toMsg(quat_eig);
    }
    pose_msg.pose.pose.orientation = quat_msg;
    this->pose_pub_.publish(pose_msg);
    geometry_msgs::Pose2D pose;
    pose.x = pose_msg.pose.pose.position.x;
    pose.y = pose_msg.pose.pose.position.y;
    pose.theta = tf2::getYaw(pose_msg.pose.pose.orientation);
    savePoseToServer(pose); 

    geometry_msgs::TransformStamped transform_stamped_base_link_to_map;
    transform_stamped_base_link_to_map.header.stamp = this->imu_stamp;
    transform_stamped_base_link_to_map.header.frame_id = global_frame_id_;
    transform_stamped_base_link_to_map.child_frame_id = base_frame_id_;
    transform_stamped_base_link_to_map.transform.translation.x = pose_msg.pose.pose.position.x;
    transform_stamped_base_link_to_map.transform.translation.y = pose_msg.pose.pose.position.y;
    transform_stamped_base_link_to_map.transform.translation.z = pose_msg.pose.pose.position.z;
    transform_stamped_base_link_to_map.transform.rotation = quat_msg;
    this->broadcaster_.sendTransform(transform_stamped_base_link_to_map);
}

void dlio::OdomNode::cloudData(sensor_msgs::PointCloud2ConstPtr msg) {
    latest_cloud_msg_ = msg;
    if (msg == nullptr) {
        ROS_INFO("no data!!!");
        return;
    } 
}

void dlio::OdomNode::timerCallbackCloud(const ros::WallTimerEvent&) {
    // 检查是否有点云
    if (latest_cloud_msg_ != nullptr) {

        //检查IMU是否完成初始化
        if (!this->first_imu_received_ || !this->imu_calibrated) {
            return;
        }
        //初始化第一帧雷达时间
        if (this->prev_scan_stamp == 0.) {
            this->prev_scan_stamp = latest_cloud_msg_->header.stamp.toSec();
            ROS_INFO("prev_scan_stamp: %f seconds", this->prev_scan_stamp);

            //需要直接返回，保证后续的dt为非零值
            return;
        }
        //第一帧有效雷达时间
        if (this->first_scan_stamp == 0.) {
            this->first_scan_stamp = latest_cloud_msg_->header.stamp.toSec();
            ROS_INFO("first_scan_stamp: %f seconds", first_scan_stamp);

        }
        this->getScanFromROS(latest_cloud_msg_);
        //畸变补偿
        this->deskewPointcloud();

        if (scan_dt < 1e-6) {
            return;
        }
        //是否开启体素滤波
        if (this->vf_use_) {
            pcl::PointCloud<PointType>::Ptr current_scan_ =
                pcl::make_shared<pcl::PointCloud<PointType>>(*this->deskewed_scan);
            this->voxel.setInputCloud(current_scan_);
            this->voxel.filter(*current_scan_);
            this->current_scan = current_scan_;
        } else {
            this->current_scan = this->deskewed_scan;
        }   
        //如果IMU数据始终晚于激光数据，则不进行配准
        if (!this->first_valid_scan) {
            return;
        }   
        this->setInputSource();
        this->getNextPose();

        this->prev_scan_stamp = this->scan_stamp;
    }
}

void dlio::OdomNode::cloudReceived(sensor_msgs::PointCloud2ConstPtr msg) {
    if (!map_recieved_ || !initialpose_recieved_) {
        return;
    }
    //检查IMU是否完成初始化
    if (!this->first_imu_received_ || !this->imu_calibrated) {
        return;
    }
    //初始化第一帧雷达时间
    if (this->prev_scan_stamp == 0.) {
        this->prev_scan_stamp = msg->header.stamp.toSec();
        //需要直接返回，保证后续的dt为非零值
        return;
    }
    //第一帧有效雷达时间
    if (this->first_scan_stamp == 0.) {
        this->first_scan_stamp = msg->header.stamp.toSec();
    }
    this->getScanFromROS(msg);
    //畸变补偿
    this->deskewPointcloud();
    if (scan_dt < 1e-6) {
        ROS_ERROR("scan dt is invalid: %f", scan_dt);
        return;
    }
    //是否开启体素滤波
    if (this->vf_use_) {
        pcl::PointCloud<PointType>::Ptr current_scan_ =
            pcl::make_shared<pcl::PointCloud<PointType>>(*this->deskewed_scan);
        this->voxel.setInputCloud(current_scan_);
        this->voxel.filter(*current_scan_);
        this->current_scan = current_scan_;
    } else {
        this->current_scan = this->deskewed_scan;
    }   
    //如果IMU数据始终晚于激光数据，则不进行配准
    if (!this->first_valid_scan) {
        return;
    }   
    this->setInputSource();
    this->getNextPose();
    this->prev_scan_stamp = this->scan_stamp;
}
