#include "dlio/odom.h"

sensor_msgs::Imu::Ptr dlio::OdomNode::transformImu(const sensor_msgs::Imu::ConstPtr& imu_raw) {
    // auto imu = std::make_shared<sensor_msgs::msg::Imu>();
    // auto imu = std::make_shared<sensor_msgs::Imu>();
    sensor_msgs::Imu::Ptr imu (new sensor_msgs::Imu);
    // Copy header
    imu->header = imu_raw->header;
  
    //double imu_stamp_secs = rclcpp::Time(imu->header.stamp).seconds();
    double imu_stamp_secs = imu->header.stamp.toSec();
    static double prev_stamp = imu_stamp_secs;
    double dt = imu_stamp_secs - prev_stamp;
    prev_stamp = imu_stamp_secs;
  
    if (dt == 0) {
        dt = 1.0 / 200.0;
    }
  
    // Transform angular velocity (will be the same on a rigid body, so just
    // rotate to ROS convention)
    Eigen::Vector3f ang_vel(imu_raw->angular_velocity.x,
                            imu_raw->angular_velocity.y,
                            imu_raw->angular_velocity.z);
  
    Eigen::Vector3f ang_vel_cg = this->extrinsics.baselink2imu.R * ang_vel;
  
    imu->angular_velocity.x = ang_vel_cg[0];
    imu->angular_velocity.y = ang_vel_cg[1];
    imu->angular_velocity.z = ang_vel_cg[2];
  
    static Eigen::Vector3f ang_vel_cg_prev = ang_vel_cg;
  
    // Transform linear acceleration (need to account for component due to
    // translational difference)
    Eigen::Vector3f lin_accel(imu_raw->linear_acceleration.x,
                              imu_raw->linear_acceleration.y,
                              imu_raw->linear_acceleration.z);
  
    Eigen::Vector3f lin_accel_cg = this->extrinsics.baselink2imu.R * lin_accel;
  
    lin_accel_cg =
        lin_accel_cg +
        ((ang_vel_cg - ang_vel_cg_prev) / dt)
            .cross(-this->extrinsics.baselink2imu.t) +
        ang_vel_cg.cross(ang_vel_cg.cross(-this->extrinsics.baselink2imu.t));
  
    ang_vel_cg_prev = ang_vel_cg;
  
    imu->linear_acceleration.x = lin_accel_cg[0];
    imu->linear_acceleration.y = lin_accel_cg[1];
    imu->linear_acceleration.z = lin_accel_cg[2];
  
    return imu;
}

bool dlio::OdomNode::imuMeasFromTimeRange(
    double start_time, double end_time,
    boost::circular_buffer<ImuMeas>::reverse_iterator& begin_imu_it,
    boost::circular_buffer<ImuMeas>::reverse_iterator& end_imu_it) {
    
    // IMU数据并未完整覆盖整个扫描周期
    if (this->imu_buffer.empty() || this->imu_buffer.front().stamp < end_time) {
      // Wait for the latest IMU data
        std::unique_lock<decltype(this->mtx_imu)> lock(this->mtx_imu);
        this->cv_imu_stamp.wait(lock, [this, &end_time] {
        return this->imu_buffer.front().stamp >= end_time;
        });
    }
  
    // std::cout << "finish wait" << std::endl;
  
    auto imu_it = this->imu_buffer.begin();
  
    auto last_imu_it = imu_it;
    imu_it++;
    while (imu_it != this->imu_buffer.end() && imu_it->stamp >= end_time) {
        last_imu_it = imu_it;
        imu_it++;
    }
  
    while (imu_it != this->imu_buffer.end() && imu_it->stamp >= start_time) {
        imu_it++;
    }
  
    if (imu_it == this->imu_buffer.end()) {
        // not enough IMU measurements, return false
        return false;
    }
    imu_it++;
  
    // Set reverse iterators (to iterate forward in time)
    end_imu_it = boost::circular_buffer<ImuMeas>::reverse_iterator(last_imu_it);
    begin_imu_it = boost::circular_buffer<ImuMeas>::reverse_iterator(imu_it);
    return true;
}

void dlio::OdomNode::getScanFromROS(sensor_msgs::PointCloud2ConstPtr pc) {
    // ROS_INFO("1");
    pcl::PointCloud<PointType>::Ptr original_scan_ =
    pcl::make_shared<pcl::PointCloud<PointType>>();
    // ROS_INFO("2");

    pcl::fromROSMsg(*pc, *original_scan_);
    // Remove NaNs

    std::vector<int> idx;
    original_scan_->is_dense = false;
    // ROS_INFO("3");

    pcl::removeNaNFromPointCloud(*original_scan_, *original_scan_, idx);
    // ROS_INFO("4");

    // Crop Box Filter
    this->crop.setInputCloud(original_scan_);
    // ROS_INFO("5");

    this->crop.filter(*original_scan_);

    // automatically detect sensor type
    // ROS_INFO("6");

    this->sensor = SensorType::UNKNOWN;
    for (auto& field : pc->fields) {
        // ROS_INFO("%s", field.name.c_str());
        if (field.name == "t") {
            this->sensor = SensorType::OUSTER;
            break;
        } else if (field.name == "time") {
            this->sensor = SensorType::VELODYNE;
            break;
        } else if (field.name == "timestamp") {
            this->sensor = SensorType::HESAI;
            // std::cout << "HESAI" << std::endl;
            // ROS_INFO("hesai");
            break;
        } 
    }

    this->scan_header_stamp = pc->header.stamp;
    this->original_scan = original_scan_;
}

std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
    dlio::OdomNode::integrateImuInternal(
    Eigen::Quaternionf q_init, Eigen::Vector3f p_init, Eigen::Vector3f v_init,
    const std::vector<double>& sorted_timestamps,
    boost::circular_buffer<ImuMeas>::reverse_iterator begin_imu_it,
    boost::circular_buffer<ImuMeas>::reverse_iterator end_imu_it) {
    
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>imu_se3;

    // Initialization
    Eigen::Quaternionf q = q_init;
    Eigen::Vector3f p = p_init;
    Eigen::Vector3f v = v_init;
    Eigen::Vector3f a = q._transformVector(begin_imu_it->lin_accel);
    a[2] -= this->gravity_;

    // Iterate over IMU measurements and timestamps
    auto prev_imu_it = begin_imu_it;
    auto imu_it = prev_imu_it + 1;

    auto stamp_it = sorted_timestamps.begin();

    for (; imu_it != end_imu_it; imu_it++) {
        const ImuMeas& f0 = *prev_imu_it;
        const ImuMeas& f = *imu_it;

        // Time between IMU samples
        double dt = f.dt;

        // Angular acceleration
        Eigen::Vector3f alpha_dt = f.ang_vel - f0.ang_vel;
        Eigen::Vector3f alpha = alpha_dt / dt;

        // Average angular velocity
        Eigen::Vector3f omega = f0.ang_vel + 0.5 * alpha_dt;

        // Orientation
        q = Eigen::Quaternionf(
            q.w() - 0.5 * (q.x() * omega[0] + q.y() * omega[1] + q.z() * omega[2]) * dt,
            q.x() + 0.5 * (q.w() * omega[0] - q.z() * omega[1] + q.y() * omega[2]) * dt,
            q.y() + 0.5 * (q.z() * omega[0] + q.w() * omega[1] - q.x() * omega[2]) * dt,
            q.z() + 0.5 * (q.x() * omega[1] - q.y() * omega[0] + q.w() * omega[2]) * dt);
        q.normalize();

        // Acceleration
        Eigen::Vector3f a0 = a;
        a = q._transformVector(f.lin_accel);
        a[2] -= this->gravity_;

        // Jerk
        Eigen::Vector3f j_dt = a - a0;
        Eigen::Vector3f j = j_dt / dt;

        // Interpolate for given timestamps
        while (stamp_it != sorted_timestamps.end() && *stamp_it <= f.stamp) {
            // Time between previous IMU sample and given timestamp
            double idt = *stamp_it - f0.stamp;

            // Average angular velocity
            Eigen::Vector3f omega_i = f0.ang_vel + 0.5 * alpha * idt;

            // Orientation
            Eigen::Quaternionf q_i(
                q.w() - 0.5 * (q.x() * omega_i[0] + q.y() * omega_i[1] + q.z() * omega_i[2]) * idt,
                q.x() + 0.5 * (q.w() * omega_i[0] - q.z() * omega_i[1] + q.y() * omega_i[2]) * idt,
                q.y() + 0.5 * (q.z() * omega_i[0] + q.w() * omega_i[1] - q.x() * omega_i[2]) * idt,
                q.z() + 0.5 * (q.x() * omega_i[1] - q.y() * omega_i[0] + q.w() * omega_i[2]) * idt);
            q_i.normalize();

            // Position
            Eigen::Vector3f p_i =
                p + v * idt + 0.5 * a0 * idt * idt + (1 / 6.) * j * idt * idt * idt;

            // Transformation
            Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
            T.block(0, 0, 3, 3) = q_i.toRotationMatrix();
            T.block(0, 3, 3, 1) = p_i;

            imu_se3.push_back(T);

            stamp_it++;
        }

        // Position
        p += v * dt + 0.5 * a0 * dt * dt + (1 / 6.) * j_dt * dt * dt;

        // Velocity
        v += a0 * dt + 0.5 * j_dt * dt;

        prev_imu_it = imu_it;
    }

    return imu_se3;
}

std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
    dlio::OdomNode::integrateImu(double start_time, // 开始时间
    Eigen::Quaternionf q_init, // 初始方向的四元数
    Eigen::Vector3f p_init, // 初始位置
    Eigen::Vector3f v_init, // 初始速度
    const std::vector<double>& sorted_timestamps) {

    // std::cout << "enter integrateImu..." << std::endl;
    // 初始化一个空的变换矩阵向量，用于存储计算结果
    const std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
        empty;

    // 检查输入的时间戳序列是否为空，或开始时间是否超出范围，若是，则返回空向量
    if (sorted_timestamps.empty() || start_time > sorted_timestamps.front()) {
        // invalid input, return empty vector
        // std::cout << "invalid input" << std::endl;
        ROS_INFO("invalid input, prev scan time: %.8f, point time.front: %.8f",
            start_time, sorted_timestamps.front());
        return empty;
    }

    // 定义用于存储从开始时间到结束时间内的IMU测量数据的迭代器
    boost::circular_buffer<ImuMeas>::reverse_iterator begin_imu_it;
    boost::circular_buffer<ImuMeas>::reverse_iterator end_imu_it;
    // 获取指定时间范围内的IMU测量数据
    if (this->imuMeasFromTimeRange(start_time, sorted_timestamps.back(),
                                   begin_imu_it, end_imu_it) == false) {
        // not enough IMU measurements, return empty vector
        std::cout << "not enough IMU measurements" << std::endl;
        return empty;
    }

    // std::cout << "aft ImuMeas..." << std::endl;
    // Backwards integration to find pose at first IMU sample
    // 反向积分以找到第一个IMU样本的位姿
    const ImuMeas& f1 = *begin_imu_it;
    const ImuMeas& f2 = *(begin_imu_it + 1);

    // std::cout << "first imu" << f1.lin_accel << std::endl;
    // std::cout << "second imu" << f2.lin_accel << std::endl;

    // Time between first two IMU samples
    // 计算第一对IMU样本之间的时间差
    double dt = f2.dt;

    // Time between first IMU sample and start_time
    // 计算第一个IMU样本与开始时间之间的时间差
    double idt = start_time - f1.stamp;

    // Angular acceleration between first two IMU samples
    // 计算第一对IMU样本之间的角速度差，用以估计角加速度
    Eigen::Vector3f alpha_dt = f2.ang_vel - f1.ang_vel;
    Eigen::Vector3f alpha = alpha_dt / dt;

    // Average angular velocity (reversed) between first IMU sample and start_time
    // 计算从开始时间到第一个IMU样本之间的平均角速度（反向）
    Eigen::Vector3f omega_i = -(f1.ang_vel + 0.5 * alpha * idt);

    // Set q_init to orientation at first IMU sample
    // 根据角速度更新初始方向的四元数
    q_init = Eigen::Quaternionf(
        q_init.w() - 0.5 *
                         (q_init.x() * omega_i[0] + q_init.y() * omega_i[1] +
                          q_init.z() * omega_i[2]) *
                         idt,
        q_init.x() + 0.5 *
                         (q_init.w() * omega_i[0] - q_init.z() * omega_i[1] +
                          q_init.y() * omega_i[2]) *
                         idt,
        q_init.y() + 0.5 *
                         (q_init.z() * omega_i[0] + q_init.w() * omega_i[1] -
                          q_init.x() * omega_i[2]) *
                         idt,
        q_init.z() + 0.5 *
                         (q_init.x() * omega_i[1] - q_init.y() * omega_i[0] +
                          q_init.w() * omega_i[2]) *
                         idt);
    q_init.normalize(); // 归一化

    // Average angular velocity between first two IMU samples
    // 计算第一对IMU样本之间的平均角速度
    Eigen::Vector3f omega = f1.ang_vel + 0.5 * alpha_dt;

    // Orientation at second IMU sample
    // 更新到第二个IMU样本的方向
    Eigen::Quaternionf q2(
        q_init.w() - 0.5 *
                         (q_init.x() * omega[0] + q_init.y() * omega[1] +
                          q_init.z() * omega[2]) *
                         dt,
        q_init.x() + 0.5 *
                         (q_init.w() * omega[0] - q_init.z() * omega[1] +
                          q_init.y() * omega[2]) *
                         dt,
        q_init.y() + 0.5 *
                         (q_init.z() * omega[0] + q_init.w() * omega[1] -
                          q_init.x() * omega[2]) *
                         dt,
        q_init.z() + 0.5 *
                         (q_init.x() * omega[1] - q_init.y() * omega[0] +
                          q_init.w() * omega[2]) *
                         dt);
    q2.normalize();

    // Acceleration at first IMU sample
    // 计算第一个IMU样本的线加速度
    Eigen::Vector3f a1 = q_init._transformVector(f1.lin_accel);
    a1[2] -= this->gravity_;

    // Acceleration at second IMU sample
    // 计算第二个IMU样本的线加速度
    Eigen::Vector3f a2 = q2._transformVector(f2.lin_accel);
    a2[2] -= this->gravity_;

    // Jerk between first two IMU samples
    Eigen::Vector3f j = (a2 - a1) / dt;

    // Set v_init to velocity at first IMU sample (go backwards from start_time)
    v_init -= a1 * idt + 0.5 * j * idt * idt;

    // std::cout << "v_init" << v_init << std::endl;

    // Set p_init to position at first IMU sample (go backwards from start_time)
    // 根据线加速度和加速度变化率更新初始位置
    p_init -=
        v_init * idt + 0.5 * a1 * idt * idt + (1 / 6.) * j * idt * idt * idt;
    // std::cout << "p_init" << v_init << std::endl;

    // std::cout << "bf integrateImuInternal" << std::endl;
    // 调用内部函数，继续进行IMU数据的积分处理，返回每个时间戳对应的位姿变换矩阵
    return this->integrateImuInternal(q_init, p_init, v_init, sorted_timestamps,
                                    begin_imu_it, end_imu_it);


}

void dlio::OdomNode::pointTimeCallback(
    std::function<bool(const PointType&, const PointType&)>& point_time_cmp,
    std::function<bool(boost::range::index_value<PointType&, long>,
    boost::range::index_value<PointType&, long>)>&point_time_neq,
    std::function<double(boost::range::index_value<PointType&, long>)>&extract_point_time) {

    if (this->sensor == SensorType::OUSTER) {
        point_time_cmp = [](const PointType& p1, const PointType& p2) {
            return p1.t < p2.t;
        };
        point_time_neq = [](boost::range::index_value<PointType&, long> p1,
                            boost::range::index_value<PointType&, long> p2) {
            return p1.value().t != p2.value().t;
        };
        extract_point_time = [&](boost::range::index_value<PointType&, long> pt) {
            return sweep_ref_time + pt.value().t * 1e-9f;
        };

    } else if (this->sensor == SensorType::VELODYNE) {
        point_time_cmp = [](const PointType& p1, const PointType& p2) {
            return p1.time < p2.time;
        };
        point_time_neq = [](boost::range::index_value<PointType&, long> p1,
                          boost::range::index_value<PointType&, long> p2) {
            return p1.value().time != p2.value().time;
        };
        extract_point_time = [&](boost::range::index_value<PointType&, long> pt) {
            return sweep_ref_time + pt.value().time;
      };

    } else if (this->sensor == SensorType::HESAI) {
        point_time_cmp = [](const PointType& p1, const PointType& p2) {
            return p1.timestamp < p2.timestamp;
        };
        point_time_neq = [](boost::range::index_value<PointType&, long> p1,
                          boost::range::index_value<PointType&, long> p2) {
            return p1.value().timestamp != p2.value().timestamp;
        };
        extract_point_time = [&](boost::range::index_value<PointType&, long> pt) {
        // return pt.value().timestamp;
        // std::cout << std::setprecision(18) << pt.value().timestamp * 1e-9f
        // << std::endl;
        // livox_mid360's offset timestamp unit is microseconds...
        // so please multiply it with 1000000000(but not 1e-9f)
        // return pt.value().timestamp / 1000000000;
        return pt.value().timestamp;
      };
    }

}

bool dlio::OdomNode::isFileExist(const std::string& filename) {
    struct stat buffer;
    if (stat(filename.c_str(), &buffer) != 0) {
        ROS_ERROR_ONCE("File [%s] is not Exist!!!", init_pose_path_.c_str());
        return false;
    }
    return true;
}

