#include "dlio/dlio.h"
#include "dlio/utils.h"
#include "dlio_loc/state_info.h"

//ROS
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

//BOOST
#include <boost/format.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/range/adaptor/indexed.hpp>
#include <boost/range/adaptor/adjacent_filtered.hpp>

//PCL
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.h>

#include <geometry_msgs/Pose2D.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// livox
// #include "livox_ros_driver/CustomMsg.h"

#include "threadpool/hasync.h"

using namespace std::chrono_literals;

namespace dlio {

class OdomNode {
public:
  OdomNode(ros::NodeHandle node_handle, ros::NodeHandle mt_node_handle);
  ~OdomNode();

private:
  struct ImuMeas {
    double stamp;
    double dt;
    Eigen::Vector3f ang_vel;
    Eigen::Vector3f lin_accel;
  };
  ImuMeas imu_meas;
  void initialization();
  void declare_parameters();
  void declare_global_param();
  void declare_odom_param();
  void declare_extrinsics_param();
  void declare_imu_param();
  void declare_frame_param();
  void initializeParameters();
  void initializePubSub();
  void createPublishers();
  void createSubscribers();
  void loadMap();
  void loadFullMap();
  void loadPoseFromServer();
  void savePoseToServer(const geometry_msgs::Pose2D& pose);
  void setInitialPose();
  bool isFileExist(const std::string& filename);

  void pointTimeCallback(
      std::function<bool(const PointType&, const PointType&)>& point_time_cmp,
      std::function<bool(boost::range::index_value<PointType&, long>,
                         boost::range::index_value<PointType&, long>)>&
          point_time_neq,
      std::function<double(boost::range::index_value<PointType&, long>)>&
          extract_point_time);

  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
  integrateImu(double start_time, Eigen::Quaternionf q_init,
               Eigen::Vector3f p_init, Eigen::Vector3f v_init,
               const std::vector<double>& sorted_timestamps);
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
  integrateImuInternal(
      Eigen::Quaternionf q_init, Eigen::Vector3f p_init, Eigen::Vector3f v_init,
      const std::vector<double>& sorted_timestamps,
      boost::circular_buffer<ImuMeas>::reverse_iterator begin_imu_it,
      boost::circular_buffer<ImuMeas>::reverse_iterator end_imu_it);
  void initialPoseReceived(geometry_msgs::PoseWithCovarianceStampedPtr msg);
  void mapReceived(sensor_msgs::PointCloud2ConstPtr msg);
  void odomReceived(nav_msgs::Odometry::ConstPtr msg);
  void propagateState();

  void imuReceived(sensor_msgs::Imu::ConstPtr msg);
  void lioReceived(dlio_loc::state_info::ConstPtr lio_state);
  void getScanFromROS(sensor_msgs::PointCloud2ConstPtr msg);
  void deskewPointcloud();
  void setInputSource();
  void initializeInputTarget();
  void cloudReceived(sensor_msgs::PointCloud2ConstPtr msg);
  // 这里只是将最新的消息保存下来，实际处理在timerCallback中进行
  void cloudData(sensor_msgs::PointCloud2ConstPtr msg);
  void timerCallbackCloud(const ros::WallTimerEvent&);

  // void gnssReceived();
  void computeSpaciousness();
  void computeDensity();
  void computeMetrics();
  void propagateGICP();
  void updateState();
  void resetImu();
  void getNextPose();
  bool imuMeasFromTimeRange(
      double start_time, double end_time,
      boost::circular_buffer<ImuMeas>::reverse_iterator& begin_imu_it,
      boost::circular_buffer<ImuMeas>::reverse_iterator& end_imu_it);


  sensor_msgs::Imu::Ptr transformImu(
      const sensor_msgs::Imu::ConstPtr& imu_raw);
  void publishPose();
  void publishPoseThread();
  void publishCloud(const pcl::PointCloud<PointType>::ConstPtr& published_cloud);
  void publishInitialMap();
  void publishFullMap();
  void publishGicpPose();

  tf2_ros::TransformBroadcaster broadcaster_;

  ros::NodeHandle nh;
  ros::Time clock_;

  // ros::Timer publish_timer;
  ros::WallTimer publish_timer;
  ros::WallTimer publish_timer_;
  ros::WallTimer publish_full_map_;
  ros::WallTimer publish_gicp_pose_;

  // 每个1s执行一次全局地图匹配
  ros::WallTimer cloud_timer_;

  ros::NodeHandle mt_nh;

  // Subscribers
  ros::Subscriber initial_pose_sub_;
  ros::Subscriber map_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber cloud_sub_;
  ros::Subscriber cloud_data_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber lio_state_sub_;

  // Publishers
  ros::Publisher pose_pub_;
  ros::Publisher initial_map_pub_;
  ros::Publisher initial_full_map_pub_;
  ros::Publisher gicp_pose_pub;

  ros::CallbackQueue lidar_cb_group, imu_cb_group, odom_cb_group,
      initial_pose_cb_group, map_cb_group;

  geometry_msgs::PoseWithCovarianceStamped corrent_pose_stamped_;

  // Point Clouds
  pcl::PointCloud<PointType>::ConstPtr original_scan;
  pcl::PointCloud<PointType>::ConstPtr deskewed_scan;
  pcl::PointCloud<PointType>::ConstPtr current_scan;
  
  // 储存最新一帧的点云
  sensor_msgs::PointCloud2ConstPtr latest_cloud_msg_;

  sensor_msgs::PointCloud2Ptr map_msg_ptr; 
  sensor_msgs::PointCloud2Ptr full_map_msg_ptr; 
  pcl::PointCloud<PointType>::Ptr global_map_;
  pcl::PointCloud<PointType>::Ptr full_map_;
  std::shared_ptr<const nano_gicp::CovarianceList> global_map_cov;
  std::atomic<bool> map_recieved_{false};
  std::atomic<bool> initialpose_recieved_{false};
  std::atomic<bool> first_scan_{false};
  // parameters
  std::string global_frame_id_;
  std::string full_global_frame_id_;
  std::string odom_frame_id_;
  std::string base_frame_id_;
  std::string imu_frame_id_;
  std::string laser_frame_id_;
  std::string registration_method_;
  double scan_max_range_;
  double scan_min_range_;
  double scan_period_;
  double ndt_resolution_;
  double ndt_step_size_;
  double transform_epsilon_;
  double voxel_leaf_size_;
  bool use_pcd_map_{false};
  std::string map_path_;
  std::string full_map_path_;
  std::string init_pose_path_;
  bool set_initial_pose_{false};
  double initial_pose_x_;
  double initial_pose_y_;
  double initial_pose_z_;
  double initial_pose_qx_;
  double initial_pose_qy_;
  double initial_pose_qz_;
  double initial_pose_qw_;
  double initial_pose_yaw_;
  double initial_pose_roll_;
  double initial_pose_pitch_;

  bool use_odom_{false};
  double last_odom_received_time_;
  bool enable_debug_{false};

  // Flags
  std::atomic<bool> dlio_initialized;
  std::atomic<bool> first_valid_scan;
  std::atomic<bool> first_imu_received_;
  std::atomic<bool> imu_calibrated;
  std::atomic<bool> gicp_hasConverged;
  std::atomic<bool> deskew_status;
  std::atomic<int> deskew_size;
  std::atomic<bool> initial_pose_input;
  double sweep_ref_time = {0.0};

  bool vf_use_;
  double vf_res_;

  bool adaptive_params_;

  // IMU
  ros::Time imu_stamp;
  double first_imu_stamp;
  double prev_imu_stamp;
  double imu_dp, imu_dq_deg;

  bool imu_calibrate_;
  bool calibrate_gyro_;
  bool calibrate_accel_;

  // bool gravity_align_;
  double imu_calib_time_;
  int imu_buffer_size_;
  Eigen::Matrix3f imu_accel_sm_;
  double gravity_;

  int gicp_min_num_points_;
  int gicp_k_correspondences_;
  double gicp_max_corr_dist_;
  int gicp_max_iter_;
  double gicp_transformation_ep_;
  double gicp_rotation_ep_;
  double gicp_init_lambda_factor_;

  double keyframe_thresh_dist_;
  double keyframe_thresh_rot_;

  int submap_knn_;
  int submap_kcv_;
  int submap_kcc_;
  double submap_concave_alpha_;

  bool densemap_filtered_;
  bool wait_until_move_;

  double crop_size_;

  bool deskew_;

  int num_threads_;

  // Threads
  std::thread publish_pose_thread;
  std::thread publish_frame_thread;
  std::thread metrics_thread;
  std::thread debug_thread;

  // Trajectory
  std::vector<std::pair<Eigen::Vector3f, Eigen::Quaternionf>> trajectory;
  // Frames
  std::string odom_frame;
  std::string baselink_frame;
  std::string lidar_frame;
  std::string imu_frame;

  // Preprocessing
  pcl::CropBox<PointType> crop;
  pcl::VoxelGrid<PointType> voxel;

  Eigen::Matrix4f T, T_prior, T_corr, T_init, T_corr_prev;
  Eigen::Quaternionf q_final;

  Eigen::Vector3f origin;

  boost::circular_buffer<ImuMeas> imu_buffer;
  std::mutex mtx_imu;
  std::condition_variable cv_imu_stamp;

  static bool comparatorImu(ImuMeas m1, ImuMeas m2) {
    return (m1.stamp < m2.stamp);
  };

  // Geometric Observer
  struct Geo {
    bool first_opt_done;
    std::mutex mtx;
    double dp;
    double dq_deg;
    Eigen::Vector3f prev_p;
    Eigen::Quaternionf prev_q;
    Eigen::Vector3f prev_vel;
  };
  Geo geo;

  // State Vector
  struct ImuBias {
    Eigen::Vector3f gyro;
    Eigen::Vector3f accel;
  };

  struct Frames {
    Eigen::Vector3f b;
    Eigen::Vector3f w;
  };

  struct Velocity {
    Frames lin;
    Frames ang;
  };

  struct State {
    Eigen::Vector3f p;     // position in world frame
    Eigen::Quaternionf q;  // orientation in world frame
    Velocity v;
    ImuBias b;  // imu biases in body frame
  };
  State state;
  dlio_loc::state_info state_info;

  struct Pose {
    Eigen::Vector3f p;     // position in world frame
    Eigen::Quaternionf q;  // orientation in world frame
  };
  Pose lidarPose;
  Pose imuPose;

  // Metrics
  struct Metrics {
    std::vector<float> spaciousness;
    std::vector<float> density;
  };
  Metrics metrics;

  struct Extrinsics {
    struct SE3 {
      Eigen::Vector3f t;
      Eigen::Matrix3f R;
    };
    SE3 baselink2imu;
    SE3 baselink2lidar;
    Eigen::Matrix4f baselink2imu_T;
    Eigen::Matrix4f baselink2lidar_T;
  };
  Extrinsics extrinsics;

  // Timestamps
  // std::mutex mtx_stamp;
  ros::Time scan_header_stamp;
  double scan_stamp;
  double prev_scan_stamp;
  double scan_dt;
  std::vector<double> comp_times;
  std::vector<double> imu_rates;
  std::vector<double> lidar_rates;

  double first_scan_stamp;
  double elapsed_time;
  geometry_msgs::Pose2D last_save_pose;
  // geometry_msgs::msg::Pose2D last_save_pose;
  ros::Time last_save_stamp;
  // GICP
  nano_gicp::NanoGICP<PointType, PointType> gicp;
  // nano_gicp::NanoGICP<PointType, PointType> gicp_temp;

  SensorType sensor;

  double geo_Kp_;
  double geo_Kv_;
  double geo_Kq_;
  double geo_Kab_;
  double geo_Kgb_;
  double geo_abias_max_;
  double geo_gbias_max_;
};
}  // namespace dlio