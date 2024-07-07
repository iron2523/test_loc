#include "dlio/odom.h"

int main(int argc, char** argv) {

  ros::init(argc, argv, "odom_node");

  pid_t pid = getpid();
  int cores = std::thread::hardware_concurrency();
  cpu_set_t cpuSet;
  CPU_ZERO(&cpuSet);
  for (int i = 1; i < 5 && i < cores; ++i) {
    CPU_SET(i, &cpuSet);
  }
  if (sched_setaffinity(pid, sizeof(cpuSet), &cpuSet) == -1) {
    ROS_ERROR("Failed to Sched Set Affinity");
    return 1;
  }

  ros::NodeHandle nh;
  ros::NodeHandle mt_nh;

  dlio::OdomNode node(nh, mt_nh);

  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::waitForShutdown();

  return 0;

}

