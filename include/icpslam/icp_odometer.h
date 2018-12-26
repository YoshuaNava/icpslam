#pragma once

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "utils/pose6DOF.h"

class ICPOdometer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ICPOdometer(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);

  void init();

  void loadParameters();

  void advertisePublishers();

  void registerSubscribers();

  bool isOdomReady();

  Pose6DOF getFirstPose();

  Pose6DOF getLatestPose();

  void getEstimates(pcl::PointCloud<pcl::PointXYZ>::Ptr* cloud, Pose6DOF* latest_icp_transform, Pose6DOF* icp_pose, bool* new_transform);

  void voxelFilterCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr* input, pcl::PointCloud<pcl::PointXYZ>::Ptr* output);

  void publishPath();

  bool updateICPOdometry(const ros::Time& stamp, const Eigen::Matrix4d& T);

  void laserCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);

 protected:
  // Constants
  const double ICP_FITNESS_THRESH = 0.1;
  const double ICP_MAX_CORR_DIST = 1.0;
  const double ICP_EPSILON = 1e-06;
  const double ICP_MAX_ITERS = 10;

  int verbosity_level_;

  bool odom_inited_;

  // ROS node handle, URDF frames, topics and publishers
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::string laser_frame_;
  std::string robot_frame_;
  std::string odom_frame_;
  std::string map_frame_;

  ros::Subscriber laser_cloud_sub_;

  ros::Publisher prev_cloud_pub_;
  ros::Publisher aligned_cloud_pub_;
  ros::Publisher icp_odom_pub_;
  ros::Publisher icp_odom_path_pub_;

  // PCL clouds
  pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr curr_cloud_;

  // Odometry path containers
  nav_msgs::Odometry icp_odom_;
  nav_msgs::Path icp_odom_path_;

  // Translations and rotations estimated by ICP
  bool new_transform_;
  double voxel_leaf_size_;

  int clouds_skipped_;
  int num_clouds_skip_;

  bool aggregate_clouds_;

  Pose6DOF icp_latest_transform_;
  std::vector<Pose6DOF> icp_odom_poses_;
  tf::TransformListener tf_listener_;
};
