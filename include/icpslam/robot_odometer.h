#pragma once

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "utils/pose6DOF.h"

class RobotOdometer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  RobotOdometer(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);

  void init();

  void loadParameters();

  void advertisePublishers();

  void registerSubscribers();

  bool isOdomReady();

  Pose6DOF getFirstPose();

  Pose6DOF getLatestPose();

  void getEstimates(Pose6DOF* latest_odom_transform, Pose6DOF* odom_pose, bool* new_transform);

  void robotOdometryCallback(const nav_msgs::Odometry::ConstPtr& robot_odom_msg);

 protected:
  // Constants
  const double POSE_DIST_THRESH = 0.1;

  int verbosity_level_;

  bool odom_inited_;

  // ROS node handle, URDF frames, topics and publishers
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::string laser_frame_;
  std::string robot_frame_;
  std::string odom_frame_;
  std::string map_frame_;

  std::string robot_odom_topic_;
  std::string robot_odom_path_topic_;

  ros::Publisher robot_odom_path_pub_;
  ros::Subscriber robot_odometry_sub_;

  // Odometry path containers
  nav_msgs::Path robot_odom_path_;

  // Translations and rotations estimated by robot odometry
  bool new_transform_;
  Pose6DOF rodom_first_pose_;
  Pose6DOF odom_latest_transform_;
  std::vector<Pose6DOF> robot_odom_poses_;
  tf::TransformListener tf_listener_;
};