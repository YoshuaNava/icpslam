
#include "icpslam/robot_odometer.h"

#include <tf/transform_datatypes.h>

#include "utils/geometric_utils.h"
#include "utils/messaging_utils.h"

RobotOdometer::RobotOdometer(const ros::NodeHandle& nh, const ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh) {
  odom_inited_ = false;
  new_transform_ = false;
  rodom_first_pose_.setIdentity();
  odom_latest_transform_.setIdentity();
  init();
}

void RobotOdometer::init() {
  loadParameters();
  advertisePublishers();
  registerSubscribers();

  ROS_INFO("IcpSlam: Robot odometer initialized");
}

void RobotOdometer::loadParameters() {
  pnh_.param("verbosity_level_", verbosity_level_, 1);

  // TF frames
  pnh_.param("map_frame", map_frame_, std::string("map"));
  pnh_.param("odom_frame", odom_frame_, std::string("odom"));
  pnh_.param("robot_frame", robot_frame_, std::string("base_link"));
  pnh_.param("laser_frame", laser_frame_, std::string("laser"));

  // Input robot odometry topic
  pnh_.param("robot_odom_topic", robot_odom_topic_, std::string("/odometry/filtered"));
}

void RobotOdometer::advertisePublishers() {
  robot_odom_pub_ = pnh_.advertise<nav_msgs::Odometry>("robot_odometer/odometry", 1);
  robot_pose_pub_ = pnh_.advertise<geometry_msgs::PoseStamped>("robot_odometer/pose", 1);
  robot_odom_path_pub_ = pnh_.advertise<nav_msgs::Path>("robot_odometer/path", 1);
}

void RobotOdometer::registerSubscribers() {
  robot_odometry_sub_ = nh_.subscribe(robot_odom_topic_, 5, &RobotOdometer::robotOdometryCallback, this);
}

bool RobotOdometer::isOdomReady() {
  return odom_inited_;
}

Pose6DOF RobotOdometer::getFirstPose() {
  return robot_odom_poses_.front();
}

Pose6DOF RobotOdometer::getLatestPose() {
  return robot_odom_poses_.back();
}

void RobotOdometer::getEstimates(Pose6DOF* latest_odom_transform, Pose6DOF* odom_pose, bool* new_transform) {
  *latest_odom_transform = odom_latest_transform_;
  *odom_pose = robot_odom_poses_.back();
  *new_transform = new_transform_;

  this->new_transform_ = false;
}

void RobotOdometer::robotOdometryCallback(const nav_msgs::Odometry::ConstPtr& robot_odom_msg) {
  ROS_DEBUG("IcpSlam: Robot odometry callback!");

  ros::Time stamp = robot_odom_msg->header.stamp;
  geometry_msgs::PoseWithCovariance pose_cov_msg = robot_odom_msg->pose;
  Pose6DOF pose_in_odom(pose_cov_msg, stamp), pose_in_map;

  // Remove initial pose offset and save poses
  Pose6DOF pose_without_offset = pose_in_odom - rodom_first_pose_;
  int num_poses = robot_odom_poses_.size();
  if (num_poses == 0) {
    ROS_DEBUG("IcpSlam: Robot odometry first pose.");
    rodom_first_pose_ = pose_in_odom;
    robot_odom_poses_.push_back(pose_in_odom);
    insertPoseInPath(pose_in_odom.toROSPose(), odom_frame_, stamp, robot_odom_path_);
    odom_inited_ = true;
  } else {
    if (verbosity_level_ >= 2) {
      std::cout << "Robot odometry pose:\n" << pose_without_offset.toStringQuat("   ");
      std::cout << "Robot odometry transform:\n" << odom_latest_transform_.toStringQuat("   ");
      std::cout << std::endl;
    }

    Pose6DOF prev_pose_path(robot_odom_path_.poses[num_poses - 1].pose);
    if (Pose6DOF::distanceEuclidean(pose_without_offset, prev_pose_path) >= POSE_DIST_THRESH) {
      robot_odom_poses_.push_back(pose_in_odom);
      insertPoseInPath(pose_without_offset.toROSPose(), odom_frame_, stamp, robot_odom_path_);
    }
  }

  // Publish odometry, pose and path for visualization
  if (robot_odom_pub_.getNumSubscribers() > 0) {
    robot_odom_pub_.publish(robot_odom_msg);
  }
  if (robot_pose_pub_.getNumSubscribers() > 0) {
    geometry_msgs::PoseStamped pose_stamped_msg;
    pose_stamped_msg.header.stamp = stamp;
    pose_stamped_msg.pose = robot_odom_msg->pose.pose;
    robot_pose_pub_.publish(pose_stamped_msg);
  }
  if (robot_odom_path_pub_.getNumSubscribers() > 0) {
    robot_odom_path_.header.stamp = stamp;
    robot_odom_path_.header.frame_id = odom_frame_;
    robot_odom_path_pub_.publish(robot_odom_path_);
  }

  new_transform_ = true;
}
