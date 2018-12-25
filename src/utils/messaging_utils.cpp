
#include "utils/messaging_utils.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/impl/transforms.hpp>

#include "utils/geometric_utils.h"

void insertPoseInPath(
    Eigen::Vector3d position, Eigen::Quaterniond orientation, std::string frame_id, ros::Time stamp, nav_msgs::Path& path) {
  geometry_msgs::PoseStamped pose_stamped_msg;
  pose_stamped_msg.pose = getROSPoseFromPosQuat(position, orientation);
  pose_stamped_msg.header.stamp = stamp;
  pose_stamped_msg.header.frame_id = frame_id;
  path.poses.push_back(pose_stamped_msg);
}

void insertPoseInPath(geometry_msgs::Pose pose, std::string frame_id, ros::Time stamp, nav_msgs::Path& path) {
  geometry_msgs::PoseStamped pose_stamped_msg;
  pose_stamped_msg.pose = pose;
  pose_stamped_msg.header.stamp = stamp;
  pose_stamped_msg.header.frame_id = frame_id;
  path.poses.push_back(pose_stamped_msg);
}

void publishOdometry(
    Eigen::Vector3d position, Eigen::Quaterniond orientation, std::string ref_frame, std::string robot_frame, ros::Time stamp,
    ros::Publisher* pub_ptr) {
  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = stamp;
  odom_msg.header.frame_id = ref_frame;

  // set the position
  odom_msg.pose.pose.position.x = position(0);
  odom_msg.pose.pose.position.y = position(1);
  odom_msg.pose.pose.position.z = position(2);
  odom_msg.pose.pose.orientation.x = orientation.x();
  odom_msg.pose.pose.orientation.y = orientation.y();
  odom_msg.pose.pose.orientation.z = orientation.z();
  odom_msg.pose.pose.orientation.w = orientation.w();

  // set the velocity
  odom_msg.child_frame_id = robot_frame;
  odom_msg.twist.twist.linear.x = 0;
  odom_msg.twist.twist.linear.y = 0;
  odom_msg.twist.twist.angular.z = 0;

  pub_ptr->publish(odom_msg);
}

void publishPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string frame_id, ros::Time stamp, ros::Publisher* pub_ptr) {
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud, cloud_msg);
  cloud_msg.header.stamp = stamp;
  cloud_msg.header.frame_id = frame_id;
  pub_ptr->publish(cloud_msg);
}
