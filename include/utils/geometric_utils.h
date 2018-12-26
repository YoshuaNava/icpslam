#pragma once

#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Transform.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "utils/pose6DOF.h"

geometry_msgs::Point getROSPointFromPose6DOF(Pose6DOF pose);

Eigen::Matrix<double, 6, 6> getCovarianceFromROSPoseWithCovariance(geometry_msgs::PoseWithCovariance pose_msg);

tf::Transform getTFTransformFromROSOdometry(nav_msgs::Odometry odom_msg);

tf::Pose getTFPoseFromROSPose(geometry_msgs::Pose pose);

tf::Transform getInverseTFTransformFromROSOdometry(nav_msgs::Odometry odom_msg);

tf::Transform getTFTransformFromPositionQuaternion(Eigen::Vector3d pos, Eigen::Quaterniond q);

tf::Pose getTFPoseFromPositionQuaternion(Eigen::Vector3d pos, Eigen::Quaterniond q);

geometry_msgs::Pose getROSPoseFromPosQuat(Eigen::Vector3d pos, Eigen::Quaterniond q);

Eigen::Vector3d getTranslationFromROSPose(geometry_msgs::Pose pose);

Eigen::Quaterniond getQuaternionFromROSPose(geometry_msgs::Pose pose);

std::string getStringFromVector3d(Eigen::Vector3d vector);

std::string getStringFromQuaternion(Eigen::Quaterniond q);

tf::Pose differenceBetweenPoses(geometry_msgs::Pose p1, geometry_msgs::Pose p2);

double lengthOfRosPose(tf::Pose vector);

Eigen::Isometry3d getEigenIsometry3FromRosOdometry(const nav_msgs::Odometry odom_msg);

nav_msgs::Odometry getRosOdometryFromEigenIsometry3(const std::string frame_id, const Eigen::Isometry3d& odom, const ros::Time stamp);

geometry_msgs::TransformStamped getTfStampedFromEigenMatrix(
    const ros::Time& stamp, const Eigen::Matrix4f& pose, const std::string& frame_id, const std::string& child_frame_id);