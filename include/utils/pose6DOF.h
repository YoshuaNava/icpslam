#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Transform.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

class Pose6DOF {
 private:
  const double EQUALITY_THRESH = 1e-10;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ros::Time time_stamp;
  Eigen::Vector3d pos;
  Eigen::Quaterniond rot;
  Eigen::Matrix<double, 6, 6> cov;

  Pose6DOF();

  Pose6DOF(Eigen::Matrix4d& T, ros::Time stamp = ros::Time(0));

  Pose6DOF(
      Eigen::Matrix4d& T, std::string tgt_frame, std::string src_frame, tf::TransformListener* tf_listener, ros::Time stamp = ros::Time(0));

  Pose6DOF operator+=(const Pose6DOF& p2) {
    Pose6DOF p3 = compose(*this, p2);
    pos = p3.pos;
    rot = p3.rot;
    return p3;
  }

  Pose6DOF operator-=(const Pose6DOF& p2) {
    Pose6DOF p3 = subtract(p2, *this);
    pos = p3.pos;
    rot = p3.rot;
    return p3;
  }

  Pose6DOF operator+(const Pose6DOF& p2) const {
    Pose6DOF p3 = compose(*this, p2);
    return p3;
  }

  Pose6DOF operator-(const Pose6DOF& p2) const {
    Pose6DOF p3 = subtract(p2, *this);
    return p3;
  }

  Pose6DOF& operator=(const Pose6DOF pose) {
    time_stamp = pose.time_stamp;
    pos(0) = pose.pos(0);
    pos(1) = pose.pos(1);
    pos(2) = pose.pos(2);

    rot.x() = pose.rot.x();
    rot.y() = pose.rot.y();
    rot.z() = pose.rot.z();
    rot.w() = pose.rot.w();
    cov = pose.cov;

    return *this;
  }

  bool operator==(const Pose6DOF& pose) {
    return (((this->pos - pose.pos).norm() < EQUALITY_THRESH) && (fabs(this->rot.dot(pose.rot)) < 1 - EQUALITY_THRESH));
  }

  friend std::ostream& operator<<(std::ostream& os, const Pose6DOF& pose) {
    os << pose.toStringQuat();
    return os;
  }

  double norm() const;
  Pose6DOF inverse() const;
  Pose6DOF compose(const Pose6DOF& p2);
  Pose6DOF subtract(const Pose6DOF& p2);
  Pose6DOF setIdentity();
  double distanceEuclidean(const Pose6DOF p2) const;

  static double distanceEuclidean(const Pose6DOF& p1, const Pose6DOF& p2);
  static Pose6DOF compose(const Pose6DOF& p1, const Pose6DOF& p2);
  static Pose6DOF subtract(const Pose6DOF& p1, const Pose6DOF& p2);
  static Pose6DOF inverse(const Pose6DOF& pose);
  static Pose6DOF getIdentity();

  std::string toStringQuat(const std::string& indent = "") const;
  std::string toStringRPY(const std::string& indent = "") const;

  void transformToFixedFrame(std::string tgt_frame, std::string src_frame, tf::TransformListener* tf_listener);
  static Pose6DOF transformToFixedFrame(
      Pose6DOF pose_in_src, std::string tgt_frame, std::string src_frame, tf::TransformListener* tf_listener);

  Pose6DOF(geometry_msgs::Pose& pose_msg, ros::Time stamp = ros::Time(0));
  Pose6DOF(
      geometry_msgs::Pose& pose_msg, std::string tgt_frame, std::string src_frame, tf::TransformListener* tf_listener,
      ros::Time stamp = ros::Time(0));
  Pose6DOF(geometry_msgs::PoseWithCovariance& pose_msg, ros::Time stamp = ros::Time(0));
  Pose6DOF(tf::Transform& transform, ros::Time stamp = ros::Time(0));
  Pose6DOF(geometry_msgs::Transform& transform, ros::Time stamp = ros::Time(0));

  void fromEigenMatrix(Eigen::Matrix4d& T);
  void fromEigenMatrixInFixedFrame(Eigen::Matrix4d& T, std::string tgt_frame, std::string src_frame, tf::TransformListener* tf_listener);
  void fromROSPoseInFixedFrame(
      geometry_msgs::Pose pose_msg, std::string tgt_frame, std::string src_frame, tf::TransformListener* tf_listener);
  void fromROSPose(geometry_msgs::Pose& pose);
  void fromROSPoseWithCovariance(geometry_msgs::PoseWithCovariance& pose_cov);
  void fromROSTransform(geometry_msgs::Transform& transform);
  void fromTFPose(tf::Pose& pose);
  void fromTFTransform(tf::Transform& transform);

  Eigen::Matrix4d toTMatrix() const;
  tf::Transform toTFTransform() const;
  tf::Pose toTFPose() const;
  geometry_msgs::Point toROSPoint() const;
  geometry_msgs::Pose toROSPose() const;
  geometry_msgs::PoseWithCovariance toROSPoseWithCovariance() const;
};
