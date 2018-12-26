#pragma once

#include <slam_3d/pose_graph_g2o.hpp>

#include "icpslam/icp_odometer.h"
#include "icpslam/keyframe.h"
#include "icpslam/octree_mapper.h"
#include "icpslam/robot_odometer.h"

class IcpSlam {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  IcpSlam(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);

 protected:
  void mainLoop();

  const double KFS_DIST_THRESH = 0.3;
  const double VERTEX_DIST_THRESH = 0.05;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  RobotOdometer robot_odometer_;
  ICPOdometer icp_odometer_;
  OctreeMapper octree_mapper_;
  pose_graph_utils::PoseGraphG2O::Ptr pose_graph_;

  std::vector<Keyframe<pcl::PointXYZ>::Ptr> keyframes_;
};