#pragma once

#include <ros/ros.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <slam_3d/pose_graph_g2o.hpp>

#include "utils/pose6DOF.h"

template <typename PointType>
class Keyframe {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::shared_ptr<Keyframe>;
  typedef pcl::PointCloud<PointType> PclPointCloud;
  typedef typename pcl::PointCloud<PointType>::Ptr PclPointCloudPtr;

  Keyframe(const unsigned long& id) : id_(id), point_cloud_(new PclPointCloud()) {
  }

  Keyframe(const unsigned long& id, const ros::Time& stamp, const Pose6DOF& pose_in_odom, const PclPointCloudPtr& point_cloud)
      : id_(id), stamp_(stamp), pose_in_odom_(pose_in_odom), point_cloud_(point_cloud) {
  }

  //  protected:
  const unsigned long id_;
  ros::Time stamp_;
  Pose6DOF pose_in_odom_;
  g2o::VertexSE3* graph_node_;

  PclPointCloudPtr point_cloud_;
};