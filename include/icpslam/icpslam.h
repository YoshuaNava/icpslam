#pragma once

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

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
  void loadParameters();

  void advertisePublishers();

  void addNewKeyframe(
      const unsigned long& id, const ros::Time& stamp, const Pose6DOF& pose_in_odom, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

  void computeMapToOdomTransform();

  void publishMapToOdomTf(const ros::Time& stamp);

  void mainLoop();

  void publishPoseGraphMarkers(const ros::Time& stamp);

  const double KFS_DIST_THRESH = 0.3;
  const double VERTEX_DIST_THRESH = 0.05;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::string laser_frame_;
  std::string robot_frame_;
  std::string odom_frame_;
  std::string map_frame_;

  RobotOdometer robot_odometer_;
  IcpOdometer icp_odometer_;
  OctreeMapper octree_mapper_;
  pose_graph_utils::PoseGraphG2O::Ptr pose_graph_;

  Eigen::MatrixXd icp_inf_matrix_;
  Eigen::MatrixXd loop_inf_matrix_;
  Eigen::MatrixXd wheel_odom_inf_matrix_;

  std::vector<float> icp_nodes_markers_color_;
  std::vector<float> odom_nodes_markers_color_;
  std::vector<float> edges_markers_color_;
  double marker_scale_nodes_;
  double marker_scale_edges_;
  ros::Publisher markers_pub_;

  unsigned long num_keyframes_;
  int keyframes_window_;

  Eigen::Isometry3d T_map_to_odom_;
  tf::TransformBroadcaster tf_broadcaster_;

  std::vector<Keyframe<pcl::PointXYZ>::Ptr> keyframes_list_;
};