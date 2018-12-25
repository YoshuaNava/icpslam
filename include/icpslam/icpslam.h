
#include <slam_3d/pose_graph_g2o.hpp>

#include "icpslam/icp_odometer.h"
#include "icpslam/octree_mapper.h"
#include "icpslam/robot_odometer.h"
// #include "icpslam/pose_graph_gtsam.h"

class IcpSlam {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  IcpSlam(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);

 protected:
  void mainLoop();

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  RobotOdometer robot_odometer_;
  ICPOdometer icp_odometer_;
  OctreeMapper octree_mapper_;
  pose_graph_utils::PoseGraphG2O pose_graph_;

  const double KFS_DIST_THRESH = 0.3;
  const double VERTEX_DIST_THRESH = 0.05;
};