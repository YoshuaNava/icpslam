#pragma once

#include <ros/ros.h>

#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/octree/octree_search.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "utils/pose6DOF.h"

class OctreeMapper {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::shared_ptr<OctreeMapper>;

  OctreeMapper(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);

  void init();

  void loadParameters();

  void advertisePublishers();

  void registerSubscribers();

  void resetMap();

  /* This function is inspired on https://github.com/erik-nelson/point_cloud_mapper */
  void addPointsToMap(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);

  void incrementCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);

  bool refineTransformICP(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const Pose6DOF& prev_pose, Pose6DOF* transform);

  bool approxNearestNeighbors(const pcl::PointCloud<pcl::PointXYZ>& cloud, pcl::PointCloud<pcl::PointXYZ>* neighbors);

  void transformCloudToPoseFrame(
      const pcl::PointCloud<pcl::PointXYZ>& in_cloud, const Pose6DOF& pose, pcl::PointCloud<pcl::PointXYZ>* out_cloud);

  void publishPath(const Pose6DOF& latest_pose);

  bool estimateTransformICP(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& curr_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& nn_cloud, Pose6DOF* transform);

 protected:
  // Constants for mapping
  const float OCTREE_RESOLUTION = 0.3;
  const double ICP_FITNESS_THRESH = 0.1;
  const double ICP_MAX_CORR_DIST = 1.0;
  const double ICP_EPSILON = 1e-06;
  const double ICP_MAX_ITERS = 10;

  int verbosity_level_;

  // Frames, topics and publishers
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::string laser_frame_;
  std::string robot_frame_;
  std::string odom_frame_;
  std::string map_frame_;

  ros::Subscriber increment_cloud_sub_;

  ros::Publisher map_cloud_pub_;
  ros::Publisher nn_cloud_pub_;
  ros::Publisher refined_path_pub_;

  // tf
  tf::TransformListener tf_listener_;

  // PCL clouds for mapping
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr map_octree_;

  // Odometry path containers
  nav_msgs::Path refined_path_;
};