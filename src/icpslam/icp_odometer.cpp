
#include "icpslam/icp_odometer.h"

#include <tf/transform_datatypes.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/impl/transforms.hpp>

#include "utils/geometric_utils.h"
#include "utils/messaging_utils.h"

ICPOdometer::ICPOdometer(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh), prev_cloud_(new pcl::PointCloud<pcl::PointXYZ>()), curr_cloud_(new pcl::PointCloud<pcl::PointXYZ>()) {
  odom_inited_ = false;
  new_transform_ = false;
  clouds_skipped_ = 0;
  icp_latest_transform_.setIdentity();
  icp_odom_poses_.push_back(Pose6DOF::getIdentity());
  init();
}

void ICPOdometer::init() {
  loadParameters();
  advertisePublishers();
  registerSubscribers();

  ROS_INFO("IcpSlam: ICP odometer initialized");
}

void ICPOdometer::loadParameters() {
  pnh_.param("verbosity_level_", verbosity_level_, 1);

  // TF frames
  pnh_.param("map_frame", map_frame_, std::string("map"));
  pnh_.param("odom_frame", odom_frame_, std::string("odom"));
  pnh_.param("robot_frame", robot_frame_, std::string("base_link"));
  pnh_.param("laser_frame", laser_frame_, std::string("laser"));

  // Input robot odometry and point cloud topics
  pnh_.param("laser_cloud_topic", laser_cloud_topic_, std::string("input_cloud"));

  pnh_.param("voxel_leaf_size", voxel_leaf_size_, 0.05);
  pnh_.param("aggregate_clouds", aggregate_clouds_, false);
  pnh_.param("num_clouds_skip", num_clouds_skip_, 0);

  // ICP odometry debug topics
  if (verbosity_level_ >= 1) {
    pnh_.param("prev_cloud_topic", prev_cloud_topic_, std::string("icp_odometer/prev_cloud"));
    pnh_.param("aligned_cloud_topic", aligned_cloud_topic_, std::string("icp_odometer/aligned_cloud"));

    pnh_.param("icp_odom_topic", icp_odom_topic_, std::string("icp_odometer/odom"));
    pnh_.param("icp_odom_path_topic", icp_odom_path_topic_, std::string("icp_odometer/icp_odom_path"));
  }
}

void ICPOdometer::advertisePublishers() {
  icp_odom_pub_ = pnh_.advertise<nav_msgs::Odometry>(icp_odom_topic_, 1);
  icp_odom_path_pub_ = pnh_.advertise<nav_msgs::Path>(icp_odom_path_topic_, 1);

  // ICP odometry debug topics
  if (verbosity_level_ >= 1) {
    prev_cloud_pub_ = pnh_.advertise<sensor_msgs::PointCloud2>(prev_cloud_topic_, 1);
    aligned_cloud_pub_ = pnh_.advertise<sensor_msgs::PointCloud2>(aligned_cloud_topic_, 1);
  }
}

void ICPOdometer::registerSubscribers() {
  laser_cloud_sub_ = nh_.subscribe("laser/point_cloud", 1, &ICPOdometer::laserCloudCallback, this);
}

bool ICPOdometer::isOdomReady() {
  return odom_inited_;
}

Pose6DOF ICPOdometer::getFirstPose() {
  return icp_odom_poses_.front();
}

Pose6DOF ICPOdometer::getLatestPose() {
  return icp_odom_poses_.back();
}

void ICPOdometer::getEstimates(
    pcl::PointCloud<pcl::PointXYZ>::Ptr* cloud, Pose6DOF* latest_icp_transform, Pose6DOF* icp_pose, bool* new_transform) {
  **cloud = *prev_cloud_;
  *latest_icp_transform = icp_latest_transform_;

  *icp_pose = icp_odom_poses_.back();

  *new_transform = new_transform_;

  icp_latest_transform_.setIdentity();

  this->new_transform_ = false;
}

void ICPOdometer::voxelFilterCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr* input, pcl::PointCloud<pcl::PointXYZ>::Ptr* output) {
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setInputCloud(*input);
  voxel_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
  voxel_filter.filter(**output);
}

void ICPOdometer::publishPath() {
  Pose6DOF latest_pose = getLatestPose();
  publishOdometry(latest_pose.pos, latest_pose.rot, map_frame_, odom_frame_, ros::Time().now(), &icp_odom_pub_);
  insertPoseInPath(latest_pose.toROSPose(), map_frame_, ros::Time().now(), icp_odom_path_);
  icp_odom_path_.header.stamp = ros::Time().now();
  icp_odom_path_.header.frame_id = map_frame_;
  icp_odom_path_pub_.publish(icp_odom_path_);
}

bool ICPOdometer::updateICPOdometry(Eigen::Matrix4d T) {
  // ROS_INFO("IcpSlam: ICP odometry update!");
  Pose6DOF transform(T, ros::Time().now());
  Pose6DOF prev_pose = getLatestPose();
  Pose6DOF new_pose = prev_pose + transform;
  icp_latest_transform_ += transform;

  if (verbosity_level_ >= 2) {
    std::cout << "#####		ICP odometry		#####" << std::endl;
    std::cout << "Initial pose:\n" << getFirstPose().toStringQuat("   ");
    std::cout << "Prev odometry pose:\n" << prev_pose.toStringQuat("   ");
    std::cout << "Cloud transform = " << transform.toStringQuat("   ");
    std::cout << "ICP odometry pose = " << new_pose.toStringQuat("   ");
    std::cout << std::endl;
  }

  // if(transform.norm() < 0.00001)
  // 	return false;
  // else
  {
    new_transform_ = true;
    icp_odom_poses_.push_back(new_pose);
    publishPath();

    return true;
  }
}

void ICPOdometer::laserCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
  // ROS_INFO("IcpSlam: Cloud callback!");
  // std::clock_t start;
  // start = std::clock();
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*cloud_msg, *input_cloud);
  voxelFilterCloud(&input_cloud, &curr_cloud_);

  if (prev_cloud_->points.size() == 0) {
    *prev_cloud_ = *curr_cloud_;
    return;
  }

  // Registration
  // GICP is said to be better, but what about NICP from Serafin?
  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setMaximumIterations(ICP_MAX_ITERS);
  icp.setTransformationEpsilon(ICP_EPSILON);
  icp.setMaxCorrespondenceDistance(ICP_MAX_CORR_DIST);
  icp.setRANSACIterations(0);
  icp.setInputSource(curr_cloud_);
  icp.setInputTarget(prev_cloud_);

  pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud_in_curr_frame(new pcl::PointCloud<pcl::PointXYZ>()),
      curr_cloud_in_prev_frame(new pcl::PointCloud<pcl::PointXYZ>()), joint_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  icp.align(*curr_cloud_in_prev_frame);
  Eigen::Matrix4d T = icp.getFinalTransformation().cast<double>();

  if (icp.hasConverged()) {
    // ROS_INFO("IcpSlam:		ICP converged");
    // std::cout << "Estimated T:\n" << T << std::endl;
    Eigen::Matrix4d T_inv = T.inverse();
    pcl::transformPointCloud(*prev_cloud_, *prev_cloud_in_curr_frame, T_inv);
    if (clouds_skipped_ >= num_clouds_skip_) {
      bool success = updateICPOdometry(T);
      if (success) {
        odom_inited_ = true;
        *prev_cloud_ = *curr_cloud_;
        clouds_skipped_ = 0;
      }
    } else {
      if (aggregate_clouds_) {
        // TODO: Test this. Is the transformation correct?.
        // TODO: How about implementing cloud aggregation with TF, saving the joint cloud in the parent frame?
        *joint_cloud = *prev_cloud_;
        pcl::transformPointCloud(*joint_cloud, *joint_cloud, T_inv);
        *joint_cloud += *curr_cloud_in_prev_frame;
        pcl::transformPointCloud(*joint_cloud, *joint_cloud, T);
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter_joint;
        voxel_filter_joint.setInputCloud(joint_cloud);
        voxel_filter_joint.setLeafSize(0.15, 0.15, 0.15);
        voxel_filter_joint.filter(*prev_cloud_);
      }
      clouds_skipped_++;
    }

    if (verbosity_level_ >= 1) {
      publishPointCloud(prev_cloud_, cloud_msg->header.frame_id, ros::Time().now(), &prev_cloud_pub_);
      publishPointCloud(curr_cloud_in_prev_frame, cloud_msg->header.frame_id, ros::Time().now(), &aligned_cloud_pub_);
    }
  }

  // std::cout << "Time elapsed: " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;
}
