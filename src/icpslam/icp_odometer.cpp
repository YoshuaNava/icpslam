
#include "icpslam/icp_odometer.h"

#include <tf/transform_datatypes.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/impl/transforms.hpp>

#include "utils/geometric_utils.h"
#include "utils/messaging_utils.h"

IcpOdometer::IcpOdometer(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
    : nh_(nh),
      pnh_(pnh),
      initial_pose_set_(false),
      odom_inited_(false),
      new_transform_(false),
      clouds_skipped_(0),
      num_clouds_skip_(0),
      voxel_leaf_size_(0.1),
      icp_latest_transform_(Pose6DOF::getIdentity()),
      prev_cloud_(new pcl::PointCloud<pcl::PointXYZ>()),
      curr_cloud_(new pcl::PointCloud<pcl::PointXYZ>()) {
  init();
}

void IcpOdometer::init() {
  loadParameters();
  advertisePublishers();
  registerSubscribers();

  ROS_INFO("IcpSlam: ICP odometer initialized");
}

void IcpOdometer::loadParameters() {
  pnh_.param("verbosity_level_", verbosity_level_, 1);

  // Tf frames
  pnh_.param("map_frame", map_frame_, std::string("map"));
  pnh_.param("odom_frame", odom_frame_, std::string("odom"));
  pnh_.param("robot_frame", robot_frame_, std::string("base_link"));
  pnh_.param("laser_frame", laser_frame_, std::string("laser"));

  pnh_.param("num_clouds_skip", num_clouds_skip_, 0);
  pnh_.param("voxel_leaf_size", voxel_leaf_size_, 0.05);
}

void IcpOdometer::advertisePublishers() {
  icp_odom_pub_ = pnh_.advertise<nav_msgs::Odometry>("icp_odometer/odom", 1, true);
  icp_pose_pub_ = pnh_.advertise<geometry_msgs::PoseStamped>("icp_odometer/pose", 1, true);
  icp_odom_path_pub_ = pnh_.advertise<nav_msgs::Path>("icp_odometer/path", 1, true);

  // ICP odometry debug topics
  if (verbosity_level_ >= 1) {
    prev_cloud_pub_ = pnh_.advertise<sensor_msgs::PointCloud2>("icp_odometer/prev_cloud", 1);
    aligned_cloud_pub_ = pnh_.advertise<sensor_msgs::PointCloud2>("icp_odometer/aligned_cloud", 1);
  }
}

void IcpOdometer::registerSubscribers() {
  laser_cloud_sub_ = nh_.subscribe("laser/point_cloud", 1, &IcpOdometer::laserCloudCallback, this);
}

bool IcpOdometer::isOdomReady() const {
  return odom_inited_;
}

void IcpOdometer::setInitialPose(const Pose6DOF& initial_pose) {
  icp_odom_poses_.push_back(initial_pose);
  initial_pose_set_ = true;
}

Pose6DOF IcpOdometer::getFirstPose() const {
  return icp_odom_poses_.front();
}

Pose6DOF IcpOdometer::getLatestPose() const {
  return icp_odom_poses_.back();
}

void IcpOdometer::getEstimates(
    ros::Time& stamp, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, Pose6DOF& latest_icp_transform, Pose6DOF& icp_pose, bool& new_transform) {
  cloud = prev_cloud_;

  stamp = latest_stamp;
  latest_icp_transform = icp_latest_transform_;
  icp_pose = getLatestPose();
  new_transform = new_transform_;

  icp_latest_transform_.setIdentity();

  this->new_transform_ = false;
}

void IcpOdometer::voxelFilterCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr* input, pcl::PointCloud<pcl::PointXYZ>::Ptr* output) {
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setInputCloud(*input);
  voxel_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
  voxel_filter.filter(**output);
}

void IcpOdometer::publishPath(const ros::Time& stamp) {
  icp_odom_path_.header.stamp = stamp;
  icp_odom_path_.header.frame_id = map_frame_;
  icp_odom_path_pub_.publish(icp_odom_path_);
}

bool IcpOdometer::updateICPOdometry(const ros::Time& stamp, const Eigen::Matrix4d& T) {
  // ROS_INFO("IcpSlam: ICP odometry update!");
  Pose6DOF transform(T, stamp);
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
    insertPoseInPath(new_pose.toROSPose(), map_frame_, stamp, icp_odom_path_);

    if (icp_odom_pub_.getNumSubscribers() > 0) {
      publishOdometry(new_pose.pos, new_pose.rot, map_frame_, odom_frame_, stamp, &icp_odom_pub_);
    }
    if(icp_pose_pub_.getNumSubscribers() >0) {
      publishPoseStamped(new_pose.pos, new_pose.rot, map_frame_, stamp, &icp_pose_pub_);
    }
    if (icp_odom_path_pub_.getNumSubscribers() > 0) {
      publishPath(stamp);
    }

    return true;
  }
}

void IcpOdometer::laserCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
  // ROS_INFO("IcpSlam: Cloud callback!");
  if (!initial_pose_set_) {
    ROS_WARN("IcpSlam: ICP odometer waiting for initial pose.");
    return;
  }

  if (clouds_skipped_ < num_clouds_skip_) {
    ROS_WARN("IcpSlam: ICP odometer skips cloud");
    clouds_skipped_++;
    return;
  }
  clouds_skipped_ = 0;

  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*cloud_msg, *input_cloud);

  // Filter cloud
  voxelFilterCloud(&input_cloud, &curr_cloud_);

  if (prev_cloud_->points.size() == 0) {
    *prev_cloud_ = *curr_cloud_;
    return;
  }

  latest_stamp = cloud_msg->header.stamp;

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
    // ROS_WARN("IcpSlam:    ICP odometer converged");
    // std::cout << "Estimated T:\n" << T << std::endl;
    Eigen::Matrix4d T_inv = T.inverse();
    pcl::transformPointCloud(*prev_cloud_, *prev_cloud_in_curr_frame, T_inv);
    bool success = updateICPOdometry(latest_stamp, T);
    if (success) {
      odom_inited_ = true;
      *prev_cloud_ = *curr_cloud_;
    }

    if (verbosity_level_ >= 1) {
      if (prev_cloud_pub_.getNumSubscribers() > 0) {
        publishPointCloud(prev_cloud_, cloud_msg->header.frame_id, latest_stamp, &prev_cloud_pub_);
      }
      if (aligned_cloud_pub_.getNumSubscribers() > 0) {
        publishPointCloud(curr_cloud_in_prev_frame, cloud_msg->header.frame_id, latest_stamp, &aligned_cloud_pub_);
      }
    }
  }
}
