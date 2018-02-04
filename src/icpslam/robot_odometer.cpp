
#include "utils/pose6DOF.h"
#include "utils/geometric_utils.h"
#include "utils/messaging_utils.h"
#include "icpslam/robot_odometer.h"

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>


#include <Eigen/Dense>
#include <Eigen/Geometry>



RobotOdometer::RobotOdometer(ros::NodeHandle nh) :
	nh_(nh)
{
	odom_inited_ = false;
	new_transform_ = false;
	rodom_first_pose_.setIdentity();
	odom_latest_transform_.setIdentity();
	init();	
}

void RobotOdometer::init()
{
	loadParameters();
	advertisePublishers();
	registerSubscribers();

	ROS_INFO("Robot odometer initialized");
}

void RobotOdometer::loadParameters()
{
	nh_.param("verbosity_level_", verbosity_level_, 1);

	// TF frames
	nh_.param("map_frame", map_frame_, std::string("map"));
	nh_.param("odom_frame", odom_frame_, std::string("odom"));
	nh_.param("robot_frame", robot_frame_, std::string("base_link"));
	nh_.param("laser_frame", laser_frame_, std::string("laser"));

	// Input robot odometry topic
	nh_.param("robot_odom_topic", robot_odom_topic_, std::string("/odometry/filtered"));

	// Robot odometry debug topics
	if(verbosity_level_ >=1)
	{
		nh_.param("robot_odom_path_topic", robot_odom_path_topic_, std::string("icpslam/robot_odom_path"));
		nh_.param("true_path_topic", true_path_topic_, std::string("icpslam/true_path"));
	}
}

void RobotOdometer::advertisePublishers()
{
	robot_odom_path_pub_ = nh_.advertise<nav_msgs::Path>(robot_odom_path_topic_, 1);
	
	// Robot odometry debug topics
	if(verbosity_level_ >=1)
	{
		true_path_pub_ = nh_.advertise<nav_msgs::Path>(true_path_topic_, 1);
	}
}

void RobotOdometer::registerSubscribers()
{
	robot_odometry_sub_ = nh_.subscribe(robot_odom_topic_, 5, &RobotOdometer::robotOdometryCallback, this);
}

bool RobotOdometer::isOdomReady()
{
	return odom_inited_;
}

Pose6DOF RobotOdometer::getFirstPoseRobotOdometry()
{
	return robot_odom_poses_.front(); 
}

Pose6DOF RobotOdometer::getLatestPoseRobotOdometry() 
{
	return robot_odom_poses_.back(); 
}


void RobotOdometer::getEstimates(Pose6DOF *latest_odom_transform, Pose6DOF *odom_pose, bool *new_transform)
{
	*latest_odom_transform = odom_latest_transform_;
	*odom_pose = robot_odom_poses_.back();
	*new_transform = new_transform_;

	this->new_transform_ = false;
}


void RobotOdometer::robotOdometryCallback(const nav_msgs::Odometry::ConstPtr& robot_odom_msg)
{
	// ROS_INFO("Robot odometry callback!");
	geometry_msgs::PoseWithCovariance pose_cov_msg = robot_odom_msg->pose;
	Pose6DOF pose_in_odom(pose_cov_msg, robot_odom_msg->header.stamp),
			 pose_in_map;

	if(tf_listener_.canTransform(map_frame_, odom_frame_, ros::Time(0)))
	{
		pose_in_map = Pose6DOF::transformToFixedFrame(pose_in_odom, map_frame_, odom_frame_, &tf_listener_);
		insertPoseInPath(pose_in_map.toROSPose(), map_frame_, robot_odom_msg->header.stamp, true_path_);
		true_path_.header.stamp = ros::Time().now();
		true_path_.header.frame_id = map_frame_;
		true_path_pub_.publish(true_path_);
		// if(clouds_skipped_ >= num_clouds_skip_)
		// {
		// 	std::cout << "Ground truth:\n" << pose_in_map.toStringQuat("   ");
		// }
	}
	else
	{
		ROS_ERROR("Transform from odom to map frame not available");
		// return;
	}

	Pose6DOF pose_debug = pose_in_odom - rodom_first_pose_;
	int num_poses = robot_odom_poses_.size();
	if(num_poses == 0)
	{
		rodom_first_pose_ = pose_in_odom;
		robot_odom_poses_.push_back(Pose6DOF::getIdentity());
		insertPoseInPath(Pose6DOF::getIdentity().toROSPose(), map_frame_, robot_odom_msg->header.stamp, robot_odom_path_);
		odom_inited_ = true;
	}
	else
	{
		Pose6DOF prev_pose = getLatestPoseRobotOdometry();
		odom_latest_transform_ = pose_debug - prev_pose;
		robot_odom_poses_.push_back(pose_debug);
		if(verbosity_level_ >= 2)
		{
			std::cout << "Robot odometry pose:\n" << pose_debug.toStringQuat("   ");
			std::cout << "Robot odometry transform:\n" << odom_latest_transform_.toStringQuat("   ");
			std::cout << std::endl;
		}


		Pose6DOF prev_pose_path(robot_odom_path_.poses[num_poses-1].pose); 
    	if(Pose6DOF::distanceEuclidean(pose_debug, prev_pose_path) < POSE_DIST_THRESH) 
		{
			return;
		}
		insertPoseInPath(pose_debug.toROSPose(), map_frame_, robot_odom_msg->header.stamp, robot_odom_path_);
		robot_odom_path_.header.stamp = ros::Time().now();
		robot_odom_path_.header.frame_id = map_frame_;
		robot_odom_path_pub_.publish(robot_odom_path_);

	}

	new_transform_ = true;
}

