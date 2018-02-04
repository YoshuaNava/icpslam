
#ifndef ROBOT_ODOMETER_H
#define ROBOT_ODOMETER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
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

#include "utils/pose6DOF.h"
#include "utils/geometric_utils.h"


class RobotOdometer
{
private:
	// Constants
	const double POSE_DIST_THRESH = 0.1;

	int verbosity_level_;

	bool odom_inited_;

	// ROS node handle, URDF frames, topics and publishers
	ros::NodeHandle nh_;
	std::string laser_frame_, robot_frame_, odom_frame_, map_frame_;
	std::string robot_odom_topic_, robot_odom_path_topic_;
	ros::Publisher robot_odom_path_pub_;
	ros::Subscriber robot_odometry_sub_;

	// Debug topics and publishers
	std::string true_path_topic_;
	ros::Publisher true_path_pub_;

	// Odometry path containers
	nav_msgs::Path robot_odom_path_, true_path_;

	// Translations and rotations estimated by robot odometry
	bool new_transform_;
	Pose6DOF rodom_first_pose_, odom_latest_transform_;
	std::vector<Pose6DOF> robot_odom_poses_;
	tf::TransformListener tf_listener_;
	tf::TransformBroadcaster tf_broadcaster_;

public:

	RobotOdometer(ros::NodeHandle nh);

	void init();

	void loadParameters();

	void advertisePublishers();

	void registerSubscribers();

	bool isOdomReady();

	Pose6DOF getFirstPose();

 	Pose6DOF getLatestPose(); 

	void getEstimates(Pose6DOF *latest_odom_transform, Pose6DOF *odom_pose, bool *new_transform);

	void robotOdometryCallback(const nav_msgs::Odometry::ConstPtr& robot_odom_msg);
	
};

#endif