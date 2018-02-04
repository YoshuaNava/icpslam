
#ifndef ICP_ODOMETER_H
#define ICP_ODOMETER_H

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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "utils/pose6DOF.h"
#include "utils/geometric_utils.h"


class ICPOdometer
{
private:
	// Constants
	const double ICP_FITNESS_THRESH = 0.1;
	const double ICP_MAX_CORR_DIST = 1.0;
	const double ICP_EPSILON = 1e-06;
	const double ICP_MAX_ITERS = 10;

	int verbosity_level_;

	bool odom_inited_;

	// ROS node handle, URDF frames, topics and publishers
	ros::NodeHandle nh_;
	std::string laser_frame_, robot_frame_, odom_frame_, map_frame_;
	std::string laser_cloud_topic_;
	ros::Subscriber laser_cloud_sub_;

	// Debug topics and publishers
	std::string prev_cloud_topic_, aligned_cloud_topic_, icp_odom_topic_, icp_odom_path_topic_;
	ros::Publisher prev_cloud_pub_, aligned_cloud_pub_, icp_odom_pub_, icp_odom_path_pub_;

	// PCL clouds
	pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud_, curr_cloud_;

	// Odometry path containers
	nav_msgs::Path icp_odom_path_;

	// Translations and rotations estimated by ICP
	bool new_transform_;
	int clouds_skipped_, num_clouds_skip_;
	bool aggregate_clouds_;
	Pose6DOF icp_latest_transform_;
	std::vector<Pose6DOF> icp_odom_poses_;
	tf::TransformListener tf_listener_;
	tf::TransformBroadcaster tf_broadcaster_;

public:

	ICPOdometer(ros::NodeHandle nh);

	void init();

	void loadParameters();

	void advertisePublishers();

	void registerSubscribers();

	bool isOdomReady();

	Pose6DOF getFirstPose();
	
	Pose6DOF getLatestPose();

	void getEstimates(pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud, Pose6DOF *latest_icp_transform, Pose6DOF *icp_pose, bool *new_transform);

	bool updateICPOdometry(Eigen::Matrix4d T);

	void laserCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
	
};

#endif