
#ifndef MAPPING_H
#define MAPPING_H

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

#include <pcl_ros/impl/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_search.h>

#include "utils/pose6DOF.h"
#include "utils/geometric_utils.h"
#include "utils/messaging_utils.h"



class OctreeMapper
{
private:
    // Constants for mapping
    const float OCTREE_RESOLUTION = 0.3;
    const float MAX_INCREMENTS_QUEUE = 30;

    int verbosity_level_;

    // Frames, topics and publishers
    ros::NodeHandle nh_;
    std::string laser_frame_, robot_frame_, odom_frame_, map_frame_;
    std::string map_cloud_topic_, increment_cloud_topic_;
    ros::Publisher map_cloud_pub_;
    ros::Subscriber increment_cloud_sub_;

    // tf handlers
    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;

    // PCL clouds for mapping
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;
    pcl::octree::OctreePointCloud<pcl::PointXYZ>::Ptr map_octree_;


public:

    OctreeMapper(ros::NodeHandle nh);

    void init();

    void loadParameters();

    void advertisePublishers();

    void registerSubscribers();

    void resetMap();

    /* This function is inspired on https://github.com/erik-nelson/point_cloud_mapper */
    void addPointsToMap(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);

    void incrementCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);

};


#endif