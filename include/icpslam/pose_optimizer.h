
#ifndef POSE_OPTIMIZER_H
#define POSE_OPTIMIZER_H

#include <random>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <pcl_ros/impl/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include "utils/pose6DOF.h"
#include "utils/geometric_utils.h"
#include "utils/messaging_utils.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class PoseOptimizer
{
protected:

    int verbosity_level_;

    unsigned long curr_vertex_key_, curr_edge_key_;
    std::map<unsigned long, PointCloud::Ptr> graph_scans_;
    std::map<unsigned long, Pose6DOF> graph_poses_;
    std::map<unsigned long, std::pair<unsigned long, unsigned long>> graph_edges_;
    Pose6DOF latest_pose;
    int pose_opt_iters;

    ros::NodeHandle nh_;
    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;
    ros::Timer map_transform_timer_;
    std::string namespace_, graph_edges_topic_, graph_vertices_topic_, graph_keyframes_topic_, increment_cloud_topic_;
    std::string laser_frame_, robot_frame_, odom_frame_, map_frame_;
    ros::Publisher graph_edges_pub_, graph_vertices_pub_, graph_keyframes_pub_, increment_cloud_pub_;
    bool publish_map_transform_;
    
    std_msgs::ColorRGBA vertex_color_;
    std_msgs::ColorRGBA odom_edge_color_;
    std_msgs::ColorRGBA closure_edge_color_;
    std_msgs::ColorRGBA keyframes_color_;
    geometry_msgs::Vector3 keyframes_scale_;
    geometry_msgs::Vector3 vertex_scale_;
    geometry_msgs::Vector3 closure_edge_scale_;
    geometry_msgs::Vector3 odom_edge_scale_;


public:

    PoseOptimizer()
    { }

    PoseOptimizer(ros::NodeHandle nh) :
        nh_(nh)
    { }

    virtual void init()=0;

    virtual void refinePoseGraph()=0;

    virtual bool checkLoopClosure()=0;

    Pose6DOF getStartPose()
    {
        return graph_poses_.find(0)->second;
    }

    Pose6DOF getLatestPose()
    {
        return latest_pose;
    }

    void loadParameters()
    {
        nh_.param("verbosity_level_", verbosity_level_, 1);

        nh_.param("map_frame", map_frame_, std::string("map"));
        nh_.param("odom_frame", odom_frame_, std::string("odom"));
        nh_.param("robot_frame", robot_frame_, std::string("base_link"));
        nh_.param("laser_frame", laser_frame_, std::string("laser"));
        nh_.param("publish_map_transform", publish_map_transform_, true);

        nh_.param("graph_edges_topic", graph_edges_topic_, std::string("icpslam/graph_edges"));
        nh_.param("graph_vertices_topic", graph_vertices_topic_, std::string("icpslam/graph_vertices"));
        nh_.param("graph_keyframes_topic", graph_keyframes_topic_, std::string("icpslam/graph_keyframes"));
        nh_.param("increment_cloud_topic", increment_cloud_topic_, std::string("icpslam/increment_cloud"));
    }

    void advertisePublishers()
    {
        graph_edges_pub_ = nh_.advertise<visualization_msgs::Marker>(graph_edges_topic_, 1);
        graph_vertices_pub_ = nh_.advertise<visualization_msgs::Marker>(graph_vertices_topic_, 1);
        graph_keyframes_pub_ = nh_.advertise<visualization_msgs::Marker>(graph_keyframes_topic_, 1);
        increment_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(increment_cloud_topic_, 1);

        if(publish_map_transform_)
            map_transform_timer_ = nh_.createTimer(ros::Duration(0.01), &PoseOptimizer::mapTransformCallback, this);

        setGraphMarkersProperties();
    }

    void setGraphMarkersProperties()
    {
        vertex_color_.r = 0;
        vertex_color_.g = 0;
        vertex_color_.b = 1;
        vertex_color_.a = 1;
        odom_edge_color_.r = 1;
        odom_edge_color_.g = 0;
        odom_edge_color_.b = 0;
        odom_edge_color_.a = 1;
        closure_edge_color_.r = 1;
        closure_edge_color_.g = 1;
        closure_edge_color_.b = 1;
        closure_edge_color_.a = 1;
        keyframes_color_.r = 0;
        keyframes_color_.g = 1;
        keyframes_color_.b = 0;
        keyframes_color_.a = 1;

        keyframes_scale_.x = 0.15;
        keyframes_scale_.y = 0.15;
        keyframes_scale_.z = 0.15;
        vertex_scale_.x = 0.1;
        vertex_scale_.y = 0.1;
        vertex_scale_.z = 0.1;
        closure_edge_scale_.x = 0.05;
        odom_edge_scale_.x = 0.03;
    }

    void publishPoseGraphMarkers()
    {
        visualization_msgs::Marker edges_marker;
        edges_marker.header.frame_id = map_frame_;
        edges_marker.header.stamp = ros::Time().now();
        edges_marker.ns = namespace_;
        edges_marker.id = 0;
        edges_marker.action = visualization_msgs::Marker::ADD;
        edges_marker.type = visualization_msgs::Marker::LINE_LIST;
        edges_marker.color = odom_edge_color_;
        edges_marker.scale = odom_edge_scale_;
        edges_marker.pose.position.x = 0;
        edges_marker.pose.position.y = 0;
        edges_marker.pose.position.z = 0;
        edges_marker.pose.orientation.x = 0;
        edges_marker.pose.orientation.y = 0;
        edges_marker.pose.orientation.z = 0;
        edges_marker.pose.orientation.w = 1;
        for (size_t i = 0; i < curr_edge_key_; ++i)
        {
            uint vertex1_key = graph_edges_.at(i).first;
            uint vertex2_key = graph_edges_.at(i).second;

            geometry_msgs::Point point1, point2;
            point1 = graph_poses_.at(vertex1_key).toROSPoint();
            point2 = graph_poses_.at(vertex2_key).toROSPoint();

            edges_marker.points.push_back(point1);
            edges_marker.points.push_back(point2);
        }
        graph_edges_pub_.publish(edges_marker);
        

        visualization_msgs::Marker vertices_marker;
        vertices_marker.header.frame_id = map_frame_;
        vertices_marker.header.stamp = ros::Time().now();
        vertices_marker.ns = namespace_;
        vertices_marker.id = 1;
        vertices_marker.action = visualization_msgs::Marker::ADD;
        vertices_marker.type = visualization_msgs::Marker::SPHERE_LIST;
        vertices_marker.color = vertex_color_;
        vertices_marker.scale = vertex_scale_;
        vertices_marker.pose.position.x = 0;
        vertices_marker.pose.position.y = 0;
        vertices_marker.pose.position.z = 0;
        vertices_marker.pose.orientation.x = 0;
        vertices_marker.pose.orientation.y = 0;
        vertices_marker.pose.orientation.z = 0;
        vertices_marker.pose.orientation.w = 1;

        visualization_msgs::Marker keyframes_marker;
        keyframes_marker.header.frame_id = map_frame_;
        keyframes_marker.header.stamp = ros::Time().now();
        keyframes_marker.ns = namespace_;
        keyframes_marker.id = 2;
        keyframes_marker.action = visualization_msgs::Marker::ADD;
        keyframes_marker.type = visualization_msgs::Marker::SPHERE_LIST;
        keyframes_marker.color = keyframes_color_;
        keyframes_marker.scale = keyframes_scale_;
        keyframes_marker.pose.position.x = 0;
        keyframes_marker.pose.position.y = 0;
        keyframes_marker.pose.position.z = 0;
        keyframes_marker.pose.orientation.x = 0;
        keyframes_marker.pose.orientation.y = 0;
        keyframes_marker.pose.orientation.z = 0;
        keyframes_marker.pose.orientation.w = 1;

        for (size_t i = 0; i < curr_vertex_key_; ++i)
        {
            geometry_msgs::Point point = getROSPointFromPose6DOF(graph_poses_.at(i));

            if(graph_scans_.count(i) == 1)
                keyframes_marker.points.push_back(point);
            else
                vertices_marker.points.push_back(point);
        }

        if(sizeof(vertices_marker.points) > 0)
            graph_vertices_pub_.publish(vertices_marker);

        if(sizeof(keyframes_marker.points) > 0)
            graph_keyframes_pub_.publish(keyframes_marker);
        
    }

    void gazeboMapTransformCallback(const ros::TimerEvent&);

    void mapTransformCallback(const ros::TimerEvent&)
    {
        // ROS_INFO("Map transform callback");
        tf::Transform transform;
        if(graph_poses_.size() > 0)
        {
            tf::Pose robot_in_map(tf::Quaternion(latest_pose.rot.x(), latest_pose.rot.y(), latest_pose.rot.z(), latest_pose.rot.w()), 
                tf::Vector3(latest_pose.pos(0), latest_pose.pos(1), latest_pose.pos(2)));
            tf::Pose map_in_robot = robot_in_map;
            tf::Stamped<tf::Pose> map_in_odom;
            try
            {
                tf_listener_.transformPose(odom_frame_, tf::Stamped<tf::Pose>(map_in_robot, ros::Time(0), robot_frame_), map_in_odom);
            }
            catch(tf::TransformException e)
            { }

            transform.setOrigin( map_in_odom.getOrigin() );
            transform.setRotation( map_in_odom.getRotation() );
            tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), odom_frame_, map_frame_));
        }
    }

    void publishRefinedMap()
    {
        for (size_t i = 0; i < curr_vertex_key_; ++i)
        {
            if(graph_scans_.count(i) == 1)
            {
                Pose6DOF keyframe_pose = graph_poses_.at(i);
                PointCloud::Ptr cloud_ptr = graph_scans_.at(i);
                PointCloud::Ptr cloud_out_ptr(new PointCloud());

                tf::Transform tf_keyframe_in_map = keyframe_pose.toTFTransform();
                try
                {
                    pcl_ros::transformPointCloud(*cloud_ptr, *cloud_out_ptr, tf_keyframe_in_map);
                    publishPointCloud(cloud_out_ptr, map_frame_, ros::Time().now(), &increment_cloud_pub_);
                }
                catch(tf::TransformException e)
                { }
            }
        }
    }

};

#endif