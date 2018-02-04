
#include <ros/ros.h>
#include <gazebo_msgs/GetModelState.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include "utils/pose6DOF.h"
#include "utils/geometric_utils.h"


// Class for publishing the Gazebo absolute pos reference when tuning the algorithm
class GazeboReferenceFramePublisher
{
public:

    GazeboReferenceFramePublisher(ros::NodeHandle nh) :
        nh_(nh)
    {
        init();
    }

private:
    int verbosity_level_;

    ros::NodeHandle nh_;
    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;
    std::string robot_frame_, parent_frame_, child_frame_, robot_name_;
    ros::Timer ref_transform_timer_;
    ros::ServiceClient gazebo_transform_service_;


    void init()
    {
        nh_.param("verbosity_level_", verbosity_level_, 1);

        nh_.param("robot_name", robot_name_, std::string("ridgeback_yumi"));

        nh_.param("parent_frame", parent_frame_, std::string("world"));
        nh_.param("child_frame", child_frame_, std::string("map"));
        nh_.param("robot_frame", robot_frame_, std::string("base_link"));

        ROS_INFO("Gazebo reference transform will be published, frames %s -> %s", parent_frame_.c_str(), child_frame_.c_str());

        gazebo_transform_service_ = nh_.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
        ref_transform_timer_ = nh_.createTimer(ros::Duration(0.01), &GazeboReferenceFramePublisher::gazeboRefTransformCallback, this);
    }

    void gazeboRefTransformCallback(const ros::TimerEvent&)
    {
        // ROS_INFO("Gazebo absolute reference transform callback");
        gazebo_msgs::GetModelState srv;
        srv.request.model_name = robot_name_;

        if(gazebo_transform_service_.call(srv))
        {
            // Correct estimation of map->odom from map->robot
            tf::Pose robot_in_parent = getTFPoseFromROSPose(srv.response.pose);
            tf::Pose parent_in_robot = robot_in_parent.inverse();
            tf::Stamped<tf::Pose> parent_in_child;
            try
            {
                tf_listener_.transformPose(child_frame_, tf::Stamped<tf::Pose>(parent_in_robot, ros::Time(0), robot_frame_), parent_in_child);
            }
            catch(tf::TransformException e)
            { }

            tf::Transform transform;
            transform.setOrigin( parent_in_child.getOrigin() );
            transform.setRotation( parent_in_child.getRotation() );

            tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), child_frame_, parent_frame_));
        }
    }

};


int main(int argc, char** argv)
{
   	ros::init(argc, argv, "gazebo_ref_frame_publisher");
	ros::NodeHandle nh("~");

    GazeboReferenceFramePublisher gazebo_ref_publisher(nh);

    ros::spin();

    return EXIT_SUCCESS;

}