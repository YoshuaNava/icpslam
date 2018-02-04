
#include "icpslam/robot_odometer.h"
#include "icpslam/icp_odometer.h"
#include "icpslam/octree_mapper.h"
#include "icpslam/pose_optimizer_g2o.h"
#include "icpslam/pose_optimizer_gtsam.h"

const double KFS_DIST_THRESH = 0.3;
const double VERTEX_DIST_THRESH = 0.1;


int main(int argc, char** argv)
{
	ros::init(argc, argv, "icpslam");
	ros::NodeHandle nh("~");

	ROS_INFO("#####       ICPSLAM         #####");	
	RobotOdometer robot_odometer(nh);
	ICPOdometer icp_odometer(nh);
	OctreeMapper octree_mapper(nh);
	PoseOptimizerGTSAM* pose_optimizer = new PoseOptimizerGTSAM(nh);

	int keyframes_window = 3;
	unsigned long curr_vertex_key=0;
	bool run_pose_optimization = false,
		 new_transform_icp = false,
		 new_transform_rodom = false,
		 is_keyframe = false;
	long num_keyframes = 0,
		 num_vertices = 0,
		 iter = 0;

	Pose6DOF robot_odom_pose, 
			 prev_robot_odom_pose, 
			 icp_odom_pose, 
			 prev_keyframe_pose, 
			 icp_transform,
			 robot_odom_transform;

	while(ros::ok())
    {
		if(icp_odometer.isOdomReady() && robot_odometer.isOdomReady())
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

			icp_odometer.getEstimates(&cloud, &icp_transform, &icp_odom_pose, &new_transform_icp);
			robot_odometer.getEstimates(&robot_odom_transform, &robot_odom_pose, &new_transform_rodom);

			if(num_vertices == 0)
			{
				ROS_INFO("Iteration %lu", iter);
				ROS_INFO("	Initial pose inserted");
				Pose6DOF initial_pose = Pose6DOF::getIdentity();
				pose_optimizer->setInitialPose(initial_pose);
				num_vertices++;
				prev_keyframe_pose = icp_odom_pose;
				prev_robot_odom_pose = robot_odom_pose;
				continue;
			}
			
			if (new_transform_icp)
			{
				is_keyframe = false;
				if(Pose6DOF::distanceEuclidean(icp_odom_pose, prev_keyframe_pose) > KFS_DIST_THRESH)
				{
					if((Pose6DOF::distanceEuclidean(icp_odom_pose, prev_keyframe_pose) > KFS_DIST_THRESH) && (cloud->points.size()>0))
					{
						ROS_INFO("##### Number of keyframes = %lu", num_keyframes+1);
						ROS_INFO("  Keyframe inserted! ID %lu", curr_vertex_key+1);
						is_keyframe = true;
						num_keyframes++;
						prev_keyframe_pose = icp_odom_pose;
						if(num_keyframes % keyframes_window == 0)
							run_pose_optimization = true;
					}
					else 
					{
						ROS_INFO("  ICP vertex inserted! ID %lu", curr_vertex_key+1);
					}
					pose_optimizer->addNewFactor(&cloud, icp_transform, icp_odom_pose, &curr_vertex_key, is_keyframe);
					num_vertices++;
					prev_robot_odom_pose = robot_odom_pose;
					new_transform_icp = false;
				}
			}

			if(new_transform_rodom)
				if ((Pose6DOF::distanceEuclidean(robot_odom_pose, prev_robot_odom_pose) > VERTEX_DIST_THRESH) && (num_keyframes > 0))
				{
					ROS_INFO("  Odometry vertex inserted! ID %lu", curr_vertex_key+1);
					pose_optimizer->addNewFactor(&cloud, robot_odom_transform, robot_odom_pose, &curr_vertex_key, false);
					num_vertices++;
					prev_robot_odom_pose = robot_odom_pose;
					new_transform_rodom = false;
				}


			if(run_pose_optimization)
			{
				// octree_mapper.resetMap();
				pose_optimizer->publishRefinedMap();
				run_pose_optimization = false;	
			}
			
			pose_optimizer->publishPoseGraphMarkers();

			iter++;
		}
		else
		{
			Pose6DOF identity = Pose6DOF::getIdentity();
			pose_optimizer->publishDebugTransform(identity, "debug", "odom");
		}
        ros::spinOnce();
    }

	return EXIT_SUCCESS;
}