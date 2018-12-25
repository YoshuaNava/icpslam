
#include "icpslam/icpslam.h"

IcpSlam::IcpSlam(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh), robot_odometer_(nh, pnh), icp_odometer_(nh, pnh), octree_mapper_(nh, pnh) {
  ROS_INFO("#####       ICPSLAM         #####");
  mainLoop();
}

void IcpSlam::mainLoop() {
  ROS_INFO("#####       ICPSLAM         #####");

  int keyframes_window;
  nh_.param("keyframes_window", keyframes_window, 1);

  unsigned long curr_vertex_key = 0;
  bool run_pose_optimization = false, new_transform_icp = false, new_transform_rodom = false, is_keyframe = false;
  long num_keyframes = 0, num_vertices = 0, iter = 0;

  Pose6DOF robot_odom_transform, robot_odom_pose, prev_robot_odom_pose, icp_transform, icp_odom_pose, prev_icp_odom_pose,
      prev_keyframe_pose;

  // We start at the origin
  ROS_INFO("	Initial pose");
  prev_icp_odom_pose = Pose6DOF::getIdentity();
  prev_keyframe_pose = prev_icp_odom_pose;
  prev_robot_odom_pose = prev_icp_odom_pose;
  // pose_optimizer->setInitialPose(prev_icp_odom_pose);
  num_vertices++;

  while (ros::ok()) {
    if (icp_odometer_.isOdomReady() && robot_odometer_.isOdomReady()) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
      icp_odometer_.getEstimates(&cloud, &icp_transform, &icp_odom_pose, &new_transform_icp);
      if (new_transform_icp) {
        if (num_keyframes > 0) {
          Pose6DOF refined_transform;
          std::cout << "Original transform:\n" << icp_transform;
          bool icp_refined = octree_mapper_.refineTransformICP(cloud, prev_icp_odom_pose, &refined_transform);
          if (icp_refined) {
            // icp_transform = refined_transform;
            icp_odom_pose = prev_icp_odom_pose + icp_transform;
          }
        }

        is_keyframe = false;
        if ((Pose6DOF::distanceEuclidean(icp_odom_pose, prev_keyframe_pose) > KFS_DIST_THRESH) || (num_keyframes == 0)) {
          ROS_INFO("##### Number of keyframes = %lu", num_keyframes + 1);
          // ROS_INFO("  Keyframe inserted! ID %lu", curr_vertex_key+1);
          is_keyframe = true;
          num_keyframes++;
          prev_keyframe_pose = icp_odom_pose;
          if (num_keyframes % keyframes_window == 0)
            run_pose_optimization = true;
        } else {
          // ROS_INFO("  ICP vertex inserted! ID %lu", curr_vertex_key+1);
        }
        // pose_optimizer->addNewFactor(&cloud, icp_transform, icp_odom_pose, &curr_vertex_key, is_keyframe);
        num_vertices++;
        prev_robot_odom_pose = robot_odom_pose;
        new_transform_icp = false;
      }

      // robot_odometer.getEstimates(&robot_odom_transform, &robot_odom_pose, &new_transform_rodom);
      // if(new_transform_rodom)
      // 	if ((Pose6DOF::distanceEuclidean(robot_odom_pose, prev_robot_odom_pose) > VERTEX_DIST_THRESH) && (num_keyframes > 0))
      // 	{
      // 		ROS_INFO("  Odometry vertex inserted! ID %lu", curr_vertex_key+1);
      // 		pose_optimizer->addNewFactor(&cloud, robot_odom_transform, robot_odom_pose, &curr_vertex_key, false);
      // 		num_vertices++;
      // 		prev_robot_odom_pose = robot_odom_pose;
      // 		new_transform_rodom = false;
      // 	}

      if (run_pose_optimization) {
        // octree_mapper_.resetMap();
        // pose_optimizer->publishRefinedMap();
        run_pose_optimization = false;
      }

      // pose_optimizer->publishPoseGraphMarkers();

      prev_icp_odom_pose = icp_odom_pose;
      iter++;
    } else {
      Pose6DOF identity = Pose6DOF::getIdentity();
      // pose_optimizer->publishDebugTransform(identity, "debug", "odom");
    }
    ros::spinOnce();
  }
}
