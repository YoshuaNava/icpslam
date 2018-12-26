
#include "icpslam/icpslam.h"

#include "utils/geometric_utils.h"

IcpSlam::IcpSlam(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
    : nh_(nh),
      pnh_(pnh),
      robot_odometer_(nh, pnh),
      icp_odometer_(nh, pnh),
      octree_mapper_(nh, pnh),
      pose_graph_(new pose_graph_utils::PoseGraphG2O()),
      T_map_to_odom_(Eigen::Isometry3d::Identity()) {
  loadParameters();
  mainLoop();
}

void IcpSlam::loadParameters() {
  pnh_.param("map_frame", map_frame_, std::string("map"));
  pnh_.param("odom_frame", odom_frame_, std::string("odom"));
  pnh_.param("robot_frame", robot_frame_, std::string("base_link"));
  pnh_.param("laser_frame", laser_frame_, std::string("laser"));

  nh_.param("keyframes_window", keyframes_window_, 1);
}

void IcpSlam::addNewKeyframe(
    const ros::Time& stamp, const Pose6DOF& pose_in_odom, const Pose6DOF& pose_in_map, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
  std::cout << "hola" << std::endl;
}

void IcpSlam::computeMapToOdomTransform() {
  if (keyframes_.size() > 0) {
    const auto& keyframe = keyframes_.back();
    T_map_to_odom_ = keyframe->graph_node_->estimate() * keyframe->pose_in_odom_.toEigenIsometry3().inverse();
  } else {
    T_map_to_odom_ = Eigen::Isometry3d::Identity();
  }
}

void IcpSlam::publishMapToOdomTf(const ros::Time& stamp, const Pose6DOF& pose) {
  // Publish map to odom tf
  computeMapToOdomTransform();
  geometry_msgs::TransformStamped map_to_odom_tf =
      getTfStampedFromEigenMatrix(stamp, T_map_to_odom_.matrix().cast<float>(), map_frame_, odom_frame_);
  tf_broadcaster_.sendTransform(map_to_odom_tf);

  // Publish pose in map tf
  Eigen::Isometry3d pose_in_map(Eigen::Isometry3d::Identity());
  if (keyframes_.size() > 0) {
    const auto& latest_keyframe = keyframes_.back();
    pose_in_map = latest_keyframe->graph_node_->estimate();
  }
  geometry_msgs::TransformStamped pose_in_map_tf =
      getTfStampedFromEigenMatrix(stamp, pose_in_map.matrix().cast<float>(), map_frame_, robot_frame_);
  tf_broadcaster_.sendTransform(pose_in_map_tf);
}

void IcpSlam::mainLoop() {
  ROS_INFO("IcpSlam: Main loop started");

  unsigned long curr_vertex_key = 0;
  bool run_pose_optimization = false, new_transform_icp = false, new_transform_rodom = false, is_keyframe = false;
  long num_keyframes = 0, num_vertices = 0, iter = 0;

  Pose6DOF robot_odom_transform, robot_odom_pose, prev_robot_odom_pose, icp_transform, icp_odom_pose, prev_icp_odom_pose,
      prev_keyframe_pose;

  // We start at the origin
  ROS_INFO("IcpSlam:	Initial pose");
  prev_icp_odom_pose = Pose6DOF::getIdentity();
  prev_keyframe_pose = prev_icp_odom_pose;
  prev_robot_odom_pose = prev_icp_odom_pose;
  // pose_graph_->addSe3Node(Pose6DOF::getIdentity().toEigenIsometry3());
  // pose_optimizer->setInitialPose(Pose6DOF::getIdentity());
  num_vertices++;

  while (ros::ok()) {
    if (icp_odometer_.isOdomReady() && robot_odometer_.isOdomReady()) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
      icp_odometer_.getEstimates(&cloud, &icp_transform, &icp_odom_pose, &new_transform_icp);
      if (new_transform_icp) {
        if (num_keyframes > 0) {
          Pose6DOF refined_transform;
          // std::cout << "Original transform:\n" << icp_transform;
          bool icp_refined = octree_mapper_.refineTransformICP(cloud, prev_icp_odom_pose, &refined_transform);
          if (icp_refined) {
            // icp_transform = refined_transform;
            icp_odom_pose = prev_icp_odom_pose + icp_transform;
          }
        }

        is_keyframe = false;
        if ((Pose6DOF::distanceEuclidean(icp_odom_pose, prev_keyframe_pose) > KFS_DIST_THRESH) || (num_keyframes == 0)) {
          ROS_INFO("IcpSlam: ##### Number of keyframes = %lu", num_keyframes + 1);
          // ROS_INFO("IcpSlam:  Keyframe inserted! ID %lu", curr_vertex_key+1);
          is_keyframe = true;
          num_keyframes++;
          prev_keyframe_pose = icp_odom_pose;
          if (num_keyframes % keyframes_window_ == 0)
            run_pose_optimization = true;
        } else {
          // ROS_INFO("IcpSlam:  ICP vertex inserted! ID %lu", curr_vertex_key+1);
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
      // 		ROS_INFO("IcpSlam:  Odometry vertex inserted! ID %lu", curr_vertex_key+1);
      // 		pose_optimizer->addNewFactor(&cloud, robot_odom_transform, robot_odom_pose, &curr_vertex_key, false);
      // 		num_vertices++;
      // 		prev_robot_odom_pose = robot_odom_pose;
      // 		new_transform_rodom = false;
      // 	}

      if (run_pose_optimization) {
        // octree_mapper_.resetMap();
        // pose_optimizer->publishRefinedMap();
        pose_graph_->optimize();
        run_pose_optimization = false;
      }

      // pose_optimizer->publishPoseGraphMarkers();

      prev_icp_odom_pose = icp_odom_pose;
      iter++;
    }
    ros::spinOnce();
  }
}
