
#include "icpslam/icpslam.h"

#include <visualization_msgs/MarkerArray.h>

#include "utils/geometric_utils.h"

IcpSlam::IcpSlam(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
    : nh_(nh),
      pnh_(pnh),
      robot_odometer_(nh, pnh),
      icp_odometer_(nh, pnh),
      octree_mapper_(nh, pnh),
      pose_graph_(new pose_graph_utils::PoseGraphG2O()),
      icp_inf_matrix_(Eigen::MatrixXd::Identity(6, 6)),
      loop_inf_matrix_(Eigen::MatrixXd::Identity(6, 6)),
      wheel_odom_inf_matrix_(Eigen::MatrixXd::Identity(6, 6)),
      num_keyframes_(0),
      T_map_to_odom_(Eigen::Isometry3d::Identity()),
      publish_map_transform_(true) {
  loadParameters();
  advertisePublishers();
  mainLoop();
}

void IcpSlam::loadParameters() {
  // Information matrices for pose graph
  std::vector<float> aux_vector;
  pnh_.getParam("icp_information_matrix", aux_vector);
  for (size_t i = 0; i < aux_vector.size(); i++) {
    icp_inf_matrix_(i, i) = aux_vector[i];
  }
  pnh_.getParam("loop_information_matrix", aux_vector);
  for (size_t i = 0; i < aux_vector.size(); i++) {
    loop_inf_matrix_(i, i) = aux_vector[i];
  }
  pnh_.getParam("wheel_odom_information_matrix", aux_vector);
  for (size_t i = 0; i < aux_vector.size(); i++) {
    wheel_odom_inf_matrix_(i, i) = aux_vector[i];
  }

  // Visualization markers
  marker_scale_nodes_ = pnh_.param<double>("marker_scale_nodes", 1.0);
  marker_scale_edges_ = pnh_.param<double>("marker_scale_edges", 0.1);
  if (!pnh_.getParam("icp_nodes_markers_color", icp_nodes_markers_color_)) {
    icp_nodes_markers_color_ = {0, 1.0, 0, 1.0};
  }
  if (!pnh_.getParam("odom_nodes_markers_color", odom_nodes_markers_color_)) {
    odom_nodes_markers_color_ = {0, 0.5, 0.5, 1.0};
  }
  if (!pnh_.getParam("edges_markers_color", edges_markers_color_)) {
    edges_markers_color_ = {0, 0, 1.0, 1.0};
  }

  // Tf frames
  pnh_.param("map_frame", map_frame_, std::string("map"));
  pnh_.param("odom_frame", odom_frame_, std::string("odom"));
  pnh_.param("robot_frame", robot_frame_, std::string("base_link"));
  pnh_.param("laser_frame", laser_frame_, std::string("laser"));
  pnh_.param("publish_map_transform", publish_map_transform_, true);

  // Slam parameters
  pnh_.param("keyframes_window", keyframes_window_, 1);
}

void IcpSlam::advertisePublishers() {
  markers_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("pose_graph", 1, true);
}

void IcpSlam::addNewKeyframe(
    const unsigned long& id, const ros::Time& stamp, const Pose6DOF& pose_in_odom, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
  // Create new keyframe
  Keyframe<pcl::PointXYZ>::Ptr new_keyframe(new Keyframe<pcl::PointXYZ>(id, stamp, pose_in_odom, cloud));

  // Add vertex to pose graph
  const Eigen::Isometry3d T_map_to_robot = T_map_to_odom_ * pose_in_odom.toEigenIsometry3();
  new_keyframe->graph_node_ = pose_graph_->addSe3Node(T_map_to_robot);

  if (num_keyframes_ > 0) {
    // Add edge between newest and previous keyframe
    const auto& prev_keyframe = keyframes_list_.back();
    Pose6DOF relative_pose = new_keyframe->pose_in_odom_.inverse() + prev_keyframe->pose_in_odom_;
    Eigen::Isometry3d T_between_keyframes = relative_pose.toEigenIsometry3();
    pose_graph_->addSe3Edge(new_keyframe->graph_node_, prev_keyframe->graph_node_, T_between_keyframes, icp_inf_matrix_);
  }

  // Add new keyframe to list
  keyframes_list_.push_back(new_keyframe);
}

void IcpSlam::computeMapToOdomTransform() {
  if (keyframes_list_.size() > 0) {
    const auto& keyframe = keyframes_list_.back();
    T_map_to_odom_ = keyframe->graph_node_->estimate() * keyframe->pose_in_odom_.toEigenIsometry3().inverse();
  } else {
    T_map_to_odom_ = Eigen::Isometry3d::Identity();
  }
}

void IcpSlam::publishMapToOdomTf(const ros::Time& stamp) {
  computeMapToOdomTransform();
  geometry_msgs::TransformStamped map_to_odom_tf =
      getTfStampedFromEigenMatrix(ros::Time::now(), T_map_to_odom_.matrix().cast<float>(), map_frame_, odom_frame_);
  tf_broadcaster_.sendTransform(map_to_odom_tf);
}

void IcpSlam::mainLoop() {
  ROS_INFO("IcpSlam: Main loop started");

  bool run_pose_optimization = false, new_transform_icp = false, new_transform_rodom = false;
  unsigned long num_vertices = 1, iter = 0;

  Pose6DOF robot_odom_transform, robot_odom_pose, prev_robot_odom_pose, icp_transform, icp_odom_pose, prev_icp_odom_pose, prev_icp_pose;

  // We start at the origin
  ROS_INFO("IcpSlam:	Initial pose");
  prev_icp_odom_pose = Pose6DOF::getIdentity();
  prev_icp_pose = prev_icp_odom_pose;
  prev_robot_odom_pose = prev_icp_odom_pose;
  ros::Time latest_stamp = ros::Time::now();

  while (ros::ok()) {
    computeMapToOdomTransform();
    if (publish_map_transform_) {
      publishMapToOdomTf();
    }

    if (icp_odometer_.isOdomReady()) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
      icp_odometer_.getEstimates(latest_stamp, cloud, icp_transform, icp_odom_pose, new_transform_icp);
      if (new_transform_icp) {
        if (num_keyframes_ > 0) {
          Pose6DOF refined_transform;
          ROS_DEBUG("IcpSlam: Transform refinement using nearest neighbor search");
          bool registration_success = octree_mapper_.refineTransformAndGrowMap(latest_stamp, cloud, prev_icp_odom_pose, refined_transform);
          if (registration_success) {
            ROS_DEBUG("IcpSlam: Refinement successful");
            icp_transform = refined_transform;
          }
          icp_odom_pose = prev_icp_odom_pose + icp_transform;
        }

        if ((Pose6DOF::distanceEuclidean(icp_odom_pose, prev_icp_pose) > KFS_DIST_THRESH) || (num_keyframes_ == 0)) {
          ROS_INFO("IcpSlam:    Number of keyframes = %lu", num_keyframes_ + 1);
          addNewKeyframe(num_keyframes_, latest_stamp, icp_odom_pose, cloud);
          num_keyframes_++;
          num_vertices++;
          prev_icp_pose = icp_odom_pose;
          if (num_keyframes_ % keyframes_window_ == 0) {
            run_pose_optimization = true;
          }
        }
        new_transform_icp = false;
      }

      if (robot_odometer_.isOdomReady()) {
        // robot_odometer.getEstimates(&robot_odom_transform, &robot_odom_pose, &new_transform_rodom);
        // if(new_transform_rodom)
        // 	if ((Pose6DOF::distanceEuclidean(robot_odom_pose, prev_robot_odom_pose) > VERTEX_DIST_THRESH) && (num_keyframes > 0))
        // 	{
        // Missing! -> add edge to pose graph ()
        // 		num_vertices++;
        // 		prev_robot_odom_pose = robot_odom_pose;
        // 		new_transform_rodom = false;
        // 	}
        // prev_robot_odom_pose = robot_odom_pose;
      }

      if (run_pose_optimization) {
        octree_mapper_.resetMap();
        pose_graph_->optimize(true);
        run_pose_optimization = false;
      }

      publishPoseGraphMarkers(ros::Time::now());

      prev_icp_odom_pose = icp_odom_pose;
    } else {
      // Bootstrap the icp odometer with the robot odometry initial pose
      if (robot_odometer_.isOdomReady()) {
        robot_odometer_.getEstimates(&robot_odom_transform, &robot_odom_pose, &new_transform_rodom);
        icp_odometer_.setInitialPose(robot_odom_pose);
      }
    }

    iter++;
    ros::spinOnce();
  }
}

void IcpSlam::publishPoseGraphMarkers(const ros::Time& stamp) {
  visualization_msgs::MarkerArray marker_array;
  marker_array.markers.resize(2);

  // keyframe nodes
  visualization_msgs::Marker& traj_marker = marker_array.markers[0];
  traj_marker.header.frame_id = map_frame_;
  traj_marker.header.stamp = stamp;
  traj_marker.ns = "icp_nodes";
  traj_marker.id = 0;
  traj_marker.type = visualization_msgs::Marker::SPHERE_LIST;

  traj_marker.pose.orientation.w = 1.0;
  traj_marker.scale.x = traj_marker.scale.y = traj_marker.scale.z = marker_scale_nodes_;

  traj_marker.points.resize(num_keyframes_);
  traj_marker.colors.resize(num_keyframes_);
  for (size_t i = 0; i < num_keyframes_; i++) {
    Eigen::Vector3d pos = keyframes_list_[i]->graph_node_->estimate().translation();
    traj_marker.points[i].x = pos.x();
    traj_marker.points[i].y = pos.y();
    traj_marker.points[i].z = pos.z();

    if (i == 0) {
      traj_marker.colors[i].b = 1.0;
      traj_marker.colors[i].r = 1.0;
      traj_marker.colors[i].g = 1.0;
      traj_marker.colors[i].a = 1.0;
    } else {
      traj_marker.colors[i].r = icp_nodes_markers_color_[0];
      traj_marker.colors[i].g = icp_nodes_markers_color_[1];
      traj_marker.colors[i].b = icp_nodes_markers_color_[2];
      traj_marker.colors[i].a = icp_nodes_markers_color_[3];
    }
  }

  // edge markers
  visualization_msgs::Marker& edge_marker = marker_array.markers[1];
  edge_marker.header.frame_id = map_frame_;
  edge_marker.header.stamp = stamp;
  edge_marker.ns = "edges";
  edge_marker.id = 1;
  edge_marker.type = visualization_msgs::Marker::LINE_LIST;

  edge_marker.pose.orientation.w = 1.0;
  edge_marker.scale.x = marker_scale_edges_;

  size_t num_edges_graph = pose_graph_->optimizer->edges().size();
  edge_marker.points.resize(num_edges_graph * 2);
  edge_marker.colors.resize(num_edges_graph * 2);

  auto edge_itr = pose_graph_->optimizer->edges().begin();
  for (size_t i = 0; edge_itr != pose_graph_->optimizer->edges().end(); edge_itr++, i++) {
    g2o::HyperGraph::Edge* edge = *edge_itr;
    g2o::EdgeSE3* edge_se3 = dynamic_cast<g2o::EdgeSE3*>(edge);
    if (edge_se3) {
      g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(edge_se3->vertices()[0]);
      g2o::VertexSE3* v2 = dynamic_cast<g2o::VertexSE3*>(edge_se3->vertices()[1]);
      // Eigen::Isometry3d relative_pose = 
      Eigen::Vector3d pt1 = v1->estimate().translation();
      Eigen::Vector3d pt2 = v2->estimate().translation();

      edge_marker.points[i * 2].x = pt1.x();
      edge_marker.points[i * 2].y = pt1.y();
      edge_marker.points[i * 2].z = pt1.z();
      edge_marker.points[i * 2 + 1].x = pt2.x();
      edge_marker.points[i * 2 + 1].y = pt2.y();
      edge_marker.points[i * 2 + 1].z = pt2.z();

      double p1 = static_cast<double>(v1->id()) / pose_graph_->optimizer->vertices().size();
      double p2 = static_cast<double>(v2->id()) / pose_graph_->optimizer->vertices().size();

      if (std::abs(v1->id() - v2->id()) > 2) {
        edge_marker.points[i * 2].z += 0.5;
        edge_marker.points[i * 2 + 1].z += 0.5;
      }

      edge_marker.colors[i * 2].r = edges_markers_color_[0];
      edge_marker.colors[i * 2].g = edges_markers_color_[1] + p1;
      edge_marker.colors[i * 2].b = edges_markers_color_[2] - p1;
      edge_marker.colors[i * 2].a = edges_markers_color_[3];
      edge_marker.colors[i * 2 + 1].r = edges_markers_color_[0];
      edge_marker.colors[i * 2 + 1].g = edges_markers_color_[1] + p2;
      edge_marker.colors[i * 2 + 1].b = edges_markers_color_[2] - p2;
      edge_marker.colors[i * 2 + 1].a = edges_markers_color_[3];
    }
  }

  markers_pub_.publish(marker_array);
}
