
#include "utils/pose6DOF.h"
#include "icpslam/octree_mapper.h"


OctreeMapper::OctreeMapper(ros::NodeHandle nh) :
	nh_(nh),
	map_cloud_(new pcl::PointCloud<pcl::PointXYZ>()),
	map_octree_(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(OCTREE_RESOLUTION))
{
	init();
}

void OctreeMapper::init()
{
	resetMap();
	loadParameters();
	advertisePublishers();
	registerSubscribers();

	ROS_INFO("Octree mapper started");
}

void OctreeMapper::loadParameters()
{
	nh_.param("verbosity_level", verbosity_level_, 2);

	// TF frames
	nh_.param("map_frame", map_frame_, std::string("map"));
	nh_.param("odom_frame", odom_frame_, std::string("odom"));
	nh_.param("robot_frame", robot_frame_, std::string("base_link"));
	nh_.param("laser_frame", laser_frame_, std::string("laser"));

	// Input robot odometry and point cloud topics
	nh_.param("increment_cloud_topic", increment_cloud_topic_, std::string("icpslam/increment_cloud"));
	nh_.param("map_cloud_topic", map_cloud_topic_, std::string("icpslam/map_cloud"));
  nh_.param("nn_cloud_topic", nn_cloud_topic_, std::string("icpslam/nn_cloud"));
  nh_.param("refined_path_topic", refined_path_topic_, std::string("icpslam/refined_path"));
}

void OctreeMapper::advertisePublishers()
{
	map_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(map_cloud_topic_, 1);
  nn_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(nn_cloud_topic_, 1);
  refined_path_pub_ = nh_.advertise<nav_msgs::Path>(refined_path_topic_, 1);
}

void OctreeMapper::registerSubscribers()
{
	increment_cloud_sub_ = nh_.subscribe(increment_cloud_topic_, 10, &OctreeMapper::incrementCloudCallback, this);
}

void OctreeMapper::resetMap()
{
	map_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
	map_octree_.reset(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(OCTREE_RESOLUTION));
	map_octree_->setInputCloud(map_cloud_);
}

/* This function is inspired on https://github.com/erik-nelson/point_cloud_mapper */
void OctreeMapper::addPointsToMap(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
{
	for(size_t i=0; i<input_cloud->points.size() ;i++)
	{
		pcl::PointXYZ point = input_cloud->points[i];
		if(!map_octree_->isVoxelOccupiedAtPoint(point))
		{
			map_octree_->addPointToCloud(point, map_cloud_);
		}
	}
}

void OctreeMapper::incrementCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
	ROS_INFO("Map increment cloud callback!");
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>()), new_points(new pcl::PointCloud<pcl::PointXYZ>());

	pcl::fromROSMsg(*cloud_msg, *input_cloud);

	addPointsToMap(input_cloud);

	publishPointCloud(map_cloud_, map_frame_, ros::Time().now(), &map_cloud_pub_);
}

// Credits to Erik Nelson, creator of BLAM! (https://github.com/erik-nelson/blam)
bool OctreeMapper::approxNearestNeighbors(const pcl::PointCloud<pcl::PointXYZ>& cloud, pcl::PointCloud<pcl::PointXYZ> *neighbors)
{
	neighbors->points.clear();

	// Iterate over points in the input point cloud, finding the nearest neighbor
	// for every point and storing it in the output array.
	for (size_t ii = 0; ii < cloud.points.size(); ++ii) 
  {
	// Search for nearest neighbor and store.
    float unused = 0.0f;
    int result_index = -1;

    map_octree_->approxNearestSearch(cloud.points[ii], result_index, unused);
    if (result_index >= 0)
      neighbors->push_back(map_cloud_->points[result_index]);
	}

	return neighbors->points.size() > 0;
}

void OctreeMapper::transformCloudToPoseFrame(const pcl::PointCloud<pcl::PointXYZ>& in_cloud, const Pose6DOF& pose, pcl::PointCloud<pcl::PointXYZ> *out_cloud)
{
  tf::Transform tf_cloud_in_pose = pose.toTFTransform();
  try
  {
      pcl_ros::transformPointCloud(in_cloud, *out_cloud, tf_cloud_in_pose);
  }
  catch(tf::TransformException e)
  { }
}

bool OctreeMapper::estimateTransformICP(const pcl::PointCloud<pcl::PointXYZ>::Ptr& curr_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& nn_cloud, Pose6DOF* transform)
{
  // ROS_INFO("Estimation of transform via ICP");
	pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setMaximumIterations(ICP_MAX_ITERS);
	icp.setTransformationEpsilon(ICP_EPSILON);
	icp.setMaxCorrespondenceDistance(ICP_MAX_CORR_DIST);
	icp.setRANSACIterations(0);
	icp.setInputSource(curr_cloud);
	icp.setInputTarget(nn_cloud);

	pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud_in_curr_frame(new pcl::PointCloud<pcl::PointXYZ>()),
										curr_cloud_in_prev_frame(new pcl::PointCloud<pcl::PointXYZ>()),
										joint_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	icp.align(*curr_cloud_in_prev_frame);
	Eigen::Matrix4d T = icp.getFinalTransformation().cast<double>();

	if(icp.hasConverged())
	{
    *transform = Pose6DOF(T, ros::Time().now());
    std::cout << "refined transform:\n" << *transform;
    return true;
  }

  return false;
}

void OctreeMapper::publishPath(const Pose6DOF &latest_pose)
{
	insertPoseInPath(latest_pose.toROSPose(), map_frame_, ros::Time().now(), refined_path_);
	refined_path_.header.stamp = ros::Time().now();
	refined_path_.header.frame_id = map_frame_;
	refined_path_pub_.publish(refined_path_);
}


bool OctreeMapper::refineTransformICP(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const Pose6DOF& prev_pose, Pose6DOF *transform)
{
  if(map_cloud_->points.size() == 0)
    return false;

  ROS_INFO("Transform refinement using nearest neighbor search");

  pcl::PointCloud<pcl::PointXYZ>::Ptr nn_cloud_in_map(new pcl::PointCloud<pcl::PointXYZ>()),
                                      nn_cloud(new pcl::PointCloud<pcl::PointXYZ>()),
                                      cloud_in_map(new pcl::PointCloud<pcl::PointXYZ>());

  transformCloudToPoseFrame(*cloud, prev_pose, &*cloud_in_map);
  approxNearestNeighbors(*cloud_in_map, &*nn_cloud_in_map);
  transformCloudToPoseFrame(*nn_cloud_in_map, prev_pose.inverse(), &*nn_cloud);

  if(verbosity_level_ >= 1)
    publishPointCloud(nn_cloud_in_map, map_frame_, ros::Time().now(), &nn_cloud_pub_);
    // publishPointCloud(cloud, robot_frame_, ros::Time().now(), &nn_cloud_pub_);

  if(estimateTransformICP(cloud, nn_cloud, transform))
  {
    publishPath(prev_pose + (*transform));
    return true;
  }

  return false;
}