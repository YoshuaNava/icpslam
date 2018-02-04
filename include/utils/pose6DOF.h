
#ifndef POSE6DOF_H
#define POSE6DOF_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Transform.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>


class Pose6DOF
{
private:
	const double EQUALITY_THRESH = 1e-10;

public:
	ros::Time time_stamp;
	Eigen::Vector3d pos;
	Eigen::Quaterniond rot;
	Eigen::Matrix<double, 6, 6> cov;

	Pose6DOF()
	{
		setIdentity();
	}

	Pose6DOF(Eigen::Matrix4d &T, ros::Time stamp = ros::Time(0))
	{
		setIdentity();
		time_stamp = stamp;
		this->fromTMatrix(T);
		cov = Eigen::MatrixXd::Identity(6,6);
	}

	Pose6DOF(Eigen::Matrix4d &T, std::string tgt_frame, std::string src_frame, tf::TransformListener *tf_listener, ros::Time stamp = ros::Time(0))
	{
		setIdentity();
		time_stamp = stamp;
		this->fromTMatrixInFixedFrame(T, tgt_frame, src_frame, tf_listener);
		cov = Eigen::MatrixXd::Identity(6,6);
	}

	Pose6DOF operator +(Pose6DOF &p2)
	{
		Pose6DOF p3 = compose(*this, p2);
		return p3;
	}

	Pose6DOF operator -(Pose6DOF &p2)
	{
		Pose6DOF p3 = subtract(p2, *this);
		return p3;
	}

	Pose6DOF& operator =(Pose6DOF pose)
	{
		time_stamp = pose.time_stamp;
		pos(0) = pose.pos(0);
		pos(1) = pose.pos(1);
		pos(2) = pose.pos(2);

		rot.x() = pose.rot.x();
		rot.y() = pose.rot.y();
		rot.z() = pose.rot.z();
		rot.w() = pose.rot.w();
		cov = pose.cov;
		
		return *this;
	}

	bool operator ==(const Pose6DOF &pose)
	{
		return (((this->pos - pose.pos).norm() < EQUALITY_THRESH) && (fabs(this->rot.dot(pose.rot)) < 1-EQUALITY_THRESH));
	}

	double norm()
	{
		return pos.norm();
	}

	friend std::ostream& operator <<(std::ostream& os, const Pose6DOF& pose)
	{
		os << pose.toStringQuat();
		return os;
	}

	Pose6DOF inverse()
	{
		return inverse(*this);
	}

	Pose6DOF compose(Pose6DOF &p2)
	{
		Pose6DOF p3 = compose(*this, p2);
		pos = p3.pos;
		rot = p3.rot;
		return *this;
	}

	Pose6DOF subtract(Pose6DOF &p2)
	{
		Pose6DOF p3 = subtract(*this, p2);
		pos = p3.pos;
		rot = p3.rot;
		return *this;
	}

	Pose6DOF setIdentity()
	{
		time_stamp = ros::Time(0);
		pos = Eigen::Vector3d(0,0,0);
		rot = Eigen::Quaterniond(1,0,0,0);
		cov = Eigen::MatrixXd::Identity(6,6);
		return *this;
	}

	double distanceEuclidean(Pose6DOF p2)
	{
		return distanceEuclidean(*this, p2);
	}

	static double distanceEuclidean(Pose6DOF &p1, Pose6DOF &p2)
	{
		return subtract(p1, p2).pos.norm();
	}

	static Pose6DOF compose(Pose6DOF &p1, Pose6DOF &p2)
	{
		Pose6DOF p3;
		p3.time_stamp = p2.time_stamp;
		p3.pos = p1.pos + p1.rot.toRotationMatrix() * p2.pos;
		p3.rot = p1.rot * p2.rot;
		p3.rot.normalize();
		return p3;
	}

	static Pose6DOF subtract(Pose6DOF &p1, Pose6DOF &p2)
	{
		Pose6DOF p3;
		p3.time_stamp = p1.time_stamp;
		p3.pos = p1.rot.inverse() * (p2.pos - p1.pos);
		p3.rot = p2.rot * p1.rot.inverse();
		p3.rot.normalize();
		p3.cov = p1.cov;
		return p3;
	}

	static Pose6DOF inverse(Pose6DOF &pose)
	{
		Pose6DOF inv;
		inv.pos = (pose.rot.inverse() *  pose.pos) * -1.0;
		inv.rot = pose.rot.inverse();
		return inv;
	}

	static Pose6DOF getIdentity()
	{
		Pose6DOF pose;
		pose.time_stamp = ros::Time().now();
		pose.pos = Eigen::Vector3d(0,0,0);
		pose.rot = Eigen::Quaterniond(1,0,0,0);
		pose.cov = Eigen::MatrixXd::Identity(6,6);
		return pose;
	}

	std::string toStringQuat(std::string indent = "") const
	{
		std::string output;
		output += indent + "Time stamp = " + std::to_string(time_stamp.toSec()) + "\n";
		output += indent + "Position = (" + std::to_string(pos(0)) + ", " + std::to_string(pos(1)) + ", " + std::to_string(pos(2)) + ") with Norm = " + std::to_string(pos.norm()) + "\n";
		output += indent + "Rotation (q) = (" + std::to_string(rot.w()) + ", " + std::to_string(rot.x()) + ", " + std::to_string(rot.y()) + ", " + std::to_string(rot.z()) + ") with Norm = " + std::to_string(rot.norm()) + "\n";
		
		return output;
	}

	std::string toStringRPY(std::string indent = "") const
	{
		std::string output;
		output += indent + "Time stamp = " + std::to_string(time_stamp.toSec()) + "\n";
		output += indent + "Position = (" + std::to_string(pos(0)) + ", " + std::to_string(pos(1)) + ", " + std::to_string(pos(2)) + ") with Norm = " + std::to_string(pos.norm()) + "\n";
		Eigen::Vector3d rpy = rot.toRotationMatrix().eulerAngles(0, 1, 2);
		output += indent + "Rot (RPY) = (" + std::to_string(rpy(0)) + ", " + std::to_string(rpy(1)) + ", " + std::to_string(rpy(2)) + ") with Norm = " + std::to_string(rpy.norm()) + "\n";
		
		return output;
	}

	void transformToFixedFrame(std::string tgt_frame, std::string src_frame, tf::TransformListener *tf_listener)
	{
		Pose6DOF pose_in_tgt = transformToFixedFrame(*this, tgt_frame, src_frame, tf_listener);
		pos = pose_in_tgt.pos;
		rot = pose_in_tgt.rot;
	}

	static Pose6DOF transformToFixedFrame(Pose6DOF pose_in_src, std::string tgt_frame, std::string src_frame, tf::TransformListener *tf_listener)
	{
		tf::Pose tf_in_src(tf::Quaternion(pose_in_src.rot.x(), pose_in_src.rot.y(), pose_in_src.rot.z(), pose_in_src.rot.w()), 
			tf::Vector3(pose_in_src.pos(0), pose_in_src.pos(1), pose_in_src.pos(2)));
		tf::Stamped<tf::Pose> tf_in_tgt;
		try
		{
			tf_listener->transformPose(tgt_frame, tf::Stamped<tf::Pose>(tf_in_src, ros::Time(0), src_frame), tf_in_tgt);
		}
		catch(tf::TransformException e)
		{ }
		
		Pose6DOF pose_in_tgt(tf_in_tgt, pose_in_src.time_stamp);
		pose_in_tgt.cov = pose_in_src.cov;
		return pose_in_tgt;
	}

	Pose6DOF(geometry_msgs::Pose &pose_msg, ros::Time stamp = ros::Time(0))
	{
		setIdentity();
		time_stamp = stamp;
		this->fromROSPose(pose_msg);
		cov = Eigen::MatrixXd::Identity(6,6);
	}

	Pose6DOF(geometry_msgs::Pose &pose_msg, std::string tgt_frame, std::string src_frame, tf::TransformListener *tf_listener, ros::Time stamp = ros::Time(0))
	{
		setIdentity();
		time_stamp = stamp;
		this->fromROSPoseInFixedFrame(pose_msg, tgt_frame, src_frame, tf_listener);
		cov = Eigen::MatrixXd::Identity(6,6);
	}

	Pose6DOF(geometry_msgs::PoseWithCovariance &pose_msg, ros::Time stamp = ros::Time(0))
	{
		setIdentity();
		time_stamp = stamp;
		this->fromROSPoseWithCovariance(pose_msg);
	}
	
	Pose6DOF(tf::Transform &transform, ros::Time stamp = ros::Time(0))
	{
		setIdentity();
		time_stamp = stamp;
		this->fromTFTransform(transform);
		cov = Eigen::MatrixXd::Identity(6,6);
	}

	Pose6DOF(geometry_msgs::Transform &transform, ros::Time stamp = ros::Time(0))
	{
		setIdentity();
		time_stamp = stamp;
		this->fromROSTransform(transform);
		cov = Eigen::MatrixXd::Identity(6,6);
	}

	void fromTMatrix(Eigen::Matrix4d &T)
	{
		pos = Eigen::Vector3d(T(0,3), T(1,3), T(2,3));
		Eigen::Matrix3d R = T.block(0, 0, 3, 3); //.transpose();
		rot = Eigen::Quaterniond(R);
		rot.normalize();
	}

	void fromTMatrixInFixedFrame(Eigen::Matrix4d &T, std::string tgt_frame, std::string src_frame, tf::TransformListener *tf_listener)
	{
		Pose6DOF pose_T_in_src(T);
		transformToFixedFrame(tgt_frame, src_frame, tf_listener);
	}

	void fromROSPoseInFixedFrame(geometry_msgs::Pose pose_msg, std::string tgt_frame, std::string src_frame, tf::TransformListener *tf_listener)
	{
		Pose6DOF pose_in_src(pose_msg);
		transformToFixedFrame(tgt_frame, src_frame, tf_listener);
	}

	void fromROSPose(geometry_msgs::Pose &pose)
	{
		pos = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
		rot = Eigen::Quaterniond(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
		rot.normalize();
	}

	void fromROSPoseWithCovariance(geometry_msgs::PoseWithCovariance &pose_cov)
	{
		pos = Eigen::Vector3d(pose_cov.pose.position.x, pose_cov.pose.position.y, pose_cov.pose.position.z);
		rot = Eigen::Quaterniond(pose_cov.pose.orientation.w, pose_cov.pose.orientation.x, pose_cov.pose.orientation.y, pose_cov.pose.orientation.z);
		rot.normalize();
		double* cov_arr = pose_cov.covariance.data();
		cov = Eigen::Matrix<double, 6, 6>(cov_arr);
	}

	void fromROSTransform(geometry_msgs::Transform &transform)
	{
		pos = Eigen::Vector3d(transform.translation.x, transform.translation.y, transform.translation.z);
		rot = Eigen::Quaterniond(transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z);
		rot.normalize();
	}

	void fromTFPose(tf::Pose &pose)
	{
		pos = Eigen::Vector3d(pose.getOrigin().getX(), pose.getOrigin().getY(), pose.getOrigin().getZ());
		rot = Eigen::Quaterniond(pose.getRotation().w(), pose.getRotation().x(), pose.getRotation().y(), pose.getRotation().z());
		rot.normalize();
	}
	
	void fromTFTransform(tf::Transform &transform)
	{
		pos = Eigen::Vector3d(transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());
		rot = Eigen::Quaterniond(transform.getRotation().w(), transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z());
		rot.normalize();
	}

	Eigen::Matrix4d toTMatrix()
	{
		Eigen::Matrix4d T;
		T.setIdentity();
		T.block<3,3>(0,0) = rot.toRotationMatrix();
		T.block<3,1>(0,2) = pos;
		return T;
	}

	tf::Transform toTFTransform()
	{
		tf::Transform transform;
		transform.setOrigin(tf::Vector3(pos(0), pos(1), pos(2)));
		transform.setRotation(tf::Quaternion(rot.x(), rot.y(), rot.z(), rot.w()));
		return transform;
	}

	tf::Pose toTFPose()
	{
		tf::Pose pose;
		pose.setOrigin(tf::Vector3(pos(0), pos(1), pos(2)));
		pose.setRotation(tf::Quaternion(rot.x(), rot.y(), rot.z(), rot.w()));
		return pose;
	}

	geometry_msgs::Point toROSPoint()
	{
		geometry_msgs::Point p;
		p.x = pos(0);
		p.y = pos(1);
		p.z = pos(2);

		return p;
	}

	geometry_msgs::Pose toROSPose()
	{
		geometry_msgs::Pose pose;
		pose.position.x = pos(0);
		pose.position.y = pos(1);
		pose.position.z = pos(2);
		pose.orientation.x = rot.x();
		pose.orientation.y = rot.y();
		pose.orientation.z = rot.z();
		pose.orientation.w = rot.w();
		return pose;
	}

	geometry_msgs::PoseWithCovariance toROSPoseWithCovariance()
	{
		geometry_msgs::PoseWithCovariance pose_cov;
		pose_cov.pose.position.x = pos(0);
		pose_cov.pose.position.y = pos(1);
		pose_cov.pose.position.z = pos(2);
		pose_cov.pose.orientation.x = rot.x();
		pose_cov.pose.orientation.y = rot.y();
		pose_cov.pose.orientation.z = rot.z();
		pose_cov.pose.orientation.w = rot.w();
		float *cov_arr;
		Eigen::Map<Eigen::Matrix<float,6,6>>(cov_arr, cov.rows(), cov.cols()) = cov.cast<float>();
		for(size_t i=0; i<36 ;i++)
		{
			pose_cov.covariance[i] = cov_arr[i];
		}
		return pose_cov;
	}
};




#endif