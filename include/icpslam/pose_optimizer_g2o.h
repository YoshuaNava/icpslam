
#ifndef POSE_OPTIMIZER_G2O_H
#define POSE_OPTIMIZER_G2O_H

#include "icpslam/pose_optimizer.h"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/types/icp/types_icp.h"
#include "g2o/types/slam3d/types_slam3d.h"

#include "utils/pose6DOF.h"
#include "utils/geometric_utils.h"

class PoseOptimizerG2O : public PoseOptimizer
{
protected:

    g2o::SparseOptimizer* optimizer_;


public:

    PoseOptimizerG2O(ros::NodeHandle nh);

    void init();

    void addNewKeyframeVertex(PointCloud::Ptr *new_cloud_ptr, Pose6DOF icp_transform, Pose6DOF pose, uint *key);

    void addNewOdometryVertex(PointCloud::Ptr *new_cloud_ptr, Pose6DOF pose, uint *key);

    void addNewEdge(Eigen::MatrixXd cov, uint vertex2_key, uint vertex1_key, uint *key);

    bool optimizeGraph();

    void refinePoseGraph();

    void refineVertices();

    void refineEdges();

    bool checkLoopClosure();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

#endif