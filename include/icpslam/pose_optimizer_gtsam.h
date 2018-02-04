
#ifndef POSE_OPTIMIZER_GTSAM_H
#define POSE_OPTIMIZER_GTSAM_H

#include "icpslam/pose_optimizer.h"

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include "utils/pose6DOF.h"
#include "utils/geometric_utils.h"

class PoseOptimizerGTSAM : public PoseOptimizer
{
protected:

    std::unique_ptr<gtsam::ISAM2> isam_;
    gtsam::Values graph_values_;

public:

    PoseOptimizerGTSAM(ros::NodeHandle nh);

    void init();

    void setInitialPose(Pose6DOF &initial_pose);

    void extendGraph(Pose6DOF &transform, Pose6DOF &pose, bool is_keyframe);

    void addNewFactor(PointCloud::Ptr *new_cloud_ptr, Pose6DOF transform, Pose6DOF pose, unsigned long *key, bool is_keyframe);

    void refinePoseGraph();

    bool checkLoopClosure();

    void publishDebugTransform(Pose6DOF robot_in_map, std::string parent_frame, std::string child_frame);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

#endif