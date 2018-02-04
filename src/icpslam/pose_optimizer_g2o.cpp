
#include "icpslam/pose_optimizer_g2o.h"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/types/icp/types_icp.h"
#include "g2o/types/slam3d/types_slam3d.h"

#include "utils/pose6DOF.h"
#include "utils/geometric_utils.h"
#include "utils/messaging_utils.h"


PoseOptimizerG2O::PoseOptimizerG2O(ros::NodeHandle nh) :
    PoseOptimizer(nh)
{
    init();
}

void PoseOptimizerG2O::init()
{
    curr_vertex_key_ = 0;
    curr_edge_key_ = 0;
    graph_scans_.clear();
    graph_poses_.clear();

    optimizer_ = new g2o::SparseOptimizer();

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<g2o::BlockSolver_6_3>(g2o::make_unique<g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>>()));

    optimizer_->setAlgorithm(solver);
    optimizer_->setVerbose(true);

    namespace_ = "icpslam";
    pose_opt_iters = 30;

    loadParameters();
    advertisePublishers();
    ROS_INFO("Pose optimizer initialized. Backend = g2o");
}

/* Inspired on mrsmap by Jurg Stuckler */
void PoseOptimizerG2O::addNewKeyframeVertex(PointCloud::Ptr *new_cloud_ptr, Pose6DOF icp_transform, Pose6DOF pose, uint *key)
{
    *key = curr_vertex_key_;
    graph_scans_.insert(std::pair<uint, PointCloud::Ptr>(*key, *new_cloud_ptr));
    graph_poses_.insert(std::pair<uint, Pose6DOF>(*key, pose));

    g2o::VertexSE3* v = new g2o::VertexSE3();
    v->setId(*key);
    g2o::SE3Quat se3_pose(pose.rot, pose.pos);
    v->setEstimate( se3_pose );
    // std::cout << "keyframe vertex \n" << se3_pose << std::endl;

    if(curr_vertex_key_ == 0)
    {
        ROS_ERROR("FIXED VERTEX");
        v->setFixed(true);
    }
    
    // v->setMarginalized(true);
    optimizer_->addVertex( v );
    
    curr_vertex_key_++;
}


/* Inspired on mrsmap by Jurg Stuckler */
void PoseOptimizerG2O::addNewOdometryVertex(PointCloud::Ptr *new_cloud_ptr, Pose6DOF pose, uint *key)
{
    *key = curr_vertex_key_;
    graph_poses_.insert(std::pair<uint, Pose6DOF>(*key, pose));

    if(graph_poses_.size() == 0)
        latest_pose = pose;

    g2o::VertexSE3* v = new g2o::VertexSE3();
    v->setId(*key);
    g2o::SE3Quat se3_pose(pose.rot, pose.pos);
    // std::cout << "odometry vertex \n" << se3_pose << std::endl;
    v->setEstimate( se3_pose );
    optimizer_->addVertex( v );

    curr_vertex_key_++;
}

void PoseOptimizerG2O::addNewEdge(Eigen::MatrixXd cov, uint vertex2_key, uint vertex1_key, uint *key)
{
    *key = curr_edge_key_;

	g2o::VertexSE3* vertex1 = dynamic_cast< g2o::VertexSE3* >( optimizer_->vertex( vertex1_key ) );
	g2o::VertexSE3* vertex2 = dynamic_cast< g2o::VertexSE3* >( optimizer_->vertex( vertex2_key ) );
    Eigen::Matrix4d e_T = (vertex1->estimate().inverse() * vertex2->estimate()).matrix();
    Pose6DOF e_pose(e_T);
    g2o::SE3Quat se3_pose(e_pose.rot, e_pose.pos);

    Eigen::Matrix< double, 6, 6 > meas_info = cov.inverse();
	g2o::EdgeSE3* edge = new g2o::EdgeSE3();
    edge->setId(*key);
    edge->vertices()[0] = vertex1;
	edge->vertices()[1] = vertex2;
	edge->setMeasurement( se3_pose );

    // std::cout << "vertex 1 \n" << vertex1->estimate().matrix() << std::endl;
    // std::cout << "vertex 2 \n" << vertex2->estimate().matrix() << std::endl;
    // std::cout << "edge relative transform \n" << e_T << std::endl;
    // std::cout << "information matrix \n" << meas_info << std::endl << std::endl;
	edge->setInformation( meas_info );

    bool success = optimizer_->addEdge( edge );
    
    if(!success)
    {
        ROS_ERROR("Error adding edge between vertices %d and %d", vertex1_key, vertex2_key);
        return;
    }

    std::pair<uint, uint> edge_keys(vertex1_key, vertex2_key);
    graph_edges_.insert(std::pair<uint, std::pair<uint, uint>>(*key, edge_keys));

    curr_edge_key_++;
}

bool PoseOptimizerG2O::optimizeGraph()
{
    optimizer_->initializeOptimization();
    bool valid_info_matrices = optimizer_->verifyInformationMatrices(true);
    if(valid_info_matrices)
    {
        optimizer_->save("/home/alfredoso/icpslam_posegraph_before.g2o");

        optimizer_->computeInitialGuess();
        optimizer_->computeActiveErrors();
        optimizer_->setVerbose(true);
        std::cout << "Initial chi2 = " << FIXED(optimizer_->chi2()) << std::endl;
        ROS_ERROR("Optimization iterating");
        int iters = optimizer_->optimize(pose_opt_iters);

        // std::cout << "Results: " << optimizer_->vertices().size() << " nodes, " << optimizer_->edges().size() << " edges, " << "chi2: " << optimizer_->chi2() << "\n";
        
        if (iters < pose_opt_iters)
        {
            ROS_ERROR("Pose graph optimization failed after %i iterations.", iters);
            return false;
        }

        ROS_ERROR("Pose graph optimization finished!");

        optimizer_->save("/home/alfredoso/icpslam_posegraph_after.g2o");

        return true;
    }

    return false;
}

void PoseOptimizerG2O::refinePoseGraph()
{
    refineVertices();
    refineEdges();
}

void PoseOptimizerG2O::refineVertices()
{
    ROS_INFO("Refining vertices");
    for(size_t v_key=0; v_key<optimizer_->vertices().size() ;v_key++)
    {
        g2o::VertexSE3* v = dynamic_cast< g2o::VertexSE3* >( optimizer_->vertex( v_key ) );
        Eigen::Matrix4d v_T = v->estimate().matrix();
        Pose6DOF v_pose(v_T);
        v_pose.pos = v_pose.pos;
        graph_poses_.find(v_key)->second = v_pose;

        if(v_key == optimizer_->vertices().size()-1)
            latest_pose = v_pose;
    }
}

void PoseOptimizerG2O::refineEdges()
{
    ROS_INFO("Refining edges");
    uint edge_key = 0;
    for( g2o::HyperGraph::EdgeSet::iterator edge_ptr = optimizer_->edges().begin(); edge_ptr != optimizer_->edges().end(); edge_ptr++)
    {
        g2o::EdgeSE3* edge = dynamic_cast< g2o::EdgeSE3* >( *edge_ptr );
        uint vertex1_key = edge->vertices()[0]->id();
        uint vertex2_key = edge->vertices()[1]->id();
        g2o::VertexSE3* vertex1 = dynamic_cast< g2o::VertexSE3* >(optimizer_->vertex( vertex1_key ));
	    g2o::VertexSE3* vertex2 = dynamic_cast< g2o::VertexSE3* >(optimizer_->vertex( vertex2_key ));
        Eigen::Matrix4d e_T = (vertex1->estimate().inverse() * vertex2->estimate()).matrix();
        Pose6DOF e_pose(e_T);
        g2o::SE3Quat se3_pose(e_pose.rot, e_pose.pos);
        edge->setMeasurement(se3_pose);

        edge_key++;
    }
}

bool PoseOptimizerG2O::checkLoopClosure()
{
    return false;
}

