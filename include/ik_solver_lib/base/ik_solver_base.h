#ifndef IK_SOLVER_BASE_H
#define IK_SOLVER_BASE_H

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

namespace ik_solver_plugin
{
    class IKSolverBase
    {
    public:
        virtual ~IKSolverBase() {}
        virtual void initialize(const std::string& chain_start, const std::string& chain_end, const std::string& urdf_param, double timeout, double eps) = 0; 
        virtual bool solveIK(const KDL::Frame& desired_pose, KDL::JntArray& result_joint_positions) = 0;
    };
}

#endif // IK_SOLVER_BASE_H

