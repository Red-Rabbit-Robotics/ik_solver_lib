#ifndef TRAC_IK_SOLVER_H
#define TRAC_IK_SOLVER_H

#include <ros/ros.h>
#include <trac_ik/trac_ik.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <ik_solver_lib/base/ik_solver_base.h>

namespace ik_solver_plugin
{
    class TracIKSolver : public IKSolverBase
    {
    public:
        TracIKSolver();
        void initialize(const std::string& chain_start, const std::string& chain_end, const std::string& urdf_param, double timeout, double eps) override;
        bool solveIK(const KDL::Frame& desired_pose, KDL::JntArray& result_joint_positions) override;

    private:
        std::shared_ptr<TRAC_IK::TRAC_IK> ik_solver_ptr_;
        KDL::Chain chain_;
        KDL::JntArray ll_, ul_;  // Lower and upper joint limits
    };
}

#endif // TRAC_IK_SOLVER_H

