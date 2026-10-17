// ----- KDSolver.hpp -----
#pragma once

#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h> // For parsing URDF file
#include <memory>
#include <iostream>

class KDSolver {
public:
    KDSolver(const std::string& urdf_file, 
             const std::string& base_link, 
             const std::string& tip_link);

    // Forward Kinematics
    bool forwardKinematics(const std::vector<double>& joint_positions, KDL::Frame& out_pose);

    // Inverse Kinematics
    bool inverseKinematics(const KDL::Frame& desired_pose, std::vector<double>& out_joint_positions, std::vector<double>& init_joint_positions);

    // Gravity Compensation
    bool gravityCompensation(const std::vector<double>& joint_positions, std::vector<double>& out_torques);

    // Legacy public API: gravity plus Coriolis, without inertia feedforward.
    bool gravityCompensation(const std::vector<double>& positions,
                             const std::vector<double>& velocities,
                             std::vector<double>& torques) {
        return dynamicsFeedforward(positions, velocities,
            std::vector<double>(positions.size(), 0.0),
            std::vector<double>(positions.size(), 1.0),
            std::vector<double>(positions.size(), 0.0), torques);
    }

    // Full rigid-body dynamics feedforward:
    // tau = M(q) * qdd + C(q, qd) + G(q).
    bool dynamicsFeedforward(const std::vector<double>& joint_positions,
                             const std::vector<double>& joint_velocities,
                             const std::vector<double>& joint_accelerations,
                             const std::vector<double>& coriolis_scales,
                             const std::vector<double>& inertia_scales,
                             std::vector<double>& out_torques);

private:
    KDL::Tree tree_;
    KDL::Chain chain_;

    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    std::unique_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_; 
    std::unique_ptr<KDL::ChainIkSolverPos_NR_JL> ik_solver_;
    std::unique_ptr<KDL::ChainDynParam>          dyn_param_solver_;

    size_t num_joints_;
    std::vector<std::string> joint_names_;
};
