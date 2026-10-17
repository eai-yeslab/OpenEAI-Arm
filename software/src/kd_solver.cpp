// ----- KDSolver.cpp -----
#include "kd_solver.hpp"

// Constructor
KDSolver::KDSolver(const std::string& urdf_file, const std::string& base_link, const std::string& tip_link)
{
    // 1. Read and parse URDF file
    urdf::Model urdf_model;
    if (!urdf_model.initFile(urdf_file)) {
        throw std::runtime_error("Failed to parse URDF file.");
    }

    // 2. Use kdl_parser to convert URDF to KDL Tree
    if (!kdl_parser::treeFromUrdfModel(urdf_model, tree_)) {
        throw std::runtime_error("Failed to convert urdf::Model to KDL::Tree.");
    }

    // 3. Extract the required chain segment
    if (!tree_.getChain(base_link, tip_link, chain_)) {
        throw std::runtime_error("Failed to extract KDL::Chain.");
    }
    num_joints_ = chain_.getNrOfJoints();

    // 4. Extract joint names (in the same order as KDL Chain)
    joint_names_.clear();
    for (unsigned int i=0; i < chain_.getNrOfSegments(); ++i) {
        KDL::Segment seg = chain_.getSegment(i);
        // Only take non-fixed joints
        if (seg.getJoint().getType() != KDL::Joint::None) {
            joint_names_.push_back(seg.getJoint().getName());
        }
    }
    num_joints_ = joint_names_.size();

    // 5. Use URDF Model to get joint limits (in the order of joint_names)
    KDL::JntArray joint_min(num_joints_), joint_max(num_joints_);
    for (size_t i=0; i<num_joints_; ++i) {
        std::string& jname = joint_names_[i];
        urdf::JointConstSharedPtr joint = urdf_model.getJoint(jname);
        if (joint && joint->limits) {
            joint_min(i) = joint->limits->lower;
            joint_max(i) = joint->limits->upper;
        } else {
            // Non-rotational or undefined joints, set limits to a wide range (e.g., continuous joints)
            joint_min(i) = -1e10;
            joint_max(i) = 1e10;
        }
    }

    // 6. Construct FK/IK solvers (with forward and velocity inverse solvers)
    float eps = 0.001;
    fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
    ik_vel_solver_ = std::make_unique<KDL::ChainIkSolverVel_pinv>(chain_, eps);
    ik_solver_ = std::make_unique<KDL::ChainIkSolverPos_NR_JL>(chain_, joint_min, joint_max,
                                                                *fk_solver_, *ik_vel_solver_, 100, eps);

    // Gravity direction downwards (negative z direction)
    KDL::Vector gravity_vec(0, 0, -9.81);
    dyn_param_solver_ = std::make_unique<KDL::ChainDynParam>(chain_, gravity_vec);
}

bool KDSolver::forwardKinematics(const std::vector<double>& joint_positions, KDL::Frame& out_pose)
{
    if (joint_positions.size() != num_joints_) return false;
    KDL::JntArray q(num_joints_);
    for (size_t i = 0; i < num_joints_; ++i) q(i) = joint_positions[i];
    return fk_solver_->JntToCart(q, out_pose) >= 0;
}

bool KDSolver::inverseKinematics(const KDL::Frame& desired_pose, std::vector<double>& out_joint_positions, std::vector<double>& init_joint_positions)
{
    KDL::JntArray q_init(num_joints_), q_sol(num_joints_);
    for (size_t i = 0; i < num_joints_; ++i) q_init(i) = init_joint_positions[i];

    int ret = ik_solver_->CartToJnt(q_init, desired_pose, q_sol);
    if (ret < 0) return false;

    out_joint_positions.resize(num_joints_);
    for (size_t i = 0; i < num_joints_; ++i) out_joint_positions[i] = q_sol(i);
    return true;
}

bool KDSolver::gravityCompensation(const std::vector<double>& joint_positions, std::vector<double>& out_torques)
{
    if (joint_positions.size() != num_joints_) return false;
    KDL::JntArray q(num_joints_), torques(num_joints_);
    for (size_t i = 0; i < num_joints_; ++i) q(i) = joint_positions[i];

    int ret = dyn_param_solver_->JntToGravity(q, torques);
    if (ret < 0) return false;

    out_torques.resize(num_joints_);
    for (size_t i = 0; i < num_joints_; ++i) out_torques[i] = torques(i);
    return true;
}

bool KDSolver::dynamicsFeedforward(
    const std::vector<double>& joint_positions,
    const std::vector<double>& joint_velocities,
    const std::vector<double>& joint_accelerations,
    const std::vector<double>& coriolis_scales,
    const std::vector<double>& inertia_scales,
    std::vector<double>& out_torques)
{
    if (joint_positions.size() != num_joints_ ||
        joint_velocities.size() != num_joints_ ||
        joint_accelerations.size() != num_joints_ ||
        coriolis_scales.size() != num_joints_ ||
        inertia_scales.size() != num_joints_) {
        return false;
    }

    KDL::JntArray q(num_joints_);
    KDL::JntArray qd(num_joints_);
    KDL::JntArray qdd(num_joints_);
    KDL::JntArray coriolis(num_joints_);
    KDL::JntArray gravity(num_joints_);
    KDL::JntSpaceInertiaMatrix mass(num_joints_);

    for (size_t i = 0; i < num_joints_; ++i) {
        q(i) = joint_positions[i];
        qd(i) = joint_velocities[i];
        qdd(i) = joint_accelerations[i];
    }

    if (dyn_param_solver_->JntToMass(q, mass) < 0 ||
        dyn_param_solver_->JntToCoriolis(q, qd, coriolis) < 0 ||
        dyn_param_solver_->JntToGravity(q, gravity) < 0) {
        return false;
    }

    out_torques.assign(num_joints_, 0.0);
    for (size_t i = 0; i < num_joints_; ++i) {
        double inertia_torque = 0.0;
        for (size_t j = 0; j < num_joints_; ++j) {
            inertia_torque += mass(i, j) * qdd(j);
        }
        out_torques[i] = gravity(i)
            + coriolis_scales[i] * coriolis(i)
            + inertia_scales[i] * inertia_torque;
    }
    return true;
}
