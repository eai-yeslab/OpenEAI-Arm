#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "OpenEAIArm.hpp"

namespace py = pybind11;

PYBIND11_MODULE(OpenEAIArm_py, m) {
    py::enum_<OpenEAIArm::ControlMode>(m, "ControlMode")
        .value("MIT_MIX", OpenEAIArm::ControlMode::MIT_MIX)
        .value("MIT_DRAG", OpenEAIArm::ControlMode::MIT_DRAG)
        .value("SIM", OpenEAIArm::ControlMode::SIM);

    py::enum_<OpenEAIArm::IKJumpPolicy>(m, "IKJumpPolicy")
        .value("REJECT", OpenEAIArm::IKJumpPolicy::REJECT)
        .value("CLIP", OpenEAIArm::IKJumpPolicy::CLIP)
        .value("ACCEPT", OpenEAIArm::IKJumpPolicy::ACCEPT);

    py::class_<OpenEAIArm::IKOptions>(m, "IKOptions")
        .def(py::init<>())
        .def_readwrite("angle_jump_threshold", &OpenEAIArm::IKOptions::angle_jump_threshold)
        .def_readwrite("fixed_step", &OpenEAIArm::IKOptions::fixed_step)
        .def_readwrite("policy", &OpenEAIArm::IKOptions::policy)
        .def_readwrite("report_failure", &OpenEAIArm::IKOptions::report_failure);

    py::class_<OpenEAIArm>(m, "OpenEAIArm")
        .def(py::init<const std::string &, OpenEAIArm::ControlMode, const std::string &>(),
             py::arg("config_path"),
             py::arg("control_mode") = OpenEAIArm::ControlMode::MIT_MIX,
             py::arg("arm_name") = "")
        .def("joint_step", &OpenEAIArm::joint_step,
             py::arg("joint_angles"),
             py::arg("joint_velocities") = OpenEAIArm::JointArray{},
             py::arg("joint_torques") = OpenEAIArm::JointArray{})
        .def("go_home", &OpenEAIArm::go_home, py::arg("move_time") = 2.0f)
        .def("reset", &OpenEAIArm::reset,
             py::arg("move_time") = 2.0f,
             py::arg("angle_jump_threshold") = 0.28f)
        .def("get_joint_positions", &OpenEAIArm::get_joint_positions)
        .def("get_joint_velocities", &OpenEAIArm::get_joint_velocities)
        .def("get_joint_torques", &OpenEAIArm::get_joint_torques)
        .def("get_ee_pose", &OpenEAIArm::get_ee_pose)
        .def("get_joint_names", &OpenEAIArm::get_joint_names)
        .def("forward_kinetics", &OpenEAIArm::forward_kinetics)
        .def("inverse_kinetics",
             [](const OpenEAIArm &arm,
                const OpenEAIArm::JointArray &ee_pose,
                OpenEAIArm::IKOptions options) {
                 bool success = false;
                 auto joints = arm.inverse_kinetics(ee_pose, success, options);
                 return py::make_tuple(joints, success);
             },
             py::arg("ee_pose"),
             py::arg("options") = OpenEAIArm::IKOptions(),
             "Return (joint_angles, success) for the requested end-effector pose.")
        .def("inverse_kinetics_seeded",
             [](const OpenEAIArm &arm,
                const OpenEAIArm::JointArray &ee_pose,
                const OpenEAIArm::JointArray &seed_joints,
                OpenEAIArm::IKOptions options) {
                 bool success = false;
                 auto joints = arm.inverse_kinetics(ee_pose, seed_joints, success, options);
                 return py::make_tuple(joints, success);
             },
             py::arg("ee_pose"),
             py::arg("seed_joints"),
             py::arg("options") = OpenEAIArm::IKOptions(),
             "Return (joint_angles, success) using seed_joints as the IK branch seed.")
        .def("set_joint_targets",
             py::overload_cast<const OpenEAIArm::JointArray &, float,
                               const OpenEAIArm::JointArray &,
                               const OpenEAIArm::JointArray &, bool>
                               (&OpenEAIArm::set_joint_targets),
             py::arg("target"),
             py::arg("move_time") = 0.0f,
             py::arg("vel") = OpenEAIArm::JointArray{},
             py::arg("tau") = OpenEAIArm::JointArray{},
             py::arg("interpolate") = true)
        .def("set_control_mode", &OpenEAIArm::set_control_mode)
        .def("gripper_width_to_joint", &OpenEAIArm::gripper_width_to_joint)
        .def("joint_to_gripper_width", &OpenEAIArm::joint_to_gripper_width)
        .def("q_control_to_phys", &OpenEAIArm::q_control_to_phys)
        .def("q_phys_to_control", &OpenEAIArm::q_phys_to_control)
        .def("send_drag_command", &OpenEAIArm::send_drag_command)
        .def("enable_all", &OpenEAIArm::enable_all)
        .def("disable_all", &OpenEAIArm::disable_all);
}
