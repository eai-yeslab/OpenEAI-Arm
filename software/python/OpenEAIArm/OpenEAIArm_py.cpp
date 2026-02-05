#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "OpenEAIArm.hpp"

namespace py = pybind11;

PYBIND11_MODULE(OpenEAIArm_py, m) {
    py::class_<OpenEAIArm>(m, "OpenEAIArm")
        .def(py::init<const std::string &>()) // 构造函数，假设接收串口字符串
        .def("joint_step", &OpenEAIArm::joint_step)
        .def("go_home", &OpenEAIArm::go_home,
            py::arg("move_time") = 2.0f)
        .def("set_joint_targets", &OpenEAIArm::set_joint_targets,
               py::arg("target"), py::arg("move_time") = 0.0f,
               py::arg("vel") = OpenEAIArm::JointArray{}, py::arg("tau") = OpenEAIArm::JointArray{}, py::arg("interpolate") = true)
        .def("get_joint_positions", &OpenEAIArm::get_joint_positions)
        .def("get_joint_velocities", &OpenEAIArm::get_joint_velocities)
        .def("get_joint_torques", &OpenEAIArm::get_joint_torques)
        .def("enable_all", &OpenEAIArm::enable_all)
        .def("disable_all", &OpenEAIArm::disable_all)
        ;
}