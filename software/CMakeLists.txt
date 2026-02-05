cmake_minimum_required(VERSION 3.10)
project(OpenEAI-Arm LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(${PROJECT_SOURCE_DIR}/include)

find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(urdf REQUIRED)
find_package(yaml-cpp REQUIRED)
# find_package(TinyXML REQUIRED)

# 2. Build dm_core as SHARED (dynamic) library
add_library(dm_core SHARED
    src/dm.cpp
    src/serial_port.cpp
    src/dm_manager.cpp
)

target_include_directories(dm_core PUBLIC include)

# add_library(sim_motor SHARED
#     src/sim_motor_manager.cpp
# )
# target_include_directories(sim_motor PUBLIC include)
# target_link_libraries(sim_motor
#     PUBLIC pinocchio::pinocchio Eigen3::Eigen yaml-cpp
# )

# 3. Build OpenEAIArm SHARED library
add_library(OpenEAIArm_core SHARED
    src/OpenEAIArm.cpp
    src/motor_controller.cpp
    src/kd_solver.cpp
    src/kdl_parser/kdl_parser.cpp
    src/urdf/model.cpp
    src/config_loader.cpp
    src/sim_motor_manager.cpp
)

target_include_directories(OpenEAIArm_core PUBLIC include)
# OpenEAIArm库依赖dm_core库
target_link_libraries(OpenEAIArm_core PUBLIC dm_core Eigen3::Eigen orocos-kdl tinyxml2 tinyxml 
    urdfdom_model yaml-cpp pinocchio::pinocchio)

# option(SIM "Enable simulation" ON)
# if(SIM)
# target_link_libraries(OpenEAIArm_core PUBLIC sim_motor)
# endif()

# 4. Build your test/app executables
add_executable(test_OpenEAIArm apps/test_openeai_arm.cpp)
target_link_libraries(test_OpenEAIArm PRIVATE OpenEAIArm_core) # Links to OpenEAIArm (which links to dm_core)

add_executable(test_OpenEAIArm_drag apps/test_openeai_arm_drag.cpp)
target_link_libraries(test_OpenEAIArm_drag PRIVATE OpenEAIArm_core)

add_executable(test_OpenEAIArm_ik apps/test_openeai_arm_ik.cpp)
target_link_libraries(test_OpenEAIArm_ik PRIVATE OpenEAIArm_core)

add_executable(test_OpenEAIArm_locate apps/test_openeai_arm_locate.cpp)
target_link_libraries(test_OpenEAIArm_locate PRIVATE OpenEAIArm_core)

add_executable(demo apps/demo.cpp)
target_link_libraries(demo PRIVATE OpenEAIArm_core)

add_executable(read_motors apps/read_motors.cpp)
target_link_libraries(read_motors PRIVATE dm_core)

add_executable(test_gravity_compensation apps/test_gravity_compensation.cpp)
target_link_libraries(test_gravity_compensation PRIVATE OpenEAIArm_core)

# add_executable(test_sim_mm apps/test_sim_mm.cpp)
# target_link_libraries(test_sim_mm PRIVATE sim_motor)

# Add debug options
set(CMAKE_CXX_FLAGS -g)

# Python bindings
# find_package(pybind11 REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
add_subdirectory(third_party/pybind11)
add_subdirectory(python)

# ROS2
add_custom_target(ros2
    COMMAND ${CMAKE_COMMAND} -E chdir ${CMAKE_SOURCE_DIR}/ros2 colcon build --symlink-install
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/ros2
    COMMENT "ROS2 package built with colcon"
    DEPENDS OpenEAIArm_core
)

add_custom_target(extra
    COMMENT "All packages are built, including ROS2 and python package."
    DEPENDS ros2 pip_install all
)

# (Optional) Tests
# enable_testing()
# add_executable(tests tests/test_main.cpp)
# target_link_libraries(tests PRIVATE my_lib)
# add_test(NAME all_tests COMMAND tests)