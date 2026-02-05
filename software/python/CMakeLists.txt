pybind11_add_module(OpenEAIArm_py OpenEAIArm/OpenEAIArm_py.cpp)
set_target_properties(OpenEAIArm_py PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/python/OpenEAIArm)
target_link_libraries(OpenEAIArm_py PRIVATE OpenEAIArm_core dm_core)
add_custom_target(pip_install
    COMMAND ${Python3_EXECUTABLE} -m pip install --upgrade --force-reinstall ${CMAKE_SOURCE_DIR}/python
    COMMENT "Installing Python package via pip"
    DEPENDS OpenEAIArm_py
)