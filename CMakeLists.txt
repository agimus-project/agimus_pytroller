cmake_minimum_required(VERSION 3.22.1)

#
# Project definition
#
project(agimus_pytroller LANGUAGES CXX)

#
# Options
#
option(BUILD_SHARED_LIBS "Build using shared libraries" ON)

cmake_policy(SET CMP0094 NEW)
set(Python3_FIND_UNVERSIONED_NAMES FIRST)

# Find python before pybind11
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

#
# Handle dependencies by reading the package.xml
#
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)

#
# Main Library
#

generate_parameter_library(
  agimus_pytroller_parameters
  src/agimus_pytroller_parameters.yaml
)

set(${PROJECT_NAME}_headers
    include/${PROJECT_NAME}/visibility.hpp
    include/${PROJECT_NAME}/agimus_pytroller.hpp)

set(${PROJECT_NAME}_sources
    src/agimus_pytroller.cpp)

ament_auto_add_library(${PROJECT_NAME} ${${PROJECT_NAME}_sources}
    ${${PROJECT_NAME}_headers})

set_target_properties(${PROJECT_NAME} PROPERTIES CXX_VISIBILITY_PRESET hidden)

target_compile_definitions(
  ${PROJECT_NAME}
  PUBLIC -DAGIMUS_PYTROLLER_IS_SHARED
  PRIVATE -DAGIMUS_PYTROLLER_DO_EXPORT)

target_link_libraries(${PROJECT_NAME} pybind11::embed agimus_pytroller_parameters)

#
# Export plugins
#
pluginlib_export_plugin_description_file(controller_interface
                                         controller_plugins.xml)


# Install Python modules
ament_python_install_package(${PROJECT_NAME}_py)

ament_export_libraries(${PROJECT_NAME})
ament_auto_package()
