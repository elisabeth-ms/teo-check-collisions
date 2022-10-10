# Copyright: Universidad Carlos III de Madrid (C) 2022
# Authors: Elisabeth Menendez
# CopyPolicy: Released under the terms of the LGPLv2.1 or later

cmake_minimum_required(VERSION 3.12 FATAL_ERROR)

# Start a project. Set the LANGUAGES option according to your
# needs, e.g. 'C', 'CXX', 'NONE'.
project(TEO_CHECK_COLLISIONS VERSION 0.1.0 LANGUAGES CXX)

# Set module path.
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake
                              ${CMAKE_SOURCE_DIR}/cmake/modules
                              ${CMAKE_SOURCE_DIR}/cmake/find-modules)

# Hard dependencies.
find_package(YCM 0.11 REQUIRED)
find_package(YARP 3.4 REQUIRED COMPONENTS os dev)
find_package(ROBOTICSLAB_KINEMATICS_DYNAMICS REQUIRED)
find_package(fcl 0.6.0 REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)

# Soft dependencies.
find_package(Doxygen QUIET)
find_package(GTestSources 1.6.0 QUIET)

# Add WITH_PLUGINS if this projects exports YARP devices.
yarp_configure_external_installation(teo-check-collisions)

# Retrieve current CHECK_TEO_COLLISIONS version.
include(TEO_CHECK_COLLISIONSVersion)

# Define standard paths, check system configuration.
include(TEO_CHECK_COLLISIONSSystem)

# Setup build options.
include(TEO_CHECK_COLLISIONSOptions)

# Define and enter subdirectories.
add_subdirectory(cmake)
add_subdirectory(libraries)
add_subdirectory(programs)
# add_subdirectory(tests)
add_subdirectory(share)
add_subdirectory(doc)

# Output configuration files.
include(TEO_CHECK_COLLISIONSDescribe)

# Invoke CPack from here.
include(TEO_CHECK_COLLISIONSPackage)


# Create and install config files.
include(InstallBasicPackageFiles)

install_basic_package_files(TEO_CHECK_COLLISIONS
                            VERSION 0.1.0
                            COMPATIBILITY AnyNewerVersion
                            NO_SET_AND_CHECK_MACRO
                            NO_CHECK_REQUIRED_COMPONENTS_MACRO
                            NAMESPACE ROBOTICSLAB::
                            DEPENDENCIES ${_exported_dependencies})
# Generate a report of enabled/disabled features.
feature_summary(WHAT ALL)
