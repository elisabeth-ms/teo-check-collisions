# Copyright: Universidad Carlos III de Madrid (C) 2022
# Authors: Elisabeth Menendez
# CopyPolicy: Released under the terms of the LGPLv2.1 or later

if(ENABLE_TeoCheckCollisionsLibrary)

set(KEYWORD "TeoCheckCollisionsLibrary")

# Configure library build target.
add_library(${KEYWORD} SHARED TeoCheckCollisionsLibrary.cpp
                              TeoCheckCollisionsLibrary.hpp)

# Set project properties.
set_target_properties(${KEYWORD} PROPERTIES PUBLIC_HEADER "TeoCheckCollisionsLibrary.hpp")

# Link external libraries.
target_link_libraries(${KEYWORD} PUBLIC YARP::YARP_OS
                                        ROBOTICSLAB::KdlVectorConverterLib
                                        ROBOTICSLAB::KinematicRepresentationLib
                                        fcl
                                        TeoCheckSelfCollisionsLibrary
                                 )

# Define include paths.
target_include_directories(${KEYWORD} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                                             $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
                                             ${FCL_INCLUDE_DIRS} ${teo_check_self_collisions_library}
                                             )

# Create installation rule.
install(TARGETS ${KEYWORD}
        EXPORT TEO_CHECK_COLLISIONS
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT teo_check_self_collisions_library
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT teo_check_self_collisions_library
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT teo_check_self_collisions_library
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} COMPONENT teo_check_self_collisions_library)

# Export current target.
set_property(GLOBAL APPEND PROPERTY TEO_CHECK_COLLISIONS_TARGETS ${KEYWORD})

endif()
