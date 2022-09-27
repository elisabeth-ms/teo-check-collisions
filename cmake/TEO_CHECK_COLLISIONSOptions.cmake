include(CMakeDependentOption)

# options: cpp libraries
option(ENABLE_TeoCheckSelfCollisionsLibrary "Choose if you want to compile TeoCheckSelfCollisionsLibrary" TRUE)

option(ENABLE_TeoCheckCollisionsLibrary "Choose if you want to compile TeoCheckCollisionsLibrary" TRUE)

# options: cpp programs
option(ENABLE_collisionsVisualization "Choose if you want to compile exampleProgram" TRUE)

# options: force default
option(ENABLE_exampleExtraOption "Enable/disable option exampleExtraOption" TRUE)

# options: unit testing
cmake_dependent_option(ENABLE_tests "Enable/disable unit tests" ON
                       GTestSources_FOUND OFF)

# options: code coverage
option(ENABLE_coverage "Choose if you want to enable coverage collection" OFF)

# Register features.
add_feature_info(ExampleLibrary ENABLE_ExampleLibrary "Fancy example library.")
add_feature_info(exampleProgram ENABLE_exampleProgram "Fancy example program.")

# Let the user specify a configuration (only single-config generators).
if(NOT CMAKE_CONFIGURATION_TYPES)
  # Possible values.
  set(_configurations Debug Release MinSizeRel RelWithDebInfo)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS ${_configurations})

  foreach(_conf ${_configurations})
    set(_conf_string "${_conf_string} ${_conf}")
  endforeach()

  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY HELPSTRING
               "Choose the type of build, options are:${_conf_string}")

  if(NOT CMAKE_BUILD_TYPE)
    # Encourage the user to specify build type.
    message(STATUS "Setting build type to 'Release' as none was specified.")
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE Release)
  endif()
endif()

# Hide variable to MSVC users since it is not needed.
if(MSVC)
  mark_as_advanced(CMAKE_BUILD_TYPE)
endif()
