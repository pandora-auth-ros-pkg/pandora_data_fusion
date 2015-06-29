cmake_minimum_required(VERSION 2.8.3)
project(frame_matcher)

################################################################################
#                                 Find Package                                 #
################################################################################
# - Use `find_package` command to bring dependencies
# - Use `generate_dynamic_reconfigure_options` to setup dynamic reconfigure
# - Set flags if needed
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    cmake_modules
    pluginlib
    urdf
    tf
    pose_finder
    nav_msgs
    geometry_msgs
    sensor_processor
    pandora_vision_common
    roslint
  )


################################################################################
#                             Catkin Package Setup                             #
################################################################################
# - Use `catkin_package` to declare this catkin package and how it interacts
#   with other packages in the catkin package ecosystem
# - Use `catkin_python_setup` to setup catkin python ecosystem with ./setup.py
# - Set `include_directories`
# - Set `link_directories` (if necessary, usually avoid to if concerns catkin
#   libraries)
catkin_package(
  CATKIN_DEPENDS
    roscpp
    pluginlib
    urdf
    tf
    pose_finder
    nav_msgs
    geometry_msgs
    sensor_processor
    pandora_vision_common
  INCLUDE_DIRS
    include
  LIBRARIES
    candidate_hole_postprocessor
    enhanced_image_postprocessor
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

################################################################################
#                                  Libraries                                   #
################################################################################
# - Declare software libraries that will be made in this package and their
#   target names with `add_library`
# - Set `target_link_libraries` for each library target
# - Set `add_dependencies`, if necessary, to `${catkin_EXPORTED_TARGETS}` or
#   others

add_library(candidate_hole_postprocessor
  src/plugins/candidate_hole_postprocessor.cpp
  )
target_link_libraries(candidate_hole_postprocessor
  ${catkin_LIBRARIES}
  )
add_dependencies(candidate_hole_postprocessor
  ${catkin_EXPORTED_TARGETS}
  )

add_library(enhanced_image_postprocessor
  src/plugins/enhanced_image_postprocessor.cpp
  )
target_link_libraries(enhanced_image_postprocessor
  ${catkin_LIBRARIES}
  )
add_dependencies(enhanced_image_postprocessor
  ${catkin_EXPORTED_TARGETS}
  )

################################################################################
#                                 Executables                                  #
################################################################################
# - Declare software executables that will be made in this package and their
#   target names with `add_executable`
# - Set `target_link_libraries` for each executable target

################################################################################
#                                   Roslint
################################################################################
# - Setup roslint for test code with `roslint_cpp`, `roslint_python` or
# `roslint_custom`

file(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}
    include/**/*.h
    src/**/*.cpp
    src/*.cpp
    )
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})

################################################################################
#                                   Testing                                    #
################################################################################
# - Declare test subdirectory, in order not to pollute source code's
#   CMakeLists.txt with it

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()

################################################################################
#                               Install Package                                #
################################################################################
# - Install directory of header files with `install(DIRECTORY ...)`
# - Install targets of exported libraries with `install(TARGETS ...)`
# - Install targets of executables with `install(TARGETS ...)`

# install( TARGETS
#     costmap_2d_markers
#     costmap_2d_cloud
#     costmap_2d_node
#     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

install(TARGETS
  enhanced_image_postprocessor
  candidate_hole_postprocessor
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )

install(FILES frame_matcher_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   PATTERN ".svn" EXCLUDE
# )
