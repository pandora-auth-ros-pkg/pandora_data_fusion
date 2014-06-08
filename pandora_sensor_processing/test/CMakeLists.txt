find_package(catkin REQUIRED COMPONENTS
  rostest
  )

link_directories(${catkin_LIBRARY_DIRS})
include_directories(${catkin_INCLUDE_DIRS})

########################  Tests  ####################################

########################  Unit Tests  ###############################

########################  Utils  ####################################

catkin_add_gtest(sensor_utils_test unit/utils_test.cpp)
target_link_libraries(sensor_utils_test 
  ${catkin_LIBRARIES}
  sensor_utils
  gtest_main
  )

########################  Clusterer  ################################

catkin_add_gtest(clusterer_test unit/clusterer_test.cpp)
target_link_libraries(clusterer_test
  ${catkin_LIBRARIES}
  clusterer
  gtest_main
  )

########################  Functional Tests  #########################

add_rostest(functional/co2_processor_test.launch)
