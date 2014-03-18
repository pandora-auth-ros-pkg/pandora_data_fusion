################# MapLoader ############################################

find_package(catkin REQUIRED COMPONENTS map_server)

find_package(PkgConfig)
pkg_check_modules(NEW_YAMLCPP yaml-cpp>=0.5)
if(NEW_YAMLCPP_FOUND)
  add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NEW_YAMLCPP_FOUND)

link_directories(${catkin_LIBRARY_DIRS})
include_directories( test/map_loader/include ${catkin_INCLUDE_DIRS})

add_library(map_loader test/map_loader/src/map_loader.cpp)
target_link_libraries(map_loader 
    image_loader
    yaml-cpp
    ${catkin_LIBRARIES}
)

################# Tests ################################################
# add tests here so that CMakelists is not polluted

##########  ObjectListTest ###########  


catkin_add_gtest(object_list_test test/unit/object_list_test.cpp)
target_link_libraries(object_list_test ${catkin_LIBRARIES}  objects utils
  gtest_main) 


##########  ObjectFactoryTest ########### 

catkin_add_gtest(object_factory_test test/unit/object_factory_test.cpp)
target_link_libraries(object_factory_test 

${catkin_LIBRARIES} 
object_factory
objects utils
ros_tf_listener
map_loader
gtest_main) 

##########  PoseFinderTest ###########   

set(CMAKE_BUILD_TYPE Debug)
catkin_add_gtest(pose_finder_test test/unit/pose_finder_test.cpp)
target_link_libraries(pose_finder_test ${catkin_LIBRARIES}
    pose_finder map_loader utils gtest_main)

######### VictimTest #################
catkin_add_gtest(victim_test test/unit/victim_test.cpp)
target_link_libraries(victim_test ${catkin_LIBRARIES}
   victim gtest_main) 


##########  ObjectsTest ###########       
catkin_add_gtest(objects_test test/unit/objects_test.cpp)
target_link_libraries(objects_test ${catkin_LIBRARIES} objects utils  tpa qr hazmat ) 

catkin_add_gtest(qr_test test/unit/qr_test.cpp)
target_link_libraries(qr_test ${catkin_LIBRARIES} objects utils  qr gtest_main) 

catkin_add_gtest(hazmat_test test/unit/hazmat_test.cpp)
target_link_libraries(hazmat_test ${catkin_LIBRARIES} objects utils  hazmat gtest_main)


########### RosLint ###################################################
set(ROSLINT_CPP_OPTS 
    "--filter=-whitespace/end_of_line,-build/include_order,-build/include,-whitespace/blank_line,-whitespace/parens,-whitespace/comments")
FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS 
     RELATIVE ${PROJECT_SOURCE_DIR} 
            #~ include/alert_handler/*.h 
            #~ src/*.cpp 
            test/unit/object_list_test.cpp
            )
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})
