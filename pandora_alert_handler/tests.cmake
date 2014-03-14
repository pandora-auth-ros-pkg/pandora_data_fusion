################# Tests ################################################
# add tests here so that CMakelists is not polluted

##########  ObjectListTest ###########  
#~ 
#~ 
catkin_add_gtest(object_list_test test/unit/object_list_test.cpp)
target_link_libraries(object_list_test ${catkin_LIBRARIES}  objects utils
  gtest_main) 


##########  ObjectFactoryTest ########### 

#~ catkin_add_gtest(object_factory_test test/unit/object_factory_test.cpp)
#~ target_link_libraries(object_factory_test ${catkin_LIBRARIES}  objects utils ros_tf_listener
  #~ gtest_main) 
 
  #~ 
##########  PoseFinderTest ###########   
  
#~ catkin_add_gtest(pose_finder_test test/unit/pose_finder_test.cpp)
#~ target_link_libraries(pose_finder_test ${catkin_LIBRARIES}
#~ pose_finder   utils   gtest_main) 



##########  ObjectsTest ###########       
#~ catkin_add_gtest(objects_test test/unit/objects_test.cpp)
#~ target_link_libraries(objects_test ${catkin_LIBRARIES} objects utils  tpa qr hazmat ) 
#~ 
#~ catkin_add_gtest(qr_test test/unit/qr_test.cpp)
#~ target_link_libraries(qr_test ${catkin_LIBRARIES} objects utils  qr gtest_main) 
#~ 
#~ catkin_add_gtest(hazmat_test test/unit/hazmat_test.cpp)
#~ target_link_libraries(hazmat_test ${catkin_LIBRARIES} objects utils  hazmat gtest_main)



set(ROSLINT_CPP_OPTS 
    "--filter=-whitespace/end_of_line,-build/include_order,-build/include,-whitespace/blank_line,-whitespace/parens,-whitespace/comments")
FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS 
     RELATIVE ${PROJECT_SOURCE_DIR} 
            #~ include/alert_handler/*.h 
            #~ src/*.cpp 
            test/unit/object_list_test.cpp
            )
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})
