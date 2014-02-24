# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(data_fusion)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.

set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS 
          roscpp
          tf 
          actionlib 
          state_manager
          dynamic_reconfigure
          pandora_navigation
          roslint
          data_fusion_communications 
          controllers_and_sensors_communications 
          vision_communications 
          )

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(
  config/alert_handler/alert_handler_dynamic.cfg
  config/alert_handler/mass_alert_publisher_dymamic.cfg
  )


catkin_package(
    DEPENDS 
    CATKIN-DEPENDS 
    INCLUDE_DIRS 
    LIBRARIES 
)

include_directories(include ${catkin_INCLUDE_DIRS})

#---------------Victim fusion node--------------------
add_executable(victim_fusion src/victim_fusion/main.cpp
                             src/victim_fusion/co2.cpp 
                             src/victim_fusion/camera.cpp 
                             src/victim_fusion/mlx.cpp 
                             src/victim_fusion/tpa.cpp

                             src/victim_fusion/state_abstract.cpp 
                             src/victim_fusion/state_arm_approach.cpp 
                             src/victim_fusion/state_df_hold.cpp
                             src/victim_fusion/state_exploration_identification.cpp 
                             src/victim_fusion/state_any_other.cpp

                             src/victim_fusion/victim_fusion.cpp 
          )
target_link_libraries(victim_fusion ${catkin_LIBRARIES} )
add_dependencies(victim_fusion 
  controllers_and_sensors_communications
  vision_communication_gencpp
  data_fusion_communication_gencpp
  ) 



###################### objects #########################################
add_library(objects 
        src/alert_handler/objects.cpp)
target_link_libraries(objects 
          ${catkin_LIBRARIES}
          )
          
add_library(victim 
        src/alert_handler/victim.cpp)
target_link_libraries(victim 
          ${catkin_LIBRARIES}
          )

###################### victim_list #########################################
add_library(victim_list 
        src/alert_handler/victim_list.cpp)
target_link_libraries(victim_list 
          ${catkin_LIBRARIES}
          )
          
          
##################### Utils ############################################
add_library(utils 
        src/alert_handler/utils.cpp)
target_link_libraries(utils 
          ${catkin_LIBRARIES}
          )
  
#################### PoseFinder ########################################
add_library(pose_finder 
        src/alert_handler/pose_finder.cpp)
target_link_libraries(pose_finder 
          ${catkin_LIBRARIES}
          )
          
#################### VicitmHandler ########################################
add_library(victim_clusterer 
        src/alert_handler/victim_clusterer.cpp)
target_link_libraries(victim_clusterer 
          ${catkin_LIBRARIES}
          )
          
#################### VicitmHandler ########################################
add_library(victim_handler 
        src/alert_handler/victim_handler.cpp)
target_link_libraries(victim_handler 
          ${catkin_LIBRARIES}
          )
          
##################### object_handler ############################################
add_library(object_handler 
        src/alert_handler/object_handler.cpp)
target_link_libraries(object_handler 
          ${catkin_LIBRARIES}
          )
add_dependencies(object_handler 
  data_fusion_gencfg 
  data_fusion_communication_gencpp
  ) 

##################### alert_handler ############################################
add_library(alert_handler 
        src/alert_handler/alert_handler.cpp)
target_link_libraries(alert_handler 
          ${catkin_LIBRARIES}
          )
add_dependencies(alert_handler 
  data_fusion_gencfg 
  vision_communication_gencpp
  data_fusion_communication_gencpp
  ) 

##################### alert_handler_node ############################################
add_executable(alert_handler_node 
        src/alert_handler/main.cpp )
target_link_libraries(alert_handler_node 
          ${catkin_LIBRARIES}
          alert_handler
          object_handler
          pose_finder
          victim_handler
          victim_clusterer
          victim_list
          utils
          objects
          victim
          )


include(${CMAKE_CURRENT_SOURCE_DIR}/tests.cmake)

set(ROSLINT_CPP_OPTS 
    "--filter=-whitespace/end_of_line,-build/include_order,-whitespace/blank_line,-whitespace/parens,-whitespace/comments")
FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS 
     RELATIVE ${PROJECT_SOURCE_DIR} 
            include/alert_handler/*.h 
            src/alert_handler/*.cpp 
            )
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})
