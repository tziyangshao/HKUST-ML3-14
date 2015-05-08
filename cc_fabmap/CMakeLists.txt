cmake_minimum_required(VERSION 2.8.3)
project(cc_fabmap)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
  pcl_conversions 
  pcl_ros
  genmsg
  image_geometry
  #message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


# catkin_python_setup()



## Generate messages in the 'msg' folder
add_message_files( FILES similarityTransformStamped.msg keyframeMsg.msg)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   sensor_msgs
   std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cc_fabmap
  CATKIN_DEPENDS cv_bridge image_transport roscpp rospy sensor_msgs std_msgs geometry_msgs pcl_conversions pcl_ros image_geometry
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  /home/tyshao/fyp_workspace/lib/opengv/include

)

## Declare a cpp library
# add_library(cc_fabmap
#   src/${PROJECT_NAME}/cc_fabmap.cpp
# )

## Declare a cpp executable
#add_executable(cc_fabmap_node src/cc_fabmap_node.cpp src/CentralStorage.cpp src/RobotListener_Keyframes.cpp src/HelperFcts.cpp src/PtClListener.cpp src/PoseListener.cpp src/SyncListener.cpp src/random_generators.cpp src/experiment_helpers.cpp src/time_measurement.cpp)
add_executable(cc_fabmap_node src/cc_fabmap_node.cpp src/CentralStorage.cpp src/KeyFrameListenerTraining.cpp src/HelperFcts.cpp src/SyncListener.cpp)



## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
#add_dependencies(cc_fabmap_node cc_fabmap_generate_messages_cpp CentralStorage_cpp RobotListener_Keyframes_cpp HelperFcts_cpp PtClListener_cpp PoseListener_cpp SyncListener_cpp random_generators_cpp experiment_helpers_cpp time_measurement_cpp)
add_dependencies(cc_fabmap_node cc_fabmap_generate_messages_cpp CentralStorage_cpp KeyFrameListenerTraining_cpp HelperFcts_cpp SyncListener_cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(cc_fabmap_node ${catkin_LIBRARIES} opengv)
#ExtrMatchDescrCl RobotListener_Keyframes 

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS cc_fabmap cc_fabmap_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_cc_fabmap.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
