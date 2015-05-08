# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorials)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

## Declare ROS messages and services
add_message_files(
	FILES
	keyframeMsg.msg 
	keyframeGraphMsg.msg)
add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

SET (INCLUDE_DIRS "../eigen3.1/")
##set(CMAKE_MODULE_PATH /home/turtlebot-3/catkin_ws/src/beginner_tutorials/cmake_modules)
set(CMAKE_MODULE_PATH /home/tyshao/catkin_ws/src/beginner_tutorials/cmake_modules)
find_package( Eigen3 REQUIRED )
INCLUDE_DIRECTORIES( ${EIGEN3_INCLUDE_DIR} )
SET( Sophus_INCLUDE_DIR ${Sophus_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR} )

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

##add_executable(talker src/talker.cpp)
##target_link_libraries(talker ${catkin_LIBRARIES})
##add_dependencies(talker beginner_tutorials_generate_messages_cpp)

##add_executable(listener src/listener.cpp)
##target_link_libraries(listener ${catkin_LIBRARIES})
##add_dependencies(listener beginner_tutorials_generate_messages_cpp)

##add_executable(pose src/pose.cpp)
##target_link_libraries(pose ${catkin_LIBRARIES})
##add_dependencies(pose beginner_tutorials_generate_messages_cpp)

add_executable(append src/append.cpp)
target_link_libraries(append ${catkin_LIBRARIES})
add_dependencies(append beginner_tutorials_generate_messages_cpp)

# add_dependencies(talker beginner_tutorials_generate_messages_cpp)
# %EndTag(FULLTEXT)%
