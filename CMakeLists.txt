cmake_minimum_required(VERSION 2.4.6)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
include_directories(/opt/ros/fuerte/stacks/arm_navigation/arm_navigation_msgs/msg_gen/cpp/include/)
include_directories(/opt/ros/fuerte/stacks/arm_navigation/arm_navigation_msgs/include/)
include_directories(/opt/ros/fuerte/stacks/geometry/tf/include/)
include_directories(/opt/ros/fuerte/stacks/bullet/include/)
include_directories(/opt/ros/fuerte/stacks/arm_navigation/arm_navigation_msgs/srv_gen/cpp/include/)


rosbuild_find_ros_package(actionlib_msgs)
include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
genaction()

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
rosbuild_add_library(NavigationTree src/NavigationTree.cpp)
rosbuild_add_library(GPSController src/GPSController.cpp)
rosbuild_add_library(NCSDirectionInterface src/NCSDirectionInterface.cpp)
rosbuild_add_library(KeyboardDirectionInterface src/KeyboardDirectionInterface.cpp)
rosbuild_add_library(ArmNavigation src/ArmNavigation.cpp)
#rosbuild_add_boost_directories()
rosbuild_link_boost(ArmNavigation thread)
rosbuild_add_executable(NCSNavigation src/NCSNavigation.cpp)
rosbuild_add_executable(KeyboardNavigation src/KeyboardNavigation.cpp)
rosbuild_add_executable(CollisionMap src/CollisionMap.cpp)
target_link_libraries(GPSController NavigationTree)
target_link_libraries(GPSController NCSDirectionInterface)
target_link_libraries(GPSController KeyboardDirectionInterface)
target_link_libraries(GPSController ArmNavigation)
target_link_libraries(NCSNavigation GPSController)
target_link_libraries(KeyboardNavigation GPSController)
