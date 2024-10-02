cmake_minimum_required(VERSION 3.0.2)
project(fw_plane_sim)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14) # for px4_lib

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp  # already installed with ros-noetic-desktop
  rospy  # already installed with ros-noetic-desktop
  std_msgs  # already installed with ros-noetic-desktop
  # mavlink # for px4_sitl mavlink messages (already installed with ros-noetic-mavlink by mavros) (mavlink c/c++ header files already downloaded in include/ folder)
  mavros # for mavros_sim mavros::ftf::
  mavros_msgs
  eigen_conversions # for mavros_sim tf:: #already installed with ros-noetic-desktop
  tf2_ros # for tf
  sss_sim_env # for sss_utils
  nodelet # for sim_node.cpp nodelet
  visualization_msgs # for marker visualize
)

#uncomment the following 4 lines to use the Eigen library
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

include_directories(
  include   
  src/px4_lib # for px4_modules build
  src/fw_sim/BHDynamic # for BHDynamic.so
  ${catkin_INCLUDE_DIRS}
)

add_library(px4_lib_fw
              src/px4_lib/drivers/drv_hrt.cpp
              src/px4_lib/hysteresis/hysteresis.cpp
              src/px4_lib/geo/geo.cpp
              src/px4_lib/parameters/px4_parameters.cpp
              src/px4_lib/uORB/uORB_sim.cpp) 

catkin_package(
  #  CATKIN_DEPENDS sss_sim_env
   INCLUDE_DIRS include
)


add_library(fw_plane_visualizer
            src/fw_plane_visualizer.cpp)

# link libraries for this lib
target_link_libraries(fw_plane_visualizer
  ${catkin_LIBRARIES}
  px4_lib_fw # for geo.cpp
)

add_executable(fw_plane_visualizer_node
                  src/fw_plane_visualizer_node.cpp)

# link libraries for this lib
target_link_libraries(fw_plane_visualizer_node
   ${catkin_LIBRARIES}
   fw_plane_visualizer
)




link_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/fw_sim/BHDynamic)

add_library(fw_sim_nodelet
            src/fw_sim/fw_sim_node.cpp
            src/fw_sim/fw_driver_sim.cpp)

# link libraries for this lib
target_link_libraries(fw_sim_nodelet
   ${catkin_LIBRARIES}
   px4_lib_fw # for geo.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/src/fw_sim/BHDynamic/libBHDynamic.so
)

add_executable(fw_sim_node
                  src/fw_sim/fw_sim_node.cpp
                  src/fw_sim/fw_driver_sim.cpp)

target_link_libraries(fw_sim_node
   ${catkin_LIBRARIES}
   px4_lib_fw # for geo.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/src/fw_sim/BHDynamic/libBHDynamic.so
)


install(TARGETS 
	px4_lib_fw
	fw_plane_visualizer
	fw_plane_visualizer_node
	fw_sim_node
	fw_sim_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/fw_sim/BHDynamic/libBHDynamic.so
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# directory
install(DIRECTORY plugins
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY model
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
