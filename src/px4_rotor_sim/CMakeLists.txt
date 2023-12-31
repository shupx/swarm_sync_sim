cmake_minimum_required(VERSION 3.0.2)
project(px4_rotor_sim)

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
  sss_sim_env # for sss_utils
  nodelet # for mavros_px4_quadrotor_sim_node.cpp nodelet
)

#uncomment the following 4 lines to use the Eigen library
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

include_directories(
  include
  src/mavros_px4_quadrotor_sim/px4_modules/px4_lib # for px4_modules build
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  #  CATKIN_DEPENDS sss_sim_env
   INCLUDE_DIRS include
)

add_library(quadrotor_dynamics src/mavros_px4_quadrotor_sim/quadrotor_dynamics.cpp)

add_library(px4_lib
              src/mavros_px4_quadrotor_sim/px4_modules/px4_lib/drivers/drv_hrt.cpp
              src/mavros_px4_quadrotor_sim/px4_modules/px4_lib/hysteresis/hysteresis.cpp
              src/mavros_px4_quadrotor_sim/px4_modules/px4_lib/geo/geo.cpp
              src/mavros_px4_quadrotor_sim/px4_modules/px4_lib/parameters/px4_parameters.cpp
              src/mavros_px4_quadrotor_sim/px4_modules/px4_lib/uORB/uORB_sim.cpp) 

add_library(mc_pos_control
              src/mavros_px4_quadrotor_sim/px4_modules/mc_pos_control/PositionControl/ControlMath.cpp
              src/mavros_px4_quadrotor_sim/px4_modules/mc_pos_control/PositionControl/PositionControl.cpp
              src/mavros_px4_quadrotor_sim/px4_modules/mc_pos_control/Takeoff/Takeoff.cpp
              src/mavros_px4_quadrotor_sim/px4_modules/mc_pos_control/MulticopterPositionControl.cpp
              )
# link libraries for this lib
target_link_libraries(mc_pos_control 
                  px4_lib)

# add *.hpp header path for which use this lib
target_include_directories(mc_pos_control PUBLIC
            src/mavros_px4_quadrotor_sim)


add_library(mc_att_control
              src/mavros_px4_quadrotor_sim/px4_modules/mc_att_control/AttitudeControl/AttitudeControl.cpp
              src/mavros_px4_quadrotor_sim/px4_modules/mc_att_control/mc_att_control_main.cpp
              )

# link libraries for this lib
target_link_libraries(mc_att_control 
                  px4_lib)

# add *.hpp header path for which use this lib
target_include_directories(mc_att_control PUBLIC
            src/mavros_px4_quadrotor_sim)


add_library(commander
            src/mavros_px4_quadrotor_sim/px4_modules/commander/Commander.cpp
            src/mavros_px4_quadrotor_sim/px4_modules/commander/state_machine_helper.cpp
            )

# link libraries for this lib
target_link_libraries(commander 
                px4_lib)

# add *.hpp header path for which use this lib
target_include_directories(commander PUBLIC
          src/mavros_px4_quadrotor_sim)


add_library(px4_mavlink
              src/mavros_px4_quadrotor_sim/px4_modules/mavlink/mavlink_msg_list.cpp
              src/mavros_px4_quadrotor_sim/px4_modules/mavlink/mavlink_streamer.cpp
              src/mavros_px4_quadrotor_sim/px4_modules/mavlink/mavlink_receiver.cpp
            )

# link libraries for this lib
target_link_libraries(px4_mavlink 
                px4_lib)

# add *.hpp header path for which use this lib
target_include_directories(px4_mavlink PUBLIC
          src/mavros_px4_quadrotor_sim)

# Supress the warnings of packed member in mavlink c_library_v2
# Refer to http://mavlink.io/zh/mavgen_c/#build-warnings
target_compile_options(px4_mavlink PUBLIC -Wno-address-of-packed-member -Wno-cast-align)


add_library(px4_sitl
              src/mavros_px4_quadrotor_sim/px4_sitl.cpp
            )

# link libraries for this lib
target_link_libraries(px4_sitl
    ${catkin_LIBRARIES}
    mc_att_control
    mc_pos_control
    commander
    px4_mavlink
)

# Supress the warnings of packed member in mavlink c_library_v2
# Refer to http://mavlink.io/zh/mavgen_c/#build-warnings
target_compile_options(px4_sitl PUBLIC -Wno-address-of-packed-member -Wno-cast-align)


add_library(MavrosSim
            src/mavros_px4_quadrotor_sim/mavros_sim/MavrosSim.cpp
            src/mavros_px4_quadrotor_sim/mavros_sim/lib/uas_data.cpp
            src/mavros_px4_quadrotor_sim/mavros_sim/lib/uas_stringify.cpp
            src/mavros_px4_quadrotor_sim/mavros_sim/lib/uas_timesync.cpp
          )

# add *.hpp header path for which use this lib
target_include_directories(MavrosSim PUBLIC
            src/mavros_px4_quadrotor_sim)

# link libraries for this lib
target_link_libraries(MavrosSim
  ${catkin_LIBRARIES}
  # ${catkin_LIBRARIES} for libeigen_conversions.so from eigen_conversions ROS package
  # ${catkin_LIBRARIES} for libmavros.so from eigen_conversions ROS package
)

# Supress the warnings of packed member in mavlink c_library_v2
# Refer to http://mavlink.io/zh/mavgen_c/#build-warnings
target_compile_options(MavrosSim PUBLIC -Wno-address-of-packed-member -Wno-cast-align)


add_library(drone_visualizer
            src/mavros_px4_quadrotor_sim/drone_visualizer.cpp)

# link libraries for this lib
target_link_libraries(drone_visualizer
  ${catkin_LIBRARIES}
  px4_lib # for geo.cpp
)


# add_executable(mavros_px4_quadrotor_sim_node
#                   src/mavros_px4_quadrotor_sim/mavros_px4_quadrotor_sim_node.cpp)

add_library(mavros_px4_quadrotor_sim_nodelet
            src/mavros_px4_quadrotor_sim/mavros_px4_quadrotor_sim_node.cpp)

# link libraries for this lib
target_link_libraries(mavros_px4_quadrotor_sim_nodelet
   ${catkin_LIBRARIES}
   quadrotor_dynamics
   px4_sitl
   MavrosSim
   drone_visualizer
)

add_executable(drone_visualizer_node
                  src/mavros_px4_quadrotor_sim/drone_visualizer_node.cpp)

# link libraries for this lib
target_link_libraries(drone_visualizer_node
   ${catkin_LIBRARIES}
   drone_visualizer
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

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
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
# catkin_package(
# #  INCLUDE_DIRS include
# #  LIBRARIES sss_sim_agent
# #  CATKIN_DEPENDS roscpp rospy std_msgs
# #  DEPENDS system_lib
# )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(
# # include
#   ${catkin_INCLUDE_DIRS}
# )

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/sss_sim_agent.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/sss_sim_agent_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_sss_sim_agent.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
