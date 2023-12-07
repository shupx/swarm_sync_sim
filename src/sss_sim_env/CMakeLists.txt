cmake_minimum_required(VERSION 3.0.2)
project(sss_sim_env)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_service_files(
  FILES
  ClientRegister.srv
  ClientUnregister.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

# catkin_package(
#    INCLUDE_DIRS include include/ClockUpdater
#    LIBRARIES ClockUpdater
# )

catkin_package(
   INCLUDE_DIRS include 
   LIBRARIES ClockUpdater
)

add_executable(sim_clock src/sim_clock/sim_clock.cpp
                         src/sim_clock/TimeServer.cpp 
                        )

target_link_libraries(sim_clock
   ${catkin_LIBRARIES}
)

add_library(ClockUpdater
            src/ClockUpdater/ClockUpdater.cpp
            include/ClockUpdater/ClockUpdater.h
          )

# add ClockUpdater.hpp header path for which use this lib
# target_include_directories(ClockUpdater PUBLIC include/ClockUpdater)       

## Mark executables and/or libraries for installation
install(TARGETS ClockUpdater
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation      
install(DIRECTORY include/ClockUpdater/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
