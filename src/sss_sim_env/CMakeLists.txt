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
  nodelet
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_message_files(
  FILES
  TimeRequest.msg
)

add_service_files(
  FILES
  ClientRegister.srv
  ClientUnregister.srv
  SimClockControl.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

# Output include directories and libraries for other package to use
catkin_package(
   INCLUDE_DIRS include 
   LIBRARIES ClockUpdater sss_timer sss_sleep
   CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

# add_executable(sim_clock src/sim_clock/sim_clock.cpp
#                          src/sim_clock/TimeServer.cpp 
#                         )

# target_link_libraries(sim_clock
#    ${catkin_LIBRARIES}
# )

############## sim_clock nodelet plugin lib ################

add_library(sim_clock 
                src/sim_clock/sim_clock.cpp
                src/sim_clock/TimeServer.cpp)

target_link_libraries(sim_clock
                      ${catkin_LIBRARIES}
                   )

add_dependencies(sim_clock 
                      ${${PROJECT_NAME}_EXPORTED_TARGETS} 
                      ${catkin_EXPORTED_TARGETS} )


##################### sss_utils lib #######################

add_library(ClockUpdater
              src/sss_utils/ClockUpdater.cpp
          )

add_dependencies(ClockUpdater 
                       ${${PROJECT_NAME}_EXPORTED_TARGETS} 
                       ${catkin_EXPORTED_TARGETS} )

# add ClockUpdater.hpp header path for which use this lib
# target_include_directories(ClockUpdater PUBLIC include/sss_sim_env)       

add_library(sss_timer
              src/sss_utils/Timer.cpp
          )

add_dependencies(sss_timer 
                       ${${PROJECT_NAME}_EXPORTED_TARGETS} 
                       ${catkin_EXPORTED_TARGETS} )

target_link_libraries(sss_timer
          ${catkin_LIBRARIES}
          ClockUpdater
       )

add_library(sss_sleep
       src/sss_utils/Sleep.cpp
   )

add_dependencies(sss_sleep 
                ${${PROJECT_NAME}_EXPORTED_TARGETS} 
                ${catkin_EXPORTED_TARGETS} )

target_link_libraries(sss_sleep
   ${catkin_LIBRARIES}
   ClockUpdater
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
# install(TARGETS ClockUpdater
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# ## Mark cpp header files for installation      
# install(DIRECTORY include/sss_sim_env/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
# )

# scripts
install(PROGRAMS
	scripts/clock_ui.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
	scripts/clock.png
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
	scripts/ui/main_window.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/ui
)

# targets
install(TARGETS sim_clock ClockUpdater sss_timer sss_sleep
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# directory
install(DIRECTORY plugins
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
