cmake_minimum_required(VERSION 2.8.3)
project(group4_final_project)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  moveit_ros_planning_interface
  osrf_gear
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  trajectory_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include/ariac_manager
#  LIBRARIES ariac_manager
  CATKIN_DEPENDS geometry_msgs moveit_ros_planning_interface osrf_gear roscpp sensor_msgs std_msgs std_srvs trajectory_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include/ariac_manager ${catkin_INCLUDE_DIRS} )


## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/ariac_manager_node.cpp)
add_executable(main_node src/main.cpp src/robot_controller.cpp src/sensor.cpp src/order_manager.cpp)


## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(main_node  ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
  target_link_libraries(main_node
    ${catkin_LIBRARIES}
  )

#############
## Install ##
#############


 install(TARGETS main_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
 )

## Mark other files for installation (e.g. launch and bag files, etc.)
 install(FILES
         config/my_sample_sensors.yaml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
 )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ariac_manager.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
