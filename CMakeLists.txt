cmake_minimum_required(VERSION 3.0.2)
project(use_lucid)

IF(NOT ROS_BUILD_TYPE)
  SET(ROS_BUILD_TYPE Release)
ENDIF()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-W -Wall -O3 -march=native -msse")
set(CMAKE_MODULE_PATH "/home/bhrqhb/catkin_ws/src/use_Lucid/cmake")

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  image_geometry
  image_transport
)
find_package(PCL 1.8 REQUIRED)
find_package(arena_sdk REQUIRED)

# find_package(arena_sdk
#     # MODULE
#     REQUIRED
#     PATHS ${CMAKE_CURRENT_SOURCE_DIR}/cmake/Findarena_sdk.cmake
# )


if (NOT ${arena_sdk_FOUND})
    include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/Findarena_sdk.cmake")
endif()

catkin_package(
    INCLUDE_DIRS
      include
    LIBRARIES
      ${PROJECT_NAME}
    CATKIN_DEPENDS
      ${CATKIN_COMPONENTS}
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
# set(use_lucid_name use_lucid)
add_executable(
    getPointCloud  # runs by rosrun
    src/getPointCloud.cpp
)
target_include_directories(
    getPointCloud
        PRIVATE ${catkin_INCLUDE_DIRS} 
        PRIVATE ${arena_sdk_INCLUDE_DIRS}
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(
    getPointCloud
        ${catkin_LIBRARIES}
        ${arena_sdk_LIBRARIES}
        ${PCL_LIBRARIES}
)

add_executable(
  triggerMode  # runs by rosrun
    src/triggerMode.cpp
)
target_include_directories(
  triggerMode
        PRIVATE ${catkin_INCLUDE_DIRS} 
        PRIVATE ${arena_sdk_INCLUDE_DIRS}
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(
  triggerMode
        ${catkin_LIBRARIES}
        ${arena_sdk_LIBRARIES}
        ${PCL_LIBRARIES}
)


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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_use_lucid.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
