cmake_minimum_required(VERSION 2.8.3)
project(odom_tf_converter)
set(CMAKE_BUILD_TYPE Release)
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  tf
)

# find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS} ${csm_INCLUDE_DIRS})
# link_directories(${csm_LIBRARY_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

## System dependencies are found with CMake's conventions

# catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES
  CATKIN_DEPENDS nav_msgs roscpp tf
#  DEPENDS 
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


## Declare a cpp executable
#add_executable(odom_to_tf src/odom_to_tf.cpp)
#target_link_libraries(odom_to_tf
  #${catkin_LIBRARIES}
#)

add_executable(tf_to_odom src/tf_to_odom.cpp)
target_link_libraries(tf_to_odom
  ${catkin_LIBRARIES}
)

#add_executable(pub_pose src/pub_pose.cpp)
#target_link_libraries(pub_pose
  #${catkin_LIBRARIES}
#)


