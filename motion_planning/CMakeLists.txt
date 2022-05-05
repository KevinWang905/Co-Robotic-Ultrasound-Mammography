# motion_planning cmake

cmake_minimum_required(VERSION 3.0.2)
project(motion_planning)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    interactive_markers
    moveit_core
    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_perception
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_conversions
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    pointgrey_camera_driver
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

set(THIS_PACKAGE_INCLUDE_DIRS
  doc/interactivity/include
)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
    pointgrey_camera_driver
  DEPENDS
    EIGEN3
)



include_directories(${THIS_PACKAGE_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})


add_executable(basic_movement src/basic_movement.cpp)
target_link_libraries(basic_movement ${catkin_LIBRARIES})

add_executable(translational_wobble src/translational_wobble.cpp)
target_link_libraries(translational_wobble ${catkin_LIBRARIES})

add_executable(real_time_movement src/real_time_movement.cpp)
target_link_libraries(real_time_movement ${catkin_LIBRARIES})

add_executable(init_aruco_tf src/init_aruco_tf.cpp)
target_link_libraries(init_aruco_tf ${catkin_LIBRARIES})

add_executable(rotational_wobble src/rotational_wobble.cpp)
target_link_libraries(rotational_wobble ${catkin_LIBRARIES})

add_executable(position_test src/position_test.cpp)
target_link_libraries(position_test ${catkin_LIBRARIES})

add_executable(input_wobble src/input_wobble.cpp)
target_link_libraries(input_wobble ${catkin_LIBRARIES})

add_executable(force_sensor_integration src/force_sensor_integration.cpp)
target_link_libraries(force_sensor_integration ${catkin_LIBRARIES})

add_executable(force_wobble src/force_wobble.cpp)
target_link_libraries(force_wobble ${catkin_LIBRARIES})



