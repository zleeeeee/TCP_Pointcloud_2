cmake_minimum_required(VERSION 3.0.2)
project(data_tcp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

find_package(PCL REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL 

  CATKIN_DEPENDS
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable( client
 src/client.cpp
)
target_link_libraries(client
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES} 
)

add_executable( server
 src/server.cpp
)
target_link_libraries(server
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES} 
)


