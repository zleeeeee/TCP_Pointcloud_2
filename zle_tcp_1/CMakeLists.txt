cmake_minimum_required(VERSION 2.8)
project( zle_tcp_1 )

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable( client client.cpp)
target_link_libraries(client ${PCL_LIBRARIES})

add_executable( server server.cpp)
target_link_libraries(server ${PCL_LIBRARIES})



