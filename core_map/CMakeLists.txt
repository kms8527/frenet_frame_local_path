cmake_minimum_required(VERSION 3.1)

add_compile_options(-std=c++11)

project(core_map)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  message_generation
  core_control
)

add_message_files(
    FILES
 
    point.msg # x, y, z 좌표
    waypoint.msg # x, y, z 좌표 및 velocity
    a1_lane.msg
    a3_link.msg
    road_info.msg # Left&Right 차선정보와 현재 지나는 링크 정보
    path.msg # a3_link정보 + 변경하는 차선(src & dst)
    global_path.msg # Global path
 
    lic_link.msg
    lic_path.msg
 
    traffic_light.msg
    traffic_light_info.msg
    links.msg

    frenet_input.msg
    frenet_output.msg
    frenet_obstacle.msg
)
 
add_service_files(
    FILES
    CheckProfit.srv
    RequestPath.srv
    RequestPath_lic.srv
    Bsd.srv
    Intersection.srv
    SimCallReq.srv
    FrontIrr.srv
)

generate_messages(
    DEPENDENCIES
    sensor_msgs
    std_msgs
)
 

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  j2735
  skeletons
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  j2735
)

#file(GLOB_RECURSE project_HEADERS
#    "*.h"
#    "*.hpp"
#)

add_executable(
 core_map_node
 include/hdmap_info.h
 include/HDMap.h
 include/Dijkstra.h
 include/Trajectory.h
 include/Multicall.h
 include/Coremap.h

 #v2x 관련
 include/dsrc_msg_id.h
 include/obu_message.h
 include/crc16.h
 include/Signal.h

 src/HDMap.cpp
 src/Dijkstra.cpp
 src/Trajectory.cpp
 src/Multicall.cpp
 src/Coremap.cpp
 src/main.cpp
)
target_link_libraries(
  core_map_node
  ${catkin_LIBRARIES}
  asncodec
)

add_dependencies(core_map_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

