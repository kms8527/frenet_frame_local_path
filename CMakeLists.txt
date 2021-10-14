cmake_minimum_required(VERSION 3.10)
project(frenet_local_path)


set(CMAKE_CXX_STANDARD 14)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_FLAGS "-march=native -O3 -pthread -Wall")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")

find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Eigen3)
find_package(catkin REQUIRED COMPONENTS
roscpp
rospy
std_msgs
geometry_msgs
message_generation
sensor_msgs
pcl_conversions
pcl_ros
)

add_message_files(
    FILES
    waypoint.msg
    frenet_input.msg
    frenet_output.msg
    frenet_obstacle.msg
 )
generate_messages(
    DEPENDENCIES
    std_msgs
    sensor_msgs
    geometry_msgs
)


catkin_package(CATKIN_DEPENDS rospy roscpp pcl_ros message_runtime sensor_msgs pcl_conversions)

include_directories(${catkin_INCLUDE_DIRS})

if(Eigen3_FOUND)
    set(CUSTOM_EIGEN ${EIGEN3_INCLUDE_DIR})
else()
    find_package(Eigen)
    set(CUSTOM_EIGEN ${EIGEN_INCLUDE_DIR})
endif()

include_directories(.)
include_directories(
        ${CUSTOM_EIGEN}
)

include_directories("${CMAKE_SOURCE_DIR}/src"
                    "${CMAKE_SOURCE_DIR}/src/CubicSpline"
                    "${CMAKE_SOURCE_DIR}/src/Polynomials"
                    "${CMAKE_SOURCE_DIR}/src/FrenetOptimalTrajectory"
                    "${CMAKE_SOURCE_DIR}/src/Obstacle"
                    "${CMAKE_SOURCE_DIR}/src/Car")

add_library(FrenetOptimalTrajectory SHARED
            src/main.cpp
            src/utils.h
            src/Polynomials/QuarticPolynomial.cpp
            src/Polynomials/QuarticPolynomial.h
            src/Polynomials/QuinticPolynomial.cpp
            src/Polynomials/QuinticPolynomial.h
            src/CubicSpline/CubicSpline1D.cpp
            src/CubicSpline/CubicSpline1D.h
            src/CubicSpline/CubicSpline2D.cpp
            src/CubicSpline/CubicSpline2D.h
            src/FrenetOptimalTrajectory/FrenetOptimalTrajectory.cpp
            src/FrenetOptimalTrajectory/FrenetOptimalTrajectory.h
            src/FrenetOptimalTrajectory/FrenetPath.cpp
            src/FrenetOptimalTrajectory/FrenetPath.h
            src/FrenetOptimalTrajectory/fot_wrapper.cpp
            src/FrenetOptimalTrajectory/py_cpp_struct.h
            src/Obstacle/Obstacle.cpp
            src/Obstacle/Obstacle.h
            src/Car/Car.cpp
            src/Car/Car.h)
add_executable(FrenetOptimalTrajectoryTest
               src/main.cpp
               src/utils.h
               src/Polynomials/QuarticPolynomial.cpp
               src/Polynomials/QuarticPolynomial.h
               src/Polynomials/QuinticPolynomial.cpp
               src/Polynomials/QuinticPolynomial.h
               src/CubicSpline/CubicSpline1D.cpp
               src/CubicSpline/CubicSpline1D.h
               src/CubicSpline/CubicSpline2D.cpp
               src/CubicSpline/CubicSpline2D.h
               src/FrenetOptimalTrajectory/FrenetOptimalTrajectory.cpp
               src/FrenetOptimalTrajectory/FrenetOptimalTrajectory.h
               src/FrenetOptimalTrajectory/FrenetPath.cpp
               src/FrenetOptimalTrajectory/FrenetPath.h
               src/FrenetOptimalTrajectory/fot_wrapper.cpp
               src/FrenetOptimalTrajectory/py_cpp_struct.h
               src/Obstacle/Obstacle.cpp
               src/Obstacle/Obstacle.h
               src/Car/Car.cpp
               src/Car/Car.h)
target_link_libraries(FrenetOptimalTrajectory Eigen3::Eigen Qt5::Widgets Qt5::Gui)
target_link_libraries(FrenetOptimalTrajectoryTest Eigen3::Eigen Qt5::Widgets Qt5::Gui)
target_link_libraries(
  FrenetOptimalTrajectory
  ${catkin_LIBRARIES}
)

target_link_libraries(
  FrenetOptimalTrajectoryTest
  ${catkin_LIBRARIES}
)
add_dependencies(FrenetOptimalTrajectory ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(FrenetOptimalTrajectoryTest ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

