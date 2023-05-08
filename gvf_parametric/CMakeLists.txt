cmake_minimum_required(VERSION 3.0.2)
project(gvf_parametric)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

set(gvf_trajectory_SOURCE_DIR
    src
	)

set(gvf_trajectory_INCLUDE_DIR
    include/gvf_parametric
	)

set(HEADERS
    ${gvf_trajectory_INCLUDE_DIR}/gvf_parametric_utils.h
    ${gvf_trajectory_INCLUDE_DIR}/gvf_parametric_control_2d_spline.h
  )

set(SOURCES
    ${gvf_trajectory_SOURCE_DIR}/gvf_parametric_control_2d_spline.cpp
  )

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs   
)

include_directories(
  ${gvf_trajectory_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(gvf_parametric_control_2d_spline_node ${gvf_trajectory_SOURCE_DIR}/nodes/gvf_parametric_control_2d_spline_node.cpp)
target_link_libraries(gvf_parametric_control_2d_spline_node ${PROJECT_NAME})

add_executable(TEST_trajectory_pub_node ${gvf_trajectory_SOURCE_DIR}/nodes/TEST_trajectory_pub_node.cpp)
target_link_libraries(TEST_trajectory_pub_node ${PROJECT_NAME})