cmake_minimum_required(VERSION 3.8)
project(opera_tools)

option(BUILD_TESTING "Build tests" ON)
set(CMAKE_BUILD_TYPE Debug)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_PREFIX_PATH /usr/local/lib)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(com3_msgs REQUIRED)
find_package(tms_msg_rp REQUIRED)

# Subtask Nodes for CrawlerDump
add_executable(blade_control src/Bulldozer/blade_control.cpp)
add_executable(swing_control src/CrawlerDump/swing_control.cpp)
add_executable(vessel_control src/CrawlerDump/vessel_control.cpp)
add_executable(vessel_control_updown src/CrawlerDump/vessel_control_updown.cpp)

set(TARGETS
  blade_control
  swing_control
  vessel_control
  vessel_control_updown
)

set(dependencies_cpp
  rclcpp
  rclcpp_action
  std_msgs
  com3_msgs
  sensor_msgs
  tms_msg_rp
)

foreach(target IN LISTS TARGETS)
  target_include_directories(${target} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_directories(${target} PUBLIC /usr/local/lib)

  ament_target_dependencies(${target} ${dependencies_cpp})
endforeach()

install(TARGETS ${TARGETS}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
