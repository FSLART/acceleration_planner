cmake_minimum_required(VERSION 3.8)
project(acceleration)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(lart_msgs REQUIRED)

include_directories(
  include
  ${CMAKE_SOURCE_DIR}/../lart_common
)

add_executable(acceleration_exec
  src/acceleration_node.cpp
  src/utils.cpp
)

target_include_directories(acceleration_exec PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../lart_common-main>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(acceleration_exec
  ament_index_cpp
  geometry_msgs
  rclcpp
  visualization_msgs
  nav_msgs
  tf2
  std_msgs
  lart_msgs
)

install(TARGETS acceleration_exec
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)


install(FILES src/acceleration_path.csv
  DESTINATION share/${PROJECT_NAME}/data
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
