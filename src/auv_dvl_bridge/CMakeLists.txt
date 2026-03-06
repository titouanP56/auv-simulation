cmake_minimum_required(VERSION 3.8)
project(auv_dvl_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

add_executable(dvl_bridge_node src/dvl_bridge_node.cpp)
ament_target_dependencies(dvl_bridge_node rclcpp geometry_msgs)
target_link_libraries(dvl_bridge_node
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)

install(TARGETS dvl_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
