#######################################################################################
cmake_minimum_required(VERSION 3.8)
#######################################################################################
project(l3xz_pump_ctrl)
set(L3XZ_HYDRAULIC_CTRL_TARGET ${PROJECT_NAME}_node)
#######################################################################################
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Werror -Wextra -Wpedantic)
endif()
#######################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ros2_heartbeat REQUIRED)
find_package(ros2_loop_rate_monitor REQUIRED)
#######################################################################################
include_directories(${L3XZ_HYDRAULIC_CTRL_TARGET} include)
#######################################################################################
add_executable(${L3XZ_HYDRAULIC_CTRL_TARGET}
  src/Node.cpp
  src/main.cpp
)
#######################################################################################
target_compile_features(${L3XZ_HYDRAULIC_CTRL_TARGET} PRIVATE cxx_std_17)
#######################################################################################
ament_target_dependencies(${L3XZ_HYDRAULIC_CTRL_TARGET} rclcpp std_msgs ros2_heartbeat ros2_loop_rate_monitor)
#######################################################################################
install(TARGETS
  ${L3XZ_HYDRAULIC_CTRL_TARGET}
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
#######################################################################################
ament_package()
#######################################################################################
