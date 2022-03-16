cmake_minimum_required(VERSION 3.5)
project(proxyapp_ros)
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(fog_msgs REQUIRED)

include_directories(
  include
)

set(library_name ${executable_name}_core)

set(executable_name proxyapp_ros)

add_executable(${executable_name}
  src/ProxyApp_ros.cpp
)

ament_target_dependencies(${executable_name}
  rclcpp
  std_msgs
  std_srvs
  nav_msgs
  fog_msgs
)

ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs std_srvs nav_msgs fog_msgs)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
# ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

ament_package()