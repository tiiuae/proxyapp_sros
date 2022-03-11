# Copyright 2021 Proyectos y Sistemas de Mantenimiento SL (eProsima).
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# cmake_minimum_required(VERSION 3.5)

# if(NOT CMAKE_VERSION VERSION_LESS 3.0)
#     cmake_policy(SET CMP0048 NEW)
# endif()

# project(proxyapp)

# # Find requirements
# if(NOT fastcdr_FOUND)
#     find_package(fastcdr REQUIRED)
# endif()

# if(NOT fastrtps_FOUND)
#     find_package(fastrtps REQUIRED)
# endif()

# #Check C++11
# include(CheckCXXCompilerFlag)
# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_IS_CLANG OR
#         CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#     check_cxx_compiler_flag(-std=c++11 SUPPORTS_CXX11)
#     if(NOT SUPPORTS_CXX11)
#         message(FATAL_ERROR "Compiler doesn't support C++11")
#     endif()
# endif()

# message(STATUS "Configuring ProxyApp...")
# set(PROXY_APP_SOURCE
#     ${PROJECT_SOURCE_DIR}/src/cpp/ProxyApp_main.cpp
#     ${PROJECT_SOURCE_DIR}/types/fog_msgs/Vec4.cxx
#     ${PROJECT_SOURCE_DIR}/types/fog_msgs/Vec4PubSubTypes.cxx
#     )

# add_executable(${PROJECT_NAME} ${PROXY_APP_SOURCE})
# target_compile_definitions(${PROJECT_NAME} PRIVATE
#     $<$<AND:$<NOT:$<BOOL:${WIN32}>>,$<STREQUAL:"${CMAKE_BUILD_TYPE}","Debug">>:__DEBUG>
#     $<$<BOOL:${INTERNAL_DEBUG}>:__INTERNALDEBUG> # Internal debug activated.
# )
# target_include_directories(${PROJECT_NAME} PRIVATE
#         ${PROJECT_SOURCE_DIR}
#         ${PROJECT_SOURCE_DIR}/include)
# target_link_libraries(${PROJECT_NAME} fastrtps fastcdr)
# install(TARGETS ${PROJECT_NAME}
#     RUNTIME DESTINATION ${BIN_INSTALL_DIR})

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

add_library(${library_name} SHARED
  src/node_thread.cpp
)

ament_target_dependencies(${library_name}
  rclcpp
  std_msgs
  std_srvs
  nav_msgs
  fog_msgs
)

set(executable_name proxyapp_ros)

add_executable(${executable_name}
  src/ProxyApp_ros.cpp
)


ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs std_srvs nav_msgs fog_msgs)

install(
  TARGETS
    ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

ament_package()