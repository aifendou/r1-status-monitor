cmake_minimum_required(VERSION 3.16)
project(r1_status_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rpp_msgs REQUIRED)

# 系统线程支持。
find_package(Threads REQUIRED)

# Muduo依赖。
find_path(MUDUO_INCLUDE_DIR
  NAMES muduo/net/TcpServer.h
)
find_library(MUDUO_NET_LIBRARY
  NAMES muduo_net
)

find_library(MUDUO_BASE_LIBRARY
  NAMES muduo_base
)
if(NOT MUDUO_INCLUDE_DIR
   OR NOT MUDUO_NET_LIBRARY
   OR NOT MUDUO_BASE_LIBRARY)
  message(FATAL_ERROR "Muduo headers or libraries not found.")
endif()
# 原来的独立ROS订阅实验。
add_executable(odom_subscriber
  src/odom_subscriber.cpp
)

# 新增：ROS订阅与TCP查询整合版。
add_executable(r1_status_server
  src/main.cpp
)
# 为这个程序配置ROS依赖。
ament_target_dependencies(odom_subscriber
  rclcpp
  nav_msgs
)
ament_target_dependencies(r1_status_server
    rclcpp
    nav_msgs
    rpp_msgs
)
# 自己的头文件：正常检查编译警告。
target_include_directories(r1_status_server PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../common"
)

# 第三方Muduo头文件：标记为系统头文件。
target_include_directories(r1_status_server SYSTEM PRIVATE
  "${MUDUO_INCLUDE_DIR}"
)

target_link_libraries(r1_status_server
  "${MUDUO_NET_LIBRARY}"
  "${MUDUO_BASE_LIBRARY}"
  Threads::Threads
)

# 安装到ROS能够通过ros2 run找到的位置。
install(TARGETS odom_subscriber r1_status_server
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
