# 要求CMake版本不低于3.16
cmake_minimum_required(VERSION 3.16)

# 项目名称，使用C++
project(r1_monitor_demo LANGUAGES CXX)

# 使用C++17标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 查找平台对应的线程库
find_package(Threads REQUIRED)
find_package(nlohmann_json REQUIRED)

# 查找Muduo头文件所在的根目录
# 例如找到 /usr/local/include，
# 这样才能使用 #include <muduo/net/TcpServer.h>。
find_path(MUDUO_INCLUDE_DIR
    NAMES muduo/net/TcpServer.h
)

# 查找Muduo网络库和基础库
find_library(MUDUO_NET_LIBRARY
    NAMES muduo_net
)

find_library(MUDUO_BASE_LIBRARY
    NAMES muduo_base
)

# 提前检查依赖，避免后面出现难以理解的编译错误
if(NOT MUDUO_INCLUDE_DIR
   OR NOT MUDUO_NET_LIBRARY
   OR NOT MUDUO_BASE_LIBRARY)
    message(FATAL_ERROR
        "Muduo not found. Check its header and library paths."
    )
endif()

# 从server.cpp生成可执行程序
add_executable(r1_status_server
    server.cpp
)
# 构建C++监测客户端。
add_executable(r1_status_client
    client.cpp
)
# 构建独立的消息边界测试程序。
add_executable(frame_codec_test
    frame_codec_test.cpp
)

# 告诉编译器在哪里寻找Muduo头文件
target_include_directories(r1_status_server PRIVATE
    "${MUDUO_INCLUDE_DIR}"
)
target_include_directories(r1_status_client PRIVATE
    "${MUDUO_INCLUDE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/../common"
)
target_include_directories(frame_codec_test PRIVATE
    "${MUDUO_INCLUDE_DIR}"
)

# 告诉链接器需要链接哪些库
target_link_libraries(r1_status_server PRIVATE
    "${MUDUO_NET_LIBRARY}"
    "${MUDUO_BASE_LIBRARY}"
    Threads::Threads

)
target_link_libraries(r1_status_client PRIVATE
    "${MUDUO_NET_LIBRARY}"
    "${MUDUO_BASE_LIBRARY}"
    Threads::Threads
    nlohmann_json::nlohmann_json
)
target_link_libraries(frame_codec_test PRIVATE
    "${MUDUO_NET_LIBRARY}"
    "${MUDUO_BASE_LIBRARY}"
    Threads::Threads
)

# 配置时打印找到的路径，便于检查
message(STATUS "Muduo include: ${MUDUO_INCLUDE_DIR}")
message(STATUS "Muduo net:     ${MUDUO_NET_LIBRARY}")
message(STATUS "Muduo base:    ${MUDUO_BASE_LIBRARY}")