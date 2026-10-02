cmake_minimum_required(VERSION 3.16)
# 只安装启动文件和参数，不生成单独 ROS 节点。
project(terramind_bringup)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)
find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
ament_package()
