cmake_minimum_required(VERSION 3.16)
# 安装 URDF 模板和 RViz 配置；具体轮径/轮距由仿真 launch 替换。
project(terramind_description)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)
find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf rviz DESTINATION share/${PROJECT_NAME})
ament_package()
