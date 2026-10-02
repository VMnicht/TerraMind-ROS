cmake_minimum_required(VERSION 3.16)
# 集中生成自定义消息；新增 .msg 必须在 rosidl_generate_interfaces 中登记。
project(terramind_interfaces)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
 "msg/ImplementCommand.msg"
 "msg/ControlCommand.msg"
 "msg/ChassisState.msg"
 "msg/SpreaderState.msg"
 "msg/MowerState.msg"
 "msg/LiftState.msg"
 "msg/SprayerState.msg"
 "msg/McuState.msg"
 "msg/ControlStatus.msg"
 DEPENDENCIES std_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
