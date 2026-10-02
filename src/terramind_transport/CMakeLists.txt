cmake_minimum_required(VERSION 3.16)
# Linux 串口传输库；PTY 测试需要链接系统 util 库。
project(terramind_transport)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)
find_package(ament_cmake REQUIRED)


add_library(${PROJECT_NAME} SHARED src/serial_port.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME} ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies()

if(BUILD_TESTING)
 find_package(ament_cmake_gtest REQUIRED)
 ament_add_gtest(transport_test test/transport_test.cpp)
target_link_libraries(transport_test ${PROJECT_NAME} util)
endif()

ament_package()
