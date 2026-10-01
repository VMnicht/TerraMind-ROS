cmake_minimum_required(VERSION 3.16)
project(terramind_protocol)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)
find_package(ament_cmake REQUIRED)


add_library(${PROJECT_NAME} SHARED src/crc16.cpp src/codec.cpp src/frame_parser.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME} ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies()

if(BUILD_TESTING)
 find_package(ament_cmake_gtest REQUIRED)
 ament_add_gtest(protocol_test test/protocol_test.cpp)
target_link_libraries(protocol_test ${PROJECT_NAME})
endif()

ament_package()
