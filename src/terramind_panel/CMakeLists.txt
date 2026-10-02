cmake_minimum_required(VERSION 3.16)
# 通过 ament_cmake_python 安装 Python 模块和可执行入口，Qt 测试使用离屏平台。
project(terramind_panel)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/control_panel scripts/debug_panel scripts/panel_integration_test.py
  scripts/debug_launcher_integration_test.py DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(panel_controls test/test_controls.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR} TIMEOUT 30)
  ament_add_pytest_test(debug_launcher test/test_debug_launcher.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
    ENV QT_QPA_PLATFORM=offscreen TIMEOUT 90)
endif()
ament_package()
