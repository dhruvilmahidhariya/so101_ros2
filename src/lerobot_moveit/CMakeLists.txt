cmake_minimum_required(VERSION 3.22)
project(lerobot_moveit)

find_package(ament_cmake REQUIRED)

ament_package()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
file(GLOB _lerobot_moveit_scripts "${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.py")
if(_lerobot_moveit_scripts)
  install(PROGRAMS ${_lerobot_moveit_scripts} DESTINATION lib/${PROJECT_NAME})
endif()
