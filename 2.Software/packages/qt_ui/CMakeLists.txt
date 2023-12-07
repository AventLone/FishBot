cmake_minimum_required(VERSION 3.8)
project(fishbot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Qt dependencies
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)

# Find ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rviz_common REQUIRED)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
file(GLOB FORM_DIR RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/ *.ui)
file(GLOB RESOURCE_DIR RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resource/ *.qrc)
file(GLOB_RECURSE LIB_DIR RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS lib/ *.h *.cpp)
file(GLOB_RECURSE WINDOW_DIR RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS window/ *.h *.cpp)
file(GLOB_RECURSE SOURCE_DIR RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/ *.cpp )

include_directories(lib window)

add_executable(${PROJECT_NAME}
    ${FORM_DIR}
    ${RESOURCES_DIR}
    ${DEPENDENCIES_DIR}
    ${WINDOW_DIR}
    ${SOURCES_DIR}
)

ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    std_msgs
    rviz_common
)

target_link_libraries(${PROJECT_NAME} Qt${QT_VERSION_MAJOR}::Widgets)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
