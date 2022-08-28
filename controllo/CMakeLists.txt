cmake_minimum_required(VERSION 3.5)
project(controllo)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo REQUIRED COMPONENTS gazebo_msgs)

get_filename_component(PX4_MSGS_DIR "../" ABSOLUTE BASE_DIR ${px4_msgs_DIR})

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_executable(offboard src/controllo.cpp)
ament_target_dependencies(offboard rclcpp px4_msgs)
install(TARGETS offboard DESTINATION lib/${PROJECT_NAME})

add_executable(comando src/comando.cpp)
ament_target_dependencies(comando rclcpp px4_msgs)
install(TARGETS comando DESTINATION lib/${PROJECT_NAME})

add_executable(mocap src/prova_mocap.cpp)
ament_target_dependencies(mocap rclcpp px4_msgs gazebo_msgs)
install(TARGETS mocap DESTINATION lib/${PROJECT_NAME})
target_link_libraries(mocap ${GAZEBO_LIBRARIES} pthread)

add_executable(mocap_qualisys src/mocap_qualisys.cpp)
ament_target_dependencies(mocap_qualisys rclcpp px4_msgs)
install(TARGETS mocap_qualisys DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
