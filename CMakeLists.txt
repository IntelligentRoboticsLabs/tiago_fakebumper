cmake_minimum_required(VERSION 3.5)
project(tiago_fakebumper)

set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH}  "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(kobuki_ros_interfaces REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
    rclcpp
    kobuki_ros_interfaces
    sensor_msgs
)

add_executable(fake_bumper src/fake_bumper.cpp)
ament_target_dependencies(fake_bumper ${dependencies})

install(TARGETS
  fake_bumper
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${dependencies})

ament_package()
