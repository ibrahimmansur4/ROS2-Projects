cmake_minimum_required(VERSION 3.5)

project(line_follow_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(line_follow_node src/line_following.cpp)

ament_target_dependencies(line_follow_node rclcpp cv_bridge sensor_msgs geometry_msgs OpenCV)
target_link_libraries(line_follow_node ${OpenCV_LIBS})

install(TARGETS line_follow_node DESTINATION lib/${PROJECT_NAME})

ament_package()