cmake_minimum_required(VERSION 3.5)
project(jetson_camera)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV 4 REQUIRED)


add_library(${PROJECT_NAME}_component SHARED src/JetsonCameraNode.cpp)
rclcpp_components_register_nodes(${PROJECT_NAME}_component "jetson_camera::JetsonCameraNode")

target_include_directories(${PROJECT_NAME}_component PRIVATE ${OpenCV_INCLUDE_DIRS})

ament_target_dependencies(
    ${PROJECT_NAME}_component
    rclcpp
    rclcpp_components
    camera_info_manager
    cv_bridge
    image_transport
    sensor_msgs
)

install(
    TARGETS ${PROJECT_NAME}_component
    EXPORT export_${PROJECT_NAME}_component
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

