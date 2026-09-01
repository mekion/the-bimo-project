cmake_minimum_required(VERSION 3.8)
project(bimo_ros2_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ActionCommand.msg"
  "msg/StateData.msg"
  "srv/PerformRoutine.srv"
  "srv/CaptureImage.srv"
  "srv/AddRoutine.srv"
  "srv/GetRoutine.srv"
  DEPENDENCIES sensor_msgs
)

ament_package()