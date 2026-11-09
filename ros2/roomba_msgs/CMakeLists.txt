cmake_minimum_required(VERSION 3.8)
project(roomba_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CleaningMotors.msg"
  "msg/Drive.msg"
  "msg/DriveDirect.msg"
  "msg/DrivePwm.msg"
  "msg/Leds.msg"
  "msg/SchedulingLeds.msg"
  "msg/SensorState.msg"
  "srv/DefineSong.srv"
  "srv/PlaySong.srv"
  "srv/PressButtons.srv"
  "srv/SetDayTime.srv"
  "srv/SetMode.srv"
  "srv/SetSchedule.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
