cmake_minimum_required(VERSION 3.16)
project(roomba_driver_rs)

# The driver is a plain cargo project (rclrs + ros-env, which finds the message crates on
# AMENT_PREFIX_PATH, roomba_msgs included). CMake only drives cargo so colcon, ros2 run and launch
# files work as usual, without colcon-ros-cargo.
find_package(ament_cmake REQUIRED)
find_program(CARGO cargo HINTS "$ENV{HOME}/.cargo/bin" REQUIRED)

set(CARGO_TARGET_DIR "${CMAKE_CURRENT_BINARY_DIR}/cargo")
set(NODES roomba_driver)

add_custom_target(cargo_build ALL
  COMMAND ${CMAKE_COMMAND} -E env --unset=MAKEFLAGS --unset=MFLAGS CARGO_TARGET_DIR=${CARGO_TARGET_DIR} ${CARGO} build --release --locked
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  USES_TERMINAL)

foreach(node ${NODES})
  install(PROGRAMS ${CARGO_TARGET_DIR}/release/${node} DESTINATION lib/${PROJECT_NAME})
endforeach()

if(BUILD_TESTING)
  add_test(NAME cargo_test
    COMMAND ${CMAKE_COMMAND} -E env CARGO_TARGET_DIR=${CARGO_TARGET_DIR} ${CARGO} test --release --locked --lib
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endif()

ament_package()
