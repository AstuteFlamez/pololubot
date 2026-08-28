cmake_minimum_required(VERSION 3.13)
include(cmake/pico_robot.cmake)
project(pololubot C CXX ASM)
pico_sdk_init()

robot_executable(pololubot
  src/main.c
  src/hw_millis.c
  src/hw_motors.c
  src/hw_encoders.c
  src/hw_imu.c
  src/hw_line.c
  src/hw_battery.c
  src/hw_buttons.c
  src/ui.c
  src/drive.c
  src/feedback.c
  src/lap_timer.c
  src/telemetry.c
  logic/maze_logic.c        # the maze brain, cross-compiled for the robot
  logic/maze_logic_ref.c    # and its simpler fallback classifier
)

# logic/ compiles here with the same flags it gets from plain gcc under
# tests/logic/. That is the point of keeping it free of SDK headers.
target_include_directories(pololubot PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/include
  ${CMAKE_CURRENT_LIST_DIR}/logic
)

target_link_libraries(pololubot
  pololu_3pi_2040_robot   # display and RGB LEDs only; every other driver is ours
  hardware_pwm
  hardware_adc
  hardware_i2c
)
