# Toolchain and SDK wiring for the 3pi+ 2040.
#
# Everything the build needs lives under third_party/ — the cross-compiler,
# the pico-sdk, and Pololu's display/LED library. tools/setup.sh puts the
# first two there; only the third is committed. Nothing here reads an
# environment variable, so a fresh clone builds the same way on every
# machine or it doesn't build at all.

set(ROBOT_ROOT "${CMAKE_CURRENT_LIST_DIR}/..")

set(PICO_SDK_PATH "${ROBOT_ROOT}/third_party/pico-sdk" CACHE PATH "")
set(PICO_TOOLCHAIN_PATH "${ROBOT_ROOT}/third_party/arm-gnu-toolchain" CACHE PATH "")

# Board file: flash size, crystal frequency, default pin roles.
set(PICO_BOARD pololu_3pi_2040_robot CACHE STRING "")

include(${PICO_SDK_PATH}/pico_sdk_init.cmake)

# Builds one firmware image with the settings every program on this robot
# wants: warnings on, USB serial for printf and the 1200-baud reflash knock,
# and a .uf2 next to the .elf.
function(robot_executable name)
  # Pololu's library can only be added after pico_sdk_init() has run, which
  # is why it is pulled in from here rather than at the top of the file.
  if (NOT TARGET pololu_3pi_2040_robot)
    add_subdirectory(${ROBOT_ROOT}/third_party/pololu_3pi_2040_robot
                     ${CMAKE_BINARY_DIR}/pololu_lib EXCLUDE_FROM_ALL)
  endif()

  add_executable(${name} ${ARGN})
  target_include_directories(${name} PRIVATE ${ROBOT_ROOT}/include)
  target_compile_options(${name} PRIVATE -Wall -Wextra)
  pico_enable_stdio_usb(${name} 1)
  pico_enable_stdio_uart(${name} 0)
  pico_add_extra_outputs(${name})     # .uf2, .dis, .map alongside the .elf
  target_link_libraries(${name} pico_stdlib)

  add_custom_target(flash
    COMMAND ${ROBOT_ROOT}/tools/flash.sh ${name}.uf2
    DEPENDS ${name}
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
endfunction()
