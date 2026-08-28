#!/bin/bash
# One-time environment setup: fetches the ARM cross-compiler and the
# pico-sdk into third_party/, both of which are gitignored. Idempotent —
# every step checks first, so re-running costs seconds and never disturbs
# a working tree.
#
# macOS on Apple Silicon. Elsewhere, change the toolchain URL below.
set -e
cd "$(dirname "$0")/.."

brew list cmake >/dev/null 2>&1 || brew install cmake

# pico-sdk 2.x shells out to picotool to turn the .elf into the .uf2. Without
# it CMake downloads and builds its own copy at configure time, which works
# silently with a network and fails on an offline clone with an error that
# never mentions picotool.
brew list picotool >/dev/null 2>&1 || brew install picotool || \
  echo "WARN: picotool not installed - the SDK will fetch and build its own (needs network)"

# ARM GNU toolchain 14.2. Pinned to an exact build rather than taken from
# Homebrew so every clone compiles with the same compiler.
# cmake/pico_robot.cmake points PICO_TOOLCHAIN_PATH here; no PATH edits needed.
if [ ! -x third_party/arm-gnu-toolchain/bin/arm-none-eabi-gcc ]; then
  mkdir -p third_party/arm-gnu-toolchain
  cd third_party/arm-gnu-toolchain
  curl -L -o arm-gnu.tar.xz "https://developer.arm.com/-/media/Files/downloads/gnu/14.2.rel1/binrel/arm-gnu-toolchain-14.2.rel1-darwin-arm64-arm-none-eabi.tar.xz"
  tar xf arm-gnu.tar.xz --strip-components=1 && rm arm-gnu.tar.xz
  cd ../..
fi

# pico-sdk 2.2.0, plus tinyusb for USB serial.
if [ ! -d third_party/pico-sdk ]; then
  git clone --depth 1 --branch 2.2.0 https://github.com/raspberrypi/pico-sdk third_party/pico-sdk
  git -C third_party/pico-sdk submodule update --init --depth 1 lib/tinyusb
fi

echo "OK: $(third_party/arm-gnu-toolchain/bin/arm-none-eabi-gcc --version | head -1)"
echo "Build and flash with: cmake -S . -B build && make -C build -j8 flash"
