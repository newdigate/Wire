cmake_minimum_required(VERSION 3.24)
project(wire_slave_test)

set(TEENSY_VERSION 117 CACHE STRING "")

# This gate lives in the Wire library repo (Wire/tests/wire_slave_test/): it compiles
# Wire.h + WireIMXRT1176.* from THIS repo (the core's own Wire driver was
# removed) while borrowing the evkb checkout for teensy-cmake-macros + the rest
# of the imxrt1176 core.
if(NOT DEFINED EVKB)
    set(EVKB $ENV{HOME}/Development/rt1170/evkb)
endif()

include(FetchContent)
FetchContent_Declare(teensy_cmake_macros SOURCE_DIR ${EVKB}/teensy-cmake-macros)
FetchContent_MakeAvailable(teensy_cmake_macros)
include(${teensy_cmake_macros_SOURCE_DIR}/CMakeLists.include.txt)

# Core without its Wire driver (removed in the move).
import_arduino_library(cores ${EVKB}/cores/imxrt1176)
# Wire from THIS repo (adds the include dir so #include "Wire.h" resolves here,
# and compiles WireIMXRT1176.cpp).
import_arduino_library(Wire ${CMAKE_CURRENT_LIST_DIR}/../..)

teensy_add_executable(wire_slave_test wire_slave_test.cpp)
teensy_target_link_libraries(wire_slave_test cores Wire)

target_link_libraries(wire_slave_test.elf stdc++)
