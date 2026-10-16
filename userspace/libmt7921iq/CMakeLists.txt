# SPDX-License-Identifier: MIT
cmake_minimum_required(VERSION 3.13)
project(libmt7921iq C)

set(CMAKE_C_STANDARD 99)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra)

add_library(mt7921iq STATIC mt7921iq.c)
target_link_libraries(mt7921iq PUBLIC m)
set_target_properties(mt7921iq PROPERTIES POSITION_INDEPENDENT_CODE ON)
# iqstream_ioctl.h is shared with the driver (driver/mt7921/).
target_include_directories(mt7921iq PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
                           ${CMAKE_CURRENT_SOURCE_DIR}/../../driver/mt7921)

add_executable(mt7921iq_rx mt7921iq_rx.c)
target_link_libraries(mt7921iq_rx mt7921iq)

enable_testing()
add_executable(test_mt7921iq test_mt7921iq.c)
target_link_libraries(test_mt7921iq mt7921iq)
add_test(NAME decoder COMMAND test_mt7921iq)

install(TARGETS mt7921iq_rx DESTINATION bin)
