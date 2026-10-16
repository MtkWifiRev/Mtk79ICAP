# SPDX-License-Identifier: MIT
cmake_minimum_required(VERSION 3.13)
project(SoapyMT7921 CXX C)

find_package(SoapySDR "0.8" REQUIRED)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_subdirectory(../libmt7921iq libmt7921iq)

SOAPY_SDR_MODULE_UTIL(
  TARGET mt7921Support
  SOURCES MT7921Device.cpp
  LIBRARIES mt7921iq
)
