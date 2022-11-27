cmake_minimum_required(VERSION 3.11)
project("rtl_asio")

include(FetchContent)

FetchContent_Declare(
  asio
  GIT_REPOSITORY https://github.com/chriskohlhoff/asio
  GIT_PROGRESS TRUE
  GIT_SHALLOW TRUE
  GIT_TAG origin/master)

if(NOT asio_POPULATED)
  FetchContent_Populate(asio)
  set(ASIO_STANDALONE
      ON
      CACHE BOOL "" FORCE)
  set(ASIO_HEADER_ONLY
      ON
      CACHE BOOL "" FORCE)
endif()

FetchContent_Declare(
  rtl_sdr
  GIT_REPOSITORY https://gitea.osmocom.org/sdr/rtl-sdr/
  GIT_PROGRESS TRUE
  GIT_SHALLOW TRUE
  GIT_TAG origin/master)

if(NOT rtl_sdr_POPULATED)
  FetchContent_Populate(rtl_sdr)
endif()

# select the release build type by default to get optimization flags
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
  message(STATUS "Build type not specified: defaulting to release.")
endif(NOT CMAKE_BUILD_TYPE)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ##############################################################################
# Compiler specific setup
# ##############################################################################
if(CMAKE_COMPILER_IS_GNUCC AND NOT WIN32)
  add_definitions(-Wall)
  add_definitions(-Wextra)
  add_definitions(-Wno-unused-parameter)
  add_definitions(-Wno-unused)
  add_definitions(-Wsign-compare)
  # http://gcc.gnu.org/wiki/Visibility
  add_definitions(-fvisibility=hidden)
elseif(MSVC14 OR MSVC14)
  # pthread-w32 issue, timespec is now part of time.h
  add_definitions(-D_TIMESPEC_DEFINED)
endif()

find_package(Threads)
find_package(PkgConfig)

if(PKG_CONFIG_FOUND)
  pkg_check_modules(LIBUSB libusb-1.0 IMPORTED_TARGET)
else()
  set(LIBUSB_LIBRARIES
      ""
      CACHE STRING "manual libusb path")
  set(LIBUSB_INCLUDE_DIRS
      ""
      CACHE STRING "manual libusb includepath")
endif()

if(MSVC)
  set(THREADS_PTHREADS_LIBRARY
      ""
      CACHE STRING "manual pthread-win32 path")
  set(THREADS_PTHREADS_INCLUDE_DIR
      ""
      CACHE STRING "manual pthread-win32 includepath")
else()
  set(THREADS_PTHREADS_LIBRARY
      ""
      CACHE INTERNAL "manual pthread-win32 path")
  set(THREADS_PTHREADS_INCLUDE_DIR
      ""
      CACHE INTERNAL "manual pthread-win32 includepath")
endif()

if(PKG_CONFIG_FOUND AND NOT LIBUSB_FOUND)
  message(FATAL_ERROR "LibUSB 1.0 required to compile rtl-sdr")
endif()
if(NOT THREADS_FOUND)
  message(FATAL_ERROR "pthreads(-win32) required to compile rtl-sdr")
endif()

option(DETACH_KERNEL_DRIVER "Detach kernel driver if loaded" OFF)
if(DETACH_KERNEL_DRIVER)
  message(STATUS "Building with kernel driver detaching enabled")
  add_definitions(-DDETACH_KERNEL_DRIVER=1)
else(DETACH_KERNEL_DRIVER)
  message(
    STATUS
      "Building with kernel driver detaching disabled, use -DDETACH_KERNEL_DRIVER=ON to enable"
  )
endif(DETACH_KERNEL_DRIVER)

option(ENABLE_ZEROCOPY "Enable usbfs zero-copy support" OFF)
if(ENABLE_ZEROCOPY)
  message(STATUS "Building with usbfs zero-copy support enabled")
  add_definitions(-DENABLE_ZEROCOPY=1)
else(ENABLE_ZEROCOPY)
  message(
    STATUS
      "Building with usbfs zero-copy support disabled, use -DENABLE_ZEROCOPY=ON to enable"
  )
endif(ENABLE_ZEROCOPY)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

add_executable(
  rtl_asio
  rtl_asio.cpp
  ${rtl_sdr_SOURCE_DIR}/src/librtlsdr.c
  ${rtl_sdr_SOURCE_DIR}/src/tuner_e4k.c
  ${rtl_sdr_SOURCE_DIR}/src/tuner_fc0012.c
  ${rtl_sdr_SOURCE_DIR}/src/tuner_fc0013.c
  ${rtl_sdr_SOURCE_DIR}/src/tuner_fc2580.c
  ${rtl_sdr_SOURCE_DIR}/src/tuner_r82xx.c)

target_link_libraries(rtl_asio ${LIBUSB_LIBRARIES} ${THREADS_PTHREADS_LIBRARY})
include_directories(
  ${asio_SOURCE_DIR}/asio/include ${rtl_sdr_SOURCE_DIR}/include
  ${LIBUSB_INCLUDE_DIRS} ${THREADS_PTHREADS_INCLUDE_DIR})
