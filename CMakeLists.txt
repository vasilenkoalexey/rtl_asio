cmake_minimum_required(VERSION 3.8)
project("rtl_asio")

# select the release build type by default to get optimization flags
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
  message(STATUS "Build type not specified: defaulting to release.")
endif(NOT CMAKE_BUILD_TYPE)

set(RTL rtl-sdr)

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
  ${RTL}/src/librtlsdr.c
  ${RTL}/src/tuner_e4k.c
  ${RTL}/src/tuner_fc0012.c
  ${RTL}/src/tuner_fc0013.c
  ${RTL}/src/tuner_fc2580.c
  ${RTL}/src/tuner_r82xx.c)

target_link_libraries(rtl_asio ${LIBUSB_LIBRARIES} ${THREADS_PTHREADS_LIBRARY})
include_directories(
  ${PROJECT_SOURCE_DIR}/asio/asio/include ${PROJECT_SOURCE_DIR}/${RTL}/include
  ${LIBUSB_INCLUDE_DIRS} ${THREADS_PTHREADS_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}/${RTL}/src)
