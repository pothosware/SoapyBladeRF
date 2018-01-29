########################################################################
# Build Soapy SDR support module for blade RF
########################################################################
cmake_minimum_required(VERSION 2.8.7)
project(SoapyBladeRF CXX)

find_package(SoapySDR "0.4" NO_MODULE)
if (NOT SoapySDR_FOUND)
    message(FATAL_ERROR "Soapy SDR development files not found...")
endif ()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
find_package(LibbladeRF)

if (NOT LIBBLADERF_FOUND)
    message(FATAL_ERROR "Blade RF development files not found...")
endif ()
message(STATUS "LIBBLADERF_INCLUDE_DIRS - ${LIBBLADERF_INCLUDE_DIRS}")
message(STATUS "LIBBLADERF_LIBRARIES - ${LIBBLADERF_LIBRARIES}")

#version check for recent bladerf with gain mode API
message(STATUS "Checking for bladerf_set_gain_mode API...")
message(STATUS "  Reading ${LIBBLADERF_INCLUDE_DIRS}/libbladeRF.h...")
file(READ ${LIBBLADERF_INCLUDE_DIRS}/libbladeRF.h libbladeRF_h)
string(FIND "${libbladeRF_h}" "bladerf_gain_mode" has_bladerf_gain_mode)
if ("${has_bladerf_gain_mode}" STREQUAL "-1")
    message(STATUS "  libbladeRF no support for bladerf_set_gain_mode API")
else()
    add_definitions(-DHAS_BLADERF_GAIN_MODE)
    message(STATUS "  libbladeRF supports the bladerf_set_gain_mode API")
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${LIBBLADERF_INCLUDE_DIRS})

#enable c++11 features
if(CMAKE_COMPILER_IS_GNUCXX)

    #C++11 is a required language feature for this project
    include(CheckCXXCompilerFlag)
    CHECK_CXX_COMPILER_FLAG("-std=c++11" HAS_STD_CXX11)
    if(HAS_STD_CXX11)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    else(HAS_STD_CXX11)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    endif()

    #disable warnings for unused parameters
    add_definitions(-Wno-unused-parameter)

endif(CMAKE_COMPILER_IS_GNUCXX)

if (APPLE)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wc++11-extensions")
endif(APPLE)

SOAPY_SDR_MODULE_UTIL(
    TARGET bladeRFSupport
    SOURCES
        bladeRF_Registation.cpp
        bladeRF_Settings.cpp
        bladeRF_Streaming.cpp
    LIBRARIES
        ${LIBBLADERF_LIBRARIES}
)
