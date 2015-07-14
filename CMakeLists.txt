########################################################################
# Build Soapy SDR support module for blade RF
########################################################################
cmake_minimum_required(VERSION 2.8.7)
project(SoapyBladeRF CXX)

find_package(SoapySDR "0.2.1" CONFIG)
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

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${LIBBLADERF_INCLUDE_DIRS})

SOAPY_SDR_MODULE_UTIL(
    TARGET bladeRFSupport
    SOURCES
        bladeRF_Registation.cpp
        bladeRF_Settings.cpp
        bladeRF_Streaming.cpp
    LIBRARIES
        ${LIBBLADERF_LIBRARIES}
)
