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

set(CMAKE_CXX_STANDARD 11)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${LIBBLADERF_INCLUDE_DIRS})

if(CMAKE_COMPILER_IS_GNUCXX)

    #disable warnings for unused parameters
    add_definitions(-Wno-unused-parameter)

endif(CMAKE_COMPILER_IS_GNUCXX)

SOAPY_SDR_MODULE_UTIL(
    TARGET bladeRFSupport
    SOURCES
        bladeRF_Registration.cpp
        bladeRF_Settings.cpp
        bladeRF_Streaming.cpp
    LIBRARIES
        ${LIBBLADERF_LIBRARIES}
)

########################################################################
# uninstall target
########################################################################
add_custom_target(uninstall
    "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)

