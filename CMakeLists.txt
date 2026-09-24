########################################################################
# Build Soapy SDR support module for blade RF
########################################################################
cmake_minimum_required(VERSION 2.8.12...3.10)
project(SoapyBladeRF CXX)

find_package(SoapySDR "0.4.0" NO_MODULE REQUIRED)

get_filename_component(_SoapySDR_ROOT "${SoapySDR_INCLUDE_DIRS}/.." ABSOLUTE)
message(STATUS "SoapySDR root directory: ${_SoapySDR_ROOT}")
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT AND NOT "${_SoapySDR_ROOT}" STREQUAL "/usr")
    set_property(CACHE CMAKE_INSTALL_PREFIX PROPERTY VALUE ${_SoapySDR_ROOT})
endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
find_package(LibBLADERF REQUIRED)

message(STATUS "LibBLADERF_INCLUDE_DIRS - ${LibBLADERF_INCLUDE_DIRS}")
message(STATUS "LibBLADERF_LIBRARIES - ${LibBLADERF_LIBRARIES}")

set(CMAKE_CXX_STANDARD 11)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${LibBLADERF_INCLUDE_DIRS})

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
        ${LibBLADERF_LIBRARIES}
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

