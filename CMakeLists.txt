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
