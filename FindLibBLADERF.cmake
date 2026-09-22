# - Try to find LibBLADERF
# Once done this will define
#
#  LibBLADERF_FOUND - System has libbladeRF
#  LibBLADERF_INCLUDE_DIRS - The libbladeRF include directories
#  LibBLADERF_LIBRARIES - The libraries needed to use libbladeRF
#  LibBLADERF_DEFINITIONS - Compiler switches required for using libbladeRF
#  LibBLADERF_VERSION - The libbladeRF version
#

find_package(PkgConfig)
pkg_check_modules(LibBLADERF_PKG libbladeRF)
set(LibBLADERF_DEFINITIONS ${PC_LibBLADERF_CFLAGS_OTHER})

find_path(LibBLADERF_INCLUDE_DIR NAMES libbladeRF.h
    HINTS ${LibBLADERF_PKG_INCLUDE_DIRS}
    PATHS
    /usr/include
    /usr/local/include )

find_library(LibBLADERF_LIBRARY NAMES bladeRF
    HINTS ${LibBLADERF_PKG_LIBRARY_DIRS}
    PATHS
    /usr/lib
    /usr/local/lib )

set(LibBLADERF_VERSION ${PC_LibBLADERF_VERSION})

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set LibBLADERF_FOUND to TRUE
# if all listed variables are TRUE
# Note that `FOUND_VAR LibBLADERF_FOUND` is needed for cmake 3.2 and older.
find_package_handle_standard_args(LibBLADERF
                                  FOUND_VAR LibBLADERF_FOUND
                                  REQUIRED_VARS LibBLADERF_LIBRARY LibBLADERF_INCLUDE_DIR
                                  VERSION_VAR LibBLADERF_VERSION)

mark_as_advanced(LibBLADERF_LIBRARY LibBLADERF_INCLUDE_DIR LibBLADERF_VERSION)

set(LibBLADERF_LIBRARIES ${LibBLADERF_LIBRARY} )
set(LibBLADERF_INCLUDE_DIRS ${LibBLADERF_INCLUDE_DIR} )
