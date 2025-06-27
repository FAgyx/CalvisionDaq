cmake_minimum_required(VERSION 3.18)

project(CalvisionDaq VERSION 1.0.0 LANGUAGES CXX)


set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-g -fPIC -Wall -Wextra -Werror -Wno-reorder -Wno-maybe-uninitialized")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CalvisionDaq_CXX_STD "cxx_std_17")

set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)


include("${CMAKE_PREFIX_PATH}/lib/cmake/CppUtils/CppUtilsPreProcTargets.cmake")
include("${CMAKE_PREFIX_PATH}/lib/cmake/CppUtils/CppUtilsCUtilsTargets.cmake")
include("${CMAKE_PREFIX_PATH}/lib/cmake/CppUtils/CppUtilsIOTargets.cmake")

find_package(BLAS REQUIRED)
find_path(CBLAS_INCLUDE_DIR cblas.h
  PATHS
    /usr/include
    /usr/include/openblas
    /usr/include/cblas
)

if(NOT CBLAS_INCLUDE_DIR)
  message(FATAL_ERROR "Could not find cblas.h")
else()
  message(STATUS " Found cblas.h at: ${CBLAS_INCLUDE_DIR}")
endif()

#  Make it GLOBAL so all subdirs (like exec/) see it
set(CBLAS_INCLUDE_DIR "${CBLAS_INCLUDE_DIR}" CACHE INTERNAL "Path to cblas.h for CalvisionDaq")


set(CalvisionDaq_BUILD_INCLUDE_DIR "${CalvisionDaq_SOURCE_DIR}/src")
set(CalvisionDaq_INSTALL_BIN_DIR "bin")
set(CalvisionDaq_INSTALL_LIB_DIR "lib")
set(CalvisionDaq_INSTALL_INCLUDE_DIR "include/CalvisionDaq")

include(cmake/MakeCalvisionDaqLibrary.cmake)

find_package(ROOT REQUIRED COMPONENTS
    Core
    Tree
    RIO
    Hist
    Graf
    Gpad
    Graf3d
    Imt
    Gui
)
include(${ROOT_USE_FILE})


add_subdirectory(src/CalvisionDaq)
