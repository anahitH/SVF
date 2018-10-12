cmake_minimum_required(VERSION 3.12)
project(svf VERSION 0.1.0 LANGUAGES CXX)

find_package(LLVM REQUIRED CONFIG)
list(APPEND CMAKE_MODULE_PATH "${LLVM_CMAKE_DIR}")
include(CMakePackageConfigHelpers)
include(AddLLVM)
include(GNUInstallDirs)
set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/svf)

add_subdirectory(lib)
#add_subdirectory(tools)
