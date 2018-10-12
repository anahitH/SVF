cmake_minimum_required(VERSION 3.12)
project(svf VERSION 0.1.0 LANGUAGES CXX C)

find_package(LLVM REQUIRED CONFIG)
list(APPEND CMAKE_MODULE_PATH "${LLVM_CMAKE_DIR}")
include(CMakePackageConfigHelpers)
include(AddLLVM)
include(GNUInstallDirs)
set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/svf)

add_definitions(${LLVM_DEFINITIONS})
add_subdirectory(lib)

write_basic_package_version_file(
        ${CMAKE_CURRENT_BINARY_DIR}/svfConfigVersion.cmake
        VERSION ${PROJECT_VERSION}
        COMPATIBILITY SameMajorVersion
)
configure_package_config_file(
        ${CMAKE_CURRENT_LIST_DIR}/cmake/svfConfig.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/svfConfig.cmake
        INSTALL_DESTINATION ${INSTALL_CONFIGDIR}
)

install(FILES
        ${CMAKE_CURRENT_BINARY_DIR}/svfConfigVersion.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/svfConfig.cmake
        DESTINATION ${INSTALL_CONFIGDIR}
        )
export(PACKAGE svf)

#add_subdirectory(tools)
