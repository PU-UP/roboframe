cmake_minimum_required(VERSION 3.16)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/VERSION")
include("${CMAKE_CURRENT_SOURCE_DIR}/VERSION")
project(RoboframeSDK VERSION ${ROBOFRAME_SDK_VERSION} LANGUAGES CXX)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
find_package(Threads REQUIRED)
include(cmake/RoboframeComponentMetadata.cmake)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    include(CTest)
endif()

configure_file(cmake/sdk_version.hpp.in generated/roboframe/sdk_version.hpp @ONLY)
add_library(roboframe_sdk INTERFACE)
add_library(Roboframe::sdk ALIAS roboframe_sdk)
set_target_properties(roboframe_sdk PROPERTIES EXPORT_NAME sdk
    ROBOFRAME_SDK_VERSION "${PROJECT_VERSION}"
    EXPORT_PROPERTIES ROBOFRAME_SDK_VERSION)
target_compile_features(roboframe_sdk INTERFACE cxx_std_14)
target_link_libraries(roboframe_sdk INTERFACE Threads::Threads)
target_include_directories(roboframe_sdk INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/generated>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

if(BUILD_TESTING)
    add_executable(execution_test tests/execution_test.cpp)
    target_link_libraries(execution_test PRIVATE Roboframe::sdk)
    set_target_properties(execution_test PROPERTIES CXX_STANDARD 14 CXX_STANDARD_REQUIRED ON CXX_EXTENSIONS OFF)
    add_test(NAME execution_test COMMAND execution_test)
    set_tests_properties(execution_test PROPERTIES TIMEOUT 10)
    add_executable(sdk_check tests/sdk_check.cpp)
    target_link_libraries(sdk_check PRIVATE Roboframe::sdk)
    set_target_properties(sdk_check PROPERTIES CXX_STANDARD 14 CXX_STANDARD_REQUIRED ON CXX_EXTENSIONS OFF)
    add_test(NAME sdk_check COMMAND sdk_check)
    add_executable(data_contract_test tests/data_contract_test.cpp)
    target_link_libraries(data_contract_test PRIVATE Roboframe::sdk)
    set_target_properties(data_contract_test PROPERTIES CXX_STANDARD 14 CXX_STANDARD_REQUIRED ON CXX_EXTENSIONS OFF)
    add_test(NAME data_contract_test COMMAND data_contract_test)
    add_executable(grid_sharing_test tests/grid_sharing_test.cpp)
    target_link_libraries(grid_sharing_test PRIVATE Roboframe::sdk)
    set_target_properties(grid_sharing_test PROPERTIES CXX_STANDARD 14 CXX_STANDARD_REQUIRED ON CXX_EXTENSIONS OFF)
    add_test(NAME grid_sharing_test COMMAND grid_sharing_test)
endif()

set(package_dir ${CMAKE_INSTALL_LIBDIR}/cmake/RoboframeSDK)
configure_package_config_file(cmake/RoboframeSDKConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/RoboframeSDKConfig.cmake INSTALL_DESTINATION ${package_dir})
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/RoboframeSDKConfigVersion.cmake
    COMPATIBILITY SameMajorVersion)
install(TARGETS roboframe_sdk EXPORT RoboframeSDKTargets)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/generated/roboframe/sdk_version.hpp
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/roboframe)
install(EXPORT RoboframeSDKTargets NAMESPACE Roboframe:: DESTINATION ${package_dir})
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/RoboframeSDKConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/RoboframeSDKConfigVersion.cmake DESTINATION ${package_dir})

install(FILES VERSION DESTINATION ${CMAKE_INSTALL_DATADIR}/roboframe/sdk)
install(FILES cmake/RoboframeComponentMetadata.cmake cmake/component_version.hpp.in
    DESTINATION ${package_dir})
