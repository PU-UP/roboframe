cmake_minimum_required(VERSION 3.16)
add_library(odom_session_runtime STATIC src/session_integration.cpp)
target_compile_features(odom_session_runtime PUBLIC cxx_std_14)
target_include_directories(odom_session_runtime PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(odom_session_runtime PUBLIC Odom::component)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(odom_session_runtime PRIVATE -Wall -Wextra -Wpedantic -pedantic-errors)
endif()

if(NOT TARGET roboframe_stdio_transport)
    set(interface_dir "${CMAKE_SOURCE_DIR}/interface")
    if(NOT EXISTS "${interface_dir}/CMakeLists.txt")
        message(FATAL_ERROR "odom_session needs interface/ for StdioChannel")
    endif()
    add_subdirectory("${interface_dir}" "${CMAKE_CURRENT_BINARY_DIR}/interface")
endif()

add_executable(odom_session main.cpp)
target_link_libraries(odom_session PRIVATE odom_session_runtime roboframe_stdio_transport)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(odom_session PRIVATE -Wall -Wextra -Wpedantic -pedantic-errors)
endif()

if(BUILD_TESTING)
    add_executable(odom_session_integration_test tests/session_integration_test.cpp)
    target_link_libraries(odom_session_integration_test PRIVATE odom_session_runtime)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(odom_session_integration_test PRIVATE -Wall -Wextra -Wpedantic -pedantic-errors)
    endif()
    add_test(NAME odom_session_integration_test COMMAND odom_session_integration_test)
    set_tests_properties(odom_session_integration_test PROPERTIES TIMEOUT 10)

    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    set(session_log "${CMAKE_SOURCE_DIR}/data/intel-lab/intel.log.gz")
    if(NOT EXISTS "${session_log}")
        message(FATAL_ERROR "odom_session replay requires data/intel-lab/intel.log.gz")
    endif()
    set(replay_check "${CMAKE_CURRENT_SOURCE_DIR}/tests/replay_test.py")
    add_test(NAME odom_session_replay COMMAND "${Python3_EXECUTABLE}" "${replay_check}"
        $<TARGET_FILE:odom_session> "${session_log}")
    add_test(NAME odom_session_replay_corrupt_gzip COMMAND "${Python3_EXECUTABLE}" "${replay_check}"
        $<TARGET_FILE:odom_session> "${session_log}" --check-corrupt)
    add_test(NAME odom_session_stop_restart COMMAND "${Python3_EXECUTABLE}" "${replay_check}"
        $<TARGET_FILE:odom_session> "${session_log}" --check-session)
    set_tests_properties(odom_session_replay odom_session_replay_corrupt_gzip odom_session_stop_restart
        PROPERTIES TIMEOUT 60)
endif()
