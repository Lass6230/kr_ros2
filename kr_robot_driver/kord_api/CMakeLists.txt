cmake_minimum_required(VERSION 3.5)

project(kord_remote_control)

set(CMAKE_VERBOSE_MAKEFILE OFF)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -pthread")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

# Set the default gcov executable (can be overridden by the user)
set(GCOV_EXECUTABLE "/usr/bin/gcov" CACHE STRING "Path to the gcov executable")

# Set the source and include directories for filtering coverage (relative to the project root)
set(COVERAGE_FILTER_SRC "${CMAKE_SOURCE_DIR}/src" CACHE STRING "Path to the source directory for coverage filtering")
set(COVERAGE_FILTER_INCLUDE "${CMAKE_SOURCE_DIR}/include" CACHE STRING "Path to the include directory for coverage filtering")

# Output directory for coverage reports
set(COVERAGE_FOLDER coverage CACHE STRING "Target folder for coverage data")
set(COVERAGE_OUTPUT_DIR "${CMAKE_BINARY_DIR}/${COVERAGE_FOLDER}" CACHE STRING "Directory for coverage report output")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules")

option(WITH_TESTS "Compile with tests" OFF)
option(WITH_EXAMPLES "Compile the project including examples" OFF)
option(WITH_COVERAGE "Enable coverage reporting" OFF)

# Enable coverage flags if CODE_COVERAGE is ON
if (WITH_COVERAGE)
    message(STATUS "Building with code coverage enabled")
    message(STATUS "Coverage output directory: ${COVERAGE_OUTPUT_DIR}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-arcs -ftest-coverage --coverage -O0 -g")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")

    add_custom_target(coverage
            COMMAND mkdir ${COVERAGE_OUTPUT_DIR} && gcovr -v -r ${CMAKE_SOURCE_DIR}
            --gcov-executable ${GCOV_EXECUTABLE}
            --object-directory ${CMAKE_BINARY_DIR}/CMakeFiles
            --filter="${COVERAGE_FILTER_SRC}"
            --filter="${COVERAGE_FILTER_INCLUDE}"
            --html-details ${COVERAGE_OUTPUT_DIR}/coverage.html
            WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
            COMMENT "Generating code coverage report with gcovr"
    )
endif ()

execute_process(
        COMMAND git log -1 --format=%H
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        OUTPUT_VARIABLE GIT_SHA1
        OUTPUT_STRIP_TRAILING_WHITESPACE
)

execute_process(
        COMMAND git describe --tags --long
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        OUTPUT_VARIABLE GIT_VERSION
        OUTPUT_STRIP_TRAILING_WHITESPACE
)

configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/include/kord/version.h.in
        ${CMAKE_CURRENT_SOURCE_DIR}/include/kord/version.h
)

find_package(Threads REQUIRED)
if (WITH_EXAMPLES)
    find_package(Eigen3 REQUIRED)
    if (TARGET Eigen3::Eigen)
        message("Eigen includes: ${EIGEN3_INCLUDE_DIR}")
    else ()
        error("Eigen3 not found. Install the Eigen3 library to allow compilation with examples.")
    endif ()
endif ()

include_directories(
        external/kord-protocol/include
        external
        external/asio
)

add_subdirectory(external/kord-protocol)

# Build tests if set on
if (WITH_TESTS)
    add_subdirectory(kord_test)
endif ()

add_library(kord SHARED
        src/api/kord.cpp
        src/api/connection_interface.cpp
        src/api/kord_control_interface.cpp
        src/api/kord_receive_interface.cpp
        src/api/kord_io_request.cpp
        src/utils/timex.cpp
        src/utils/utils.cpp
)

target_include_directories(
        kord
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

if (WITH_EXAMPLES)
    add_library(polynomials SHARED
            src/utils/polynomials/from_constraints.cpp
            src/utils/polynomials/polynomials.cpp
            src/utils/polynomials/spline_computation.cpp
            src/utils/polynomials/spline_equations.cpp
    )

    target_include_directories(
            polynomials
            PUBLIC
            ${EIGEN3_INCLUDE_DIR}
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
    )

    add_executable(ex_move_joints examples/ex_move_joints.cpp)
    add_executable(ex_move_linear examples/ex_move_linear.cpp)
    add_executable(ex_move_velocity examples/ex_move_velocity.cpp)
    add_executable(ex_read_state examples/ex_read_state.cpp)
    add_executable(ex_brake_control examples/ex_brake_control.cpp)
    add_executable(ex_transfer_logs examples/ex_transfer_logs.cpp)
    add_executable(ex_retrieve_errors examples/ex_retrieve_errors.cpp)
    add_executable(ex_move_direct examples/ex_move_direct.cpp)
    add_executable(ex_move_joints_discrete examples/ex_move_joints_discrete.cpp)
    add_executable(ex_move_linear_discrete examples/ex_move_linear_discrete.cpp)
    add_executable(ex_set_oport examples/ex_set_oport.cpp)
    add_executable(ex_self_motion examples/ex_self_motion.cpp)
    add_executable(ex_read_temperature examples/ex_read_temperature.cpp)
    add_executable(ex_set_frame examples/ex_set_frame.cpp)
    add_executable(ex_set_load examples/ex_set_load.cpp)
    add_executable(ex_get_tcp_and_cog examples/ex_get_tcp_and_cog.cpp)
    add_executable(ex_clean_alarm examples/ex_clean_alarm.cpp)
    add_executable(ex_transfer_json examples/ex_transfer_json.cpp)
    add_executable(ex_transfer_calibration_data examples/ex_transfer_calibration_data.cpp)
    add_executable(ex_transfer_more_files examples/ex_transfer_more_files.cpp)
    add_executable(ex_init examples/ex_init.cpp)
    add_executable(ex_fetch_data examples/ex_fetch_data.cpp)
    add_executable(ex_skip_fetch_data examples/ex_skip_fetch_data.cpp)
    add_executable(ex_init_and_fetch examples/ex_init_and_fetch.cpp)
    add_executable(ex_move_direct_interactive examples/ex_move_direct_interactive.cpp)
    add_executable(ex_set_safe_output examples/ex_set_safe_output.cpp)

    target_compile_options(ex_transfer_more_files PRIVATE -pthread)

    target_link_libraries(ex_set_safe_output kord pthread)
    target_link_libraries(ex_move_joints kord pthread)
    target_link_libraries(ex_move_linear kord pthread)
    target_link_libraries(ex_move_direct kord pthread)
    target_link_libraries(ex_self_motion kord pthread)
    target_link_libraries(ex_move_joints_discrete kord pthread)
    target_link_libraries(ex_move_linear_discrete kord pthread)
    target_link_libraries(ex_move_velocity kord pthread)
    target_link_libraries(ex_read_state kord pthread)
    target_link_libraries(ex_brake_control kord pthread)
    target_link_libraries(ex_transfer_logs kord pthread)
    target_link_libraries(ex_retrieve_errors kord pthread)
    target_link_libraries(ex_transfer_json kord pthread)
    target_link_libraries(ex_transfer_calibration_data kord pthread)
    target_link_libraries(ex_transfer_more_files kord pthread)
    target_link_libraries(ex_init kord pthread)
    target_link_libraries(ex_fetch_data kord pthread)
    target_link_libraries(ex_skip_fetch_data kord pthread)
    target_link_libraries(ex_init_and_fetch kord pthread)
    target_link_libraries(ex_move_direct_interactive kord pthread polynomials Eigen3::Eigen ncurses ${SDL2_LIBRARIES})
    target_link_libraries(ex_set_oport kord pthread)
    target_link_libraries(ex_read_temperature kord pthread)
    target_link_libraries(ex_set_frame kord pthread)
    target_link_libraries(ex_set_load kord pthread)
    target_link_libraries(ex_get_tcp_and_cog kord pthread)
    target_link_libraries(ex_clean_alarm kord pthread)
endif ()

target_link_libraries(kord kord_protocol)
