cmake_minimum_required(VERSION 3.20)

project(
    nurbspath
    VERSION 0.1.2
    DESCRIPTION "Header-only C++20 geometry and numerical queries for 2D and 3D NURBS paths"
    LANGUAGES CXX
)

# Refresh public Git metadata at configure time. A repository without a first
# commit is a valid development state and is reported as "unborn" rather than
# causing configuration to fail.
set(NURBSPATH_GIT_COMMIT "unborn")
set(NURBSPATH_GIT_COMMIT_SHORT "unborn")
set(NURBSPATH_GIT_DESCRIBE "unborn")
set(NURBSPATH_GIT_DIRTY 0)
set(NURBSPATH_GIT_COMMIT_AVAILABLE 0)
find_package(Git QUIET)
if(Git_FOUND)
    execute_process(
        COMMAND "${GIT_EXECUTABLE}" -C "${CMAKE_CURRENT_SOURCE_DIR}" rev-parse HEAD
        RESULT_VARIABLE NURBSPATH_GIT_HEAD_RESULT
        OUTPUT_VARIABLE NURBSPATH_GIT_HEAD_OUTPUT
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
    if(NURBSPATH_GIT_HEAD_RESULT EQUAL 0)
        set(NURBSPATH_GIT_COMMIT "${NURBSPATH_GIT_HEAD_OUTPUT}")
        set(NURBSPATH_GIT_COMMIT_AVAILABLE 1)
        execute_process(
            COMMAND "${GIT_EXECUTABLE}" -C "${CMAKE_CURRENT_SOURCE_DIR}"
                    rev-parse --short=12 HEAD
            OUTPUT_VARIABLE NURBSPATH_GIT_COMMIT_SHORT
            OUTPUT_STRIP_TRAILING_WHITESPACE
            ERROR_QUIET
        )
        execute_process(
            COMMAND "${GIT_EXECUTABLE}" -C "${CMAKE_CURRENT_SOURCE_DIR}"
                    describe --always --tags
            OUTPUT_VARIABLE NURBSPATH_GIT_DESCRIBE
            OUTPUT_STRIP_TRAILING_WHITESPACE
            ERROR_QUIET
        )
    endif()
    execute_process(
        COMMAND "${GIT_EXECUTABLE}" -C "${CMAKE_CURRENT_SOURCE_DIR}"
                status --porcelain
        OUTPUT_VARIABLE NURBSPATH_GIT_STATUS
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
    if(NOT NURBSPATH_GIT_STATUS STREQUAL "")
        set(NURBSPATH_GIT_DIRTY 1)
        string(APPEND NURBSPATH_GIT_DESCRIBE "-dirty")
    endif()
endif()

math(
    EXPR NURBSPATH_VERSION_NUMBER
    "${PROJECT_VERSION_MAJOR} * 10000 + ${PROJECT_VERSION_MINOR} * 100 + ${PROJECT_VERSION_PATCH}"
)
set(NURBSPATH_GIT_VERSION "${PROJECT_VERSION}+${NURBSPATH_GIT_DESCRIBE}")
set(NURBSPATH_GENERATED_INCLUDE_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated/include")
file(MAKE_DIRECTORY "${NURBSPATH_GENERATED_INCLUDE_DIR}/nurbspath")
configure_file(
    cmake/config.hpp.in
    "${NURBSPATH_GENERATED_INCLUDE_DIR}/nurbspath/config.hpp"
    @ONLY
)

add_library(nurbspath INTERFACE)
add_library(nurbspath::nurbspath ALIAS nurbspath)

target_compile_features(nurbspath INTERFACE cxx_std_20)
target_include_directories(
    nurbspath
    INTERFACE
        $<BUILD_INTERFACE:${NURBSPATH_GENERATED_INCLUDE_DIR}>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

option(NURBSPATH_BUILD_EXAMPLES "Build the nurbspath example programs" ON)
option(NURBSPATH_BUILD_TESTS "Build the numbered nurbspath test programs" ON)
option(NURBSPATH_BUILD_DOCUMENTATION "Build Doxygen API documentation" OFF)

if(NURBSPATH_BUILD_EXAMPLES)
    add_executable(nurbspath_example examples/basic_usage.cpp)
    target_link_libraries(nurbspath_example PRIVATE nurbspath::nurbspath)

    add_executable(nurbspath_2d_example examples/basic_usage_2d.cpp)
    target_link_libraries(nurbspath_2d_example PRIVATE nurbspath::nurbspath)

    add_executable(nurbspath_svg_example examples/svg_usage.cpp)
    target_link_libraries(nurbspath_svg_example PRIVATE nurbspath::nurbspath)
endif()

if(NURBSPATH_BUILD_TESTS)
    include(CTest)
    set(
        NURBSPATH_TEST_SOURCES
        tests/01_test_vector_point.cpp
        tests/02_test_surfaces.cpp
        tests/03_test_spline_evaluation.cpp
        tests/04_test_spline_interpolation.cpp
        tests/05_test_ray_intersections.cpp
        tests/06_test_spline_intersections.cpp
        tests/07_test_distances.cpp
        tests/08_test_svg_graphics.cpp
        tests/09_test_float_and_detail.cpp
        tests/10_test_2d_entities.cpp
        tests/11_test_2d_spline.cpp
        tests/12_test_2d_queries.cpp
        tests/13_test_2d_projection_graphics.cpp
        tests/14_test_creators.cpp
        tests/15_test_svg_canvas2.cpp
        tests/16_test_config.cpp
        tests/17_test_manual.cpp
    )
    set(NURBSPATH_TEST_TARGETS)
    foreach(test_source IN LISTS NURBSPATH_TEST_SOURCES)
        get_filename_component(test_name "${test_source}" NAME_WE)
        add_executable("${test_name}" "${test_source}")
        target_link_libraries("${test_name}" PRIVATE nurbspath::nurbspath)
        if(MSVC)
            target_compile_options("${test_name}" PRIVATE /W4 /permissive-)
        else()
            target_compile_options("${test_name}" PRIVATE -Wall -Wextra -Wpedantic)
        endif()
        add_test(NAME "${test_name}" COMMAND "${test_name}")
        set_tests_properties("${test_name}" PROPERTIES LABELS "nurbspath;unit")
        list(APPEND NURBSPATH_TEST_TARGETS "${test_name}")
    endforeach()
    add_custom_target(nurbspath_tests DEPENDS ${NURBSPATH_TEST_TARGETS})
endif()

if(NURBSPATH_BUILD_DOCUMENTATION)
    find_package(Doxygen REQUIRED)
    set(DOXYGEN_EXTRACT_ALL NO)
    set(DOXYGEN_EXTRACT_PRIVATE NO)
    set(DOXYGEN_EXTRACT_STATIC YES)
    set(DOXYGEN_GENERATE_TREEVIEW YES)
    set(DOXYGEN_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/docs")
    set(DOXYGEN_USE_MDFILE_AS_MAINPAGE "${CMAKE_CURRENT_SOURCE_DIR}/README.md")
    set(DOXYGEN_WARN_IF_UNDOCUMENTED YES)
    set(DOXYGEN_WARN_IF_DOC_ERROR YES)
    set(DOXYGEN_WARN_IF_INCOMPLETE_DOC YES)
    set(DOXYGEN_WARN_NO_PARAMDOC YES)
    set(DOXYGEN_WARN_AS_ERROR YES)
    doxygen_add_docs(
        nurbspath_docs
        "${CMAKE_CURRENT_SOURCE_DIR}/include/nurbspath"
        "${CMAKE_CURRENT_SOURCE_DIR}/README.md"
        "${CMAKE_CURRENT_SOURCE_DIR}/INSTALL.md"
        COMMENT "Generate nurbspath API documentation"
    )
    add_custom_target(docs DEPENDS nurbspath_docs)
endif()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

install(TARGETS nurbspath EXPORT nurbspath_targets)
install(
    DIRECTORY include/nurbspath
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    PATTERN ".DS_Store" EXCLUDE
    PATTERN "config.hpp" EXCLUDE
)
install(
    FILES "${NURBSPATH_GENERATED_INCLUDE_DIR}/nurbspath/config.hpp"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/nurbspath"
)
install(
    EXPORT nurbspath_targets
    FILE nurbspath-targets.cmake
    NAMESPACE nurbspath::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/nurbspath
)

configure_package_config_file(
    cmake/nurbspath-config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/nurbspath-config.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/nurbspath
)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/nurbspath-config-version.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)
install(
    FILES
        ${CMAKE_CURRENT_BINARY_DIR}/nurbspath-config.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/nurbspath-config-version.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/nurbspath
)
