cmake_minimum_required(VERSION 3.14)
project(ShadingAwareMVS
    LANGUAGES CXX
    VERSION 0.0.1
    DESCRIPTION "A fork of flanggut/smvs"
    HOMEPAGE_URL "https://github.com/flanggut/smvs"
)
set(ignoreMe "${CMAKE_C_COMPILER}")

option(BUILD_SHARED_LIBS "This will cause all libraries to be built shared" ON)
option(BUILD_APPS "If the SMVS applications should be built" ON)
option(BUILD_TOOLS "If the SMVS tools should be built" ON)
option(BUILD_TESTS "If the tests should be built" ON)
option(ENABLE_IPO "Attempt to enable interprocedural optimization" ON)

# Set a default build type if none was specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to 'RelWithDebInfo' as none was specified.")
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Choose the type of build." FORCE)
    # Set the possible values of build type for cmake-gui, ccmake
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "RelWithDebInfo")
endif()

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

find_package(mve REQUIRED)

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag(-msse4.1 HAS_SSE41)
if(NOT HAS_SSE41)
    message(FATAL_ERROR "Compiler does not support sse4.1 intrinsics!")
endif()
check_cxx_compiler_flag(-mpopcnt HAS_POPCNT)
if(NOT HAS_POPCNT)
    message(FATAL_ERROR "Compiler does not support popcount intrinsics!")
endif()

add_library(smvs_settings INTERFACE)
add_library(smvs::settings ALIAS smvs_settings)
target_compile_features(smvs_settings INTERFACE cxx_std_11)
target_compile_options(smvs_settings INTERFACE
    $<$<CXX_COMPILER_ID:GNU>:-Wall -Wextra -Wundef -pedantic -Wno-sign-compare>
    $<$<CXX_COMPILER_ID:Clang>:-Weverything -pedantic>
    $<$<CXX_COMPILER_ID:MSVC>:/W3 -DNOMINMAX -D_USE_MATH_DEFINES -D_CRT_SECURE_NO_WARNINGS>
    $<$<AND:$<NOT:$<CONFIG:DEBUG>>,$<CXX_COMPILER_ID:GNU>>:-funsafe-math-optimizations>
    $<$<AND:$<NOT:$<CONFIG:DEBUG>>,$<CXX_COMPILER_ID:GNU>>:-fno-math-errno>
)
set_target_properties(smvs_settings PROPERTIES EXPORT_NAME settings)
install(TARGETS smvs_settings EXPORT smvs-targets)

if (ENABLE_IPO AND NOT CMAKE_BUILD_TYPE MATCHES Debug)
    include(CheckIPOSupported)
    check_ipo_supported(RESULT CMAKE_CXX_LTO_SUPPORTED OUTPUT LTO_ERROR_DETAILS LANGUAGES CXX)
    if (NOT CMAKE_CXX_LTO_SUPPORTED)
        message(WARNING "Interprocedural Optimization was requested but not supported with details:\n${LTO_ERROR_DETAILS}")
        set(ENABLE_IPO OFF)
    endif()
endif()

include(GNUInstallDirs)

add_subdirectory(lib)

if (BUILD_APPS)
    add_subdirectory(app)
endif()

if (BUILD_TOOLS)
    add_subdirectory(tools)
endif()

if (BUILD_TESTS)
    find_package(GTest REQUIRED)
    add_subdirectory(tests)
endif()

install(EXPORT smvs-targets
    FILE smvs-targets.cmake
    NAMESPACE smvs::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/smvs/
    COMPONENT SMVS_Development
)

install(FILES LICENSE README.md
    DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/smvs/
    COMPONENT SMVS_Runtime NAMELINK_COMPONENT SMVS_Development
    )

if (CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    add_subdirectory(dist)
endif()
