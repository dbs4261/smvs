cmake_minimum_required(VERSION 3.14)
project(smvs LANGUAGES CXX)
set(ignoreMe "${CMAKE_C_COMPILER}")

set(SMVS_VERSION 20.11.14)

option(BUILD_SHARED_LIBS "This will cause all libraries to be built shared" ON)
option(BUILD_APPS "If the SMVS applications should be built" ON)
option(BUILD_TESTS "If the tests should be built" ON)
option(ENABLE_IPO "Enable Interprocedural Optimization, aka Link Time Optimization (LTO)" ON)
option(ENABLE_PIC "Enable Position Independent Code" ON)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

find_package(mve REQUIRED)

add_library(smvs_settings INTERFACE)
add_library(smvs::settings ALIAS smvs_settings)
target_compile_features(smvs_settings INTERFACE cxx_std_11)
target_compile_options(smvs_settings INTERFACE
    $<$<CXX_COMPILER_ID:GNU>:-Wall -Wextra -Wundef -pedantic -Wno-sign-compare>
    $<$<CXX_COMPILER_ID:Clang>:-Weverything -pedantic>
    $<$<CXX_COMPILER_ID:MSVC>:/W3 -DNOMINMAX -D_USE_MATH_DEFINES -D_CRT_SECURE_NO_WARNINGS>
    $<$<AND:$<NOT:$<CONFIG:DEBUG>>,$<CXX_COMPILER_ID:GNU>>:-march=native>
    $<$<AND:$<NOT:$<CONFIG:DEBUG>>,$<CXX_COMPILER_ID:GNU>>:-funsafe-math-optimizations>
    $<$<AND:$<NOT:$<CONFIG:DEBUG>>,$<CXX_COMPILER_ID:GNU>>:-fno-math-errno>
)
set_target_properties(smvs_settings PROPERTIES EXPORT_NAME settings)
install(TARGETS smvs_settings EXPORT smvs-targets)

if (ENABLE_PIC)
    include(CheckPIESupported)
    check_pie_supported(LANGUAGES CXX)
    set_property(TARGET smvs_settings PROPERTY POSITION_INDEPENDENT_CODE ${CMAKE_CXX_LINK_PIE_SUPPORTED})
endif()

if (ENABLE_IPO)
    include(CheckIPOSupported)
    check_ipo_supported(RESULT CMAKE_CXX_LTO_SUPPORTED LANGUAGES CXX)
    set_property(TARGET smvs_settings PROPERTY INTERPROCEDURAL_OPTIMIZATION ${CMAKE_CXX_LTO_SUPPORTED})
endif()

include(GNUInstallDirs)

add_subdirectory(lib)

if (BUILD_APPS)
    add_subdirectory(app)
    add_subdirectory(tools)
endif()

if (BUILD_TESTS)
    find_package(GTest REQUIRED)
    add_subdirectory(tests)
endif()
