cmake_minimum_required(VERSION 3.25)

project(soft_body_simulation
    VERSION 0.1.0
    DESCRIPTION "Soft-body physics and parallel-computing benchmark"
    LANGUAGES CXX
)

include(CTest)
include(GNUInstallDirs)

option(SOFT_BODY_BUILD_BENCHMARK "Build the benchmark executable" ON)
option(SOFT_BODY_BUILD_TESTS "Build the test suite" ${BUILD_TESTING})
option(SOFT_BODY_ENABLE_WARNINGS "Enable project compiler warnings" ON)
option(SOFT_BODY_ENABLE_NATIVE_OPTIMIZATIONS
       "Enable host-specific optimizations for local benchmark runs" OFF)

add_library(soft_body_core STATIC
    src/soft_body_core.cpp
    src/xpbd_solver.cpp
)


# XPBD DLL za integraciju s rendererom
add_library(soft_body_xpbd SHARED
    src/xpbd_solver.cpp
    src/soft_body_api.cpp
)

target_compile_features(soft_body_xpbd PUBLIC cxx_std_20)

target_include_directories(soft_body_xpbd PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_compile_definitions(soft_body_xpbd PRIVATE
    SOFT_BODY_DLL_EXPORTS
)

set_target_properties(soft_body_xpbd PROPERTIES
    CXX_EXTENSIONS OFF
    OUTPUT_NAME soft_body_xpbd
)



# XPBD adapter kompatibilan s rendererom
add_library(soft_body_xpbd_adapter SHARED
    src/xpbd_adapter.cpp
    src/xpbd_solver.cpp
)

target_compile_features(
    soft_body_xpbd_adapter PUBLIC cxx_std_20
)

target_include_directories(soft_body_xpbd_adapter PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

set_target_properties(soft_body_xpbd_adapter PROPERTIES
    CXX_EXTENSIONS OFF
    OUTPUT_NAME soft_body_xpbd_adapter
)

if(WIN32)
    set_target_properties(soft_body_xpbd_adapter PROPERTIES
        WINDOWS_EXPORT_ALL_SYMBOLS ON
    )
endif()



add_library(soft_body::core ALIAS soft_body_core)

target_compile_features(soft_body_core PUBLIC cxx_std_20)

target_include_directories(soft_body_core
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

set_target_properties(soft_body_core PROPERTIES
    CXX_EXTENSIONS OFF
    OUTPUT_NAME soft_body_core
)

if(SOFT_BODY_ENABLE_WARNINGS)
    if(MSVC)
        target_compile_options(soft_body_core PRIVATE /W4 /permissive-)
    else()
        target_compile_options(soft_body_core PRIVATE
            -Wall
            -Wextra
            -Wpedantic
            -Wconversion
            -Wsign-conversion
        )
    endif()
endif()

if(SOFT_BODY_ENABLE_NATIVE_OPTIMIZATIONS)
    if(MSVC)
        target_compile_options(soft_body_core PRIVATE /arch:AVX2)
    elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(soft_body_core PRIVATE -march=native)
    endif()
endif()


if(SOFT_BODY_BUILD_BENCHMARK)
    add_executable(soft_body_benchmark
        src/benchmark_main.cpp
        src/xpbd_adapter.cpp
        src/xpbd_solver.cpp
    )

    target_compile_features(
        soft_body_benchmark PRIVATE cxx_std_20
    )

    target_include_directories(
        soft_body_benchmark PRIVATE
        ${PROJECT_SOURCE_DIR}/include
    )

    set_target_properties(
        soft_body_benchmark PROPERTIES
        CXX_EXTENSIONS OFF
        OUTPUT_NAME soft_body_benchmark
    )
endif()



if(SOFT_BODY_BUILD_TESTS)
    if(NOT BUILD_TESTING)
        message(FATAL_ERROR
            "SOFT_BODY_BUILD_TESTS requires BUILD_TESTING=ON"
        )
    endif()

    
add_executable(
    soft_soft_edge_collision_test
    tests/soft_soft_edge_collision_test.cpp
)

target_compile_features(
    soft_soft_edge_collision_test
    PRIVATE cxx_std_20
)

target_link_libraries(
    soft_soft_edge_collision_test
    PRIVATE soft_body_xpbd_adapter
)

add_test(
    NAME soft_soft_edge_collision_test
    COMMAND soft_soft_edge_collision_test
)


    # Postojeci testovi
    add_executable(soft_body_core_tests
        tests/soft_body_core_tests.cpp
    )

    target_compile_features(soft_body_core_tests PRIVATE cxx_std_20)

    target_link_libraries(soft_body_core_tests
        PRIVATE soft_body::core
    )

    set_target_properties(soft_body_core_tests PROPERTIES
        CXX_EXTENSIONS OFF
    )

    add_test(
        NAME soft_body_core_smoke
        COMMAND soft_body_core_tests
    )

    # Novi XPBD smoke test
    add_executable(xpbd_smoke_test
        tests/xpbd_smoke_test.cpp
    )

    target_compile_features(xpbd_smoke_test PRIVATE cxx_std_20)

    target_link_libraries(xpbd_smoke_test
        PRIVATE soft_body::core
    )

    add_test(
        NAME xpbd_smoke_test
        COMMAND xpbd_smoke_test
    )
endif()


include(CMakePackageConfigHelpers)

install(
    TARGETS soft_body_core
    EXPORT soft_body_core_targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(
    TARGETS soft_body_xpbd soft_body_xpbd_adapter
    EXPORT soft_body_core_targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(
    DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
    EXPORT soft_body_core_targets
    FILE soft_body_core_targets.cmake
    NAMESPACE soft_body::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/soft_body_simulation
)


# Integracijski test XPBD DLL-a
if(SOFT_BODY_BUILD_TESTS)
    add_executable(dll_smoke_test
        tests/dll_smoke_test.cpp
    )

    target_compile_features(dll_smoke_test PRIVATE cxx_std_20)

    target_link_libraries(dll_smoke_test PRIVATE
        soft_body_xpbd
    )

    add_test(
        NAME dll_smoke_test
        COMMAND dll_smoke_test
    )
endif()



if(SOFT_BODY_BUILD_TESTS)
    add_executable(xpbd_adapter_test
        tests/xpbd_adapter_test.cpp
    )

    target_compile_features(
        xpbd_adapter_test PRIVATE cxx_std_20
    )

    target_link_libraries(xpbd_adapter_test PRIVATE
        soft_body_xpbd_adapter
    )

    add_test(
        NAME xpbd_adapter_test
        COMMAND xpbd_adapter_test
    )
endif()





# Sphere vs XPBD mesh collision regression test
if(TARGET xpbd_adapter_test)
    add_executable(xpbd_collision_test tests/xpbd_collision_test.cpp
        src/xpbd_adapter.cpp src/xpbd_solver.cpp)
    target_include_directories(xpbd_collision_test PRIVATE ${PROJECT_SOURCE_DIR}/include)
    target_compile_features(xpbd_collision_test PRIVATE cxx_std_20)
    add_test(NAME xpbd_collision_test COMMAND xpbd_collision_test)
endif()


if(SOFT_BODY_BUILD_TESTS)
    add_executable(soft_soft_collision_test
        tests/soft_soft_collision_test.cpp
    )

    target_compile_features(
        soft_soft_collision_test PRIVATE cxx_std_20
    )

    target_link_libraries(
        soft_soft_collision_test PRIVATE soft_body_xpbd_adapter
    )

    add_test(
        NAME soft_soft_collision_test
        COMMAND soft_soft_collision_test
    )
endif()
