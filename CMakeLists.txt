cmake_minimum_required(VERSION 3.20)

project(N8ROCharacterMotion
    VERSION 0.1.0
    DESCRIPTION "CoM-aware kinematic 10-joint motion controller for N8RO"
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(N8RO_MOTION_BUILD_TESTS "Build controller safety tests" ON)
set(N8RO_RELEASE_ROOT "C:/N8RO" CACHE PATH "N8RO release root that contains include, lib, and userPlugins folders")

add_library(n8ro_motion_core STATIC
    src/motion_controller.cpp
)

target_include_directories(n8ro_motion_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(n8ro_motion_core PUBLIC cxx_std_17)

add_library(n8ro_motion SHARED
    src/n8ro_adapter.cpp
)

target_link_libraries(n8ro_motion PRIVATE n8ro_motion_core)

target_include_directories(n8ro_motion
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

set_target_properties(n8ro_motion PROPERTIES
    OUTPUT_NAME "n8ro_character_motion"
)

add_executable(n8ro_motion_demo
    src/demo.cpp
)

target_link_libraries(n8ro_motion_demo PRIVATE n8ro_motion_core)

if(EXISTS "${N8RO_RELEASE_ROOT}/include/astsim/model/AnimationModel.h")
    add_library(n8ro_motion_sim_plugin SHARED
        src/astsim_animation_plugin.cpp
    )

    target_link_libraries(n8ro_motion_sim_plugin
        PRIVATE
            n8ro_motion_core
            "${N8RO_RELEASE_ROOT}/lib/arkheon-astlib.lib"
            "${N8RO_RELEASE_ROOT}/lib/arkheon-astdb.lib"
            "${N8RO_RELEASE_ROOT}/lib/n8ro-db.lib"
            "${N8RO_RELEASE_ROOT}/lib/arkheon-astsim.lib"
    )

    target_include_directories(n8ro_motion_sim_plugin
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/include
            "${N8RO_RELEASE_ROOT}/include/astlib"
            "${N8RO_RELEASE_ROOT}/include/astsim"
            "${N8RO_RELEASE_ROOT}/include/astdb"
            "${N8RO_RELEASE_ROOT}/include/n8ro-db"
    )

    target_compile_definitions(n8ro_motion_sim_plugin
        PRIVATE
            WIN32_LEAN_AND_MEAN
            NOMINMAX
            ARKHEON_ASTLIB_EXPORTS
    )

    set_target_properties(n8ro_motion_sim_plugin PROPERTIES
        OUTPUT_NAME "character_plugin_220201014"
    )

    if(MSVC)
        target_compile_options(n8ro_motion_sim_plugin PRIVATE /W4 /permissive-)
    else()
        target_compile_options(n8ro_motion_sim_plugin PRIVATE -Wall -Wextra -Wpedantic)
    endif()
endif()

if(MSVC)
    target_compile_options(n8ro_motion_core PRIVATE /W4 /permissive-)
    target_compile_options(n8ro_motion PRIVATE /W4 /permissive-)
    target_compile_options(n8ro_motion_demo PRIVATE /W4 /permissive-)
else()
    target_compile_options(n8ro_motion_core PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(n8ro_motion PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(n8ro_motion_demo PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(N8RO_MOTION_BUILD_TESTS)
    enable_testing()

    add_executable(controller_safety_tests
        tests/controller_safety_tests.cpp
    )

    target_link_libraries(controller_safety_tests PRIVATE n8ro_motion_core)

    if(MSVC)
        target_compile_options(controller_safety_tests PRIVATE /W4 /permissive-)
    else()
        target_compile_options(controller_safety_tests PRIVATE -Wall -Wextra -Wpedantic)
    endif()

    add_test(NAME controller_safety_tests COMMAND controller_safety_tests)

    add_executable(arkheon_abi_tests
        tests/arkheon_abi_tests.cpp
        src/n8ro_adapter.cpp
    )

    target_link_libraries(arkheon_abi_tests PRIVATE n8ro_motion_core)

    if(MSVC)
        target_compile_options(arkheon_abi_tests PRIVATE /W4 /permissive-)
    else()
        target_compile_options(arkheon_abi_tests PRIVATE -Wall -Wextra -Wpedantic)
    endif()

    add_test(NAME arkheon_abi_tests COMMAND arkheon_abi_tests)
endif()
