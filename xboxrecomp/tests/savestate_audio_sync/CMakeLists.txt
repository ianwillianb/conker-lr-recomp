cmake_minimum_required(VERSION 3.20)
project(savestate_audio_sync_test C)
set(CMAKE_C_STANDARD 11)

if(WIN32)
    message(FATAL_ERROR "This test currently covers the POSIX guest-lock path")
endif()

find_package(ZLIB REQUIRED)
find_package(Threads REQUIRED)

add_executable(savestate_audio_sync_test
    test_main.c
    ../../../src/clx_savestate.c
)
target_include_directories(savestate_audio_sync_test PRIVATE
    ../../../src
    ../../src
    ../../src/kernel
    ../../src/nv2a
    ../../src/apu
    ../../include
)
target_compile_options(savestate_audio_sync_test PRIVATE
    -ffunction-sections
    -fdata-sections
)
target_link_options(savestate_audio_sync_test PRIVATE -Wl,--gc-sections)
target_link_libraries(savestate_audio_sync_test PRIVATE ZLIB::ZLIB Threads::Threads)

enable_testing()
add_test(NAME savestate_audio_sync COMMAND savestate_audio_sync_test)
set_tests_properties(savestate_audio_sync PROPERTIES
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
    ENVIRONMENT "CLX_SAVESTATE_DIR=${CMAKE_CURRENT_BINARY_DIR}/states"
)
