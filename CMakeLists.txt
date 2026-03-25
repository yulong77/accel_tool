cmake_minimum_required(VERSION 3.20)

project(AccelTool LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

add_library(acceltool_core
    src/backend/wireless_accelerometer_manager.cpp
    src/core/app_config.cpp
    src/processing/magnitude_calculator.cpp
    src/processing/display_aggregator.cpp
    src/utils/csv_writer.cpp
    src/utils/display_csv_writer.cpp
    src/utils/logger.cpp
)

target_include_directories(acceltool_core
    PUBLIC
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/third_party/mscl/include
)

target_link_directories(acceltool_core
    PUBLIC
        ${CMAKE_SOURCE_DIR}/third_party/mscl/lib
)

target_link_libraries(acceltool_core
    PUBLIC
        MSCL
)

add_executable(AccelTool
    src/main.cpp
)

target_link_libraries(AccelTool
    PRIVATE
        acceltool_core
)

add_custom_command(TARGET AccelTool POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_SOURCE_DIR}/third_party/mscl/bin/MSCL.dll
        $<TARGET_FILE_DIR:AccelTool>
)

if(MSVC)
    target_compile_options(acceltool_core PRIVATE /W4 /permissive-)
    target_compile_options(AccelTool PRIVATE /W4 /permissive-)
else()
    target_compile_options(acceltool_core PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(AccelTool PRIVATE -Wall -Wextra -Wpedantic)
endif()
