cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/gcc-arm-none-eabi.cmake")

# Setup compiler settings
set(CMAKE_C_STANDARD 23)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Define the build type
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELEASE ON)
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELWITHDEBINFO ON)

# # Optional: Restrict LTO to Release and RelWithDebInfo build types
# # if you want faster debug builds
# foreach(config RELEASE RELWITHDEBINFO)
#     set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_${config} ON)
# endforeach()

# Set the project name
set(CMAKE_PROJECT_NAME nav_car)

find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    set(CMAKE_C_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
    set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
else()
    message(WARNING "ccache not found, compilation will not be cached.")
endif()

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C CXX ASM)

file(GLOB_RECURSE C_LIBS "./lib/*.c")
file(GLOB_RECURSE CXX_LIBS "./lib/*.cpp")
file(GLOB C_SOURCES "./code/*.c")
file(GLOB CXX_SOURCES "./code/*.cpp")
file(GLOB_RECURSE STORAGE_C_SOURCES "./code/storage/*.c")
file(GLOB_RECURSE STORAGE_CXX_SOURCES "./code/storage/*.cpp")

file(GLOB_RECURSE CAR_4W_SOURCES "./code/car_4w/*.cpp")
file(GLOB_RECURSE CAR_2W_SOURCES "./code/car_2w/*.cpp")

list(FILTER C_LIBS EXCLUDE REGEX ".*/lib/third_party/(littlefs|lvgl)/.*\\.c$")
list(FILTER CXX_LIBS EXCLUDE REGEX ".*/lib/third_party/(littlefs|lvgl)/.*\\.cpp$")

# message(WARNING "source cpp: " ${CXX_SOURCES})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
    ${C_LIBS}
    ${CXX_LIBS}
    ${C_SOURCES}
    ${CXX_SOURCES}
    ${STORAGE_C_SOURCES}
    ${STORAGE_CXX_SOURCES}
    # ${CAR_4W_SOURCES}
    ${CAR_2W_SOURCES}
)

target_link_options(${CMAKE_PROJECT_NAME} PRIVATE)
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -Wl,-u,_printf_float,-u,_scanf_float)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    "./code"
    "./lib"
    "./lib/third_party/flashdb/inc"
    "./lib/third_party/fal/inc"
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    LFS_THREADSAFE
    $<$<CONFIG:Debug>:UART_DBG>
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
