cmake_minimum_required(VERSION 3.23)

project(fluid LANGUAGES CXX)

set(CMAKE_CXX_STANDART 20)
set(CMAKE_CXX_STANDARD_REQUIRED on)

set(CMAKE_CXX_FLAGS_DEBUG "-std=gnu++20 -fsanitize=undefined,address,float-divide-by-zero,float-cast-overflow -g3 -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-Werror -Wall -Wextra -std=gnu++20 -g3 -O2")

set(SOURCES
    src/main.cpp
    src/thread_pool/thread_pool.cpp
        )

if (DEFINED TYPES)
    add_compile_definitions("TYPES=${TYPES}")
else()
    message(FATAL_ERROR "TYPES compile option is required")
endif()


if (DEFINED SIZES)
    add_compile_definitions("SIZES=${SIZES}")
endif()


add_executable(${CMAKE_PROJECT_NAME} ${SOURCES})

target_include_directories(${CMAKE_PROJECT_NAME} 
    PRIVATE 
    src/
    src/constexpr/
    src/simulation/
    src/io/
    src/types/
    src/thread_pool/)