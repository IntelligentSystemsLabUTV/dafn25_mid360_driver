cmake_minimum_required(VERSION 3.8)

project(livox_sdk2)

set(CMAKE_CXX_STANDARD         17    CACHE STRING "" FORCE)
set(CMAKE_CXX_STANDARD_REQUIRED ON   CACHE BOOL   "" FORCE)
set(CMAKE_CXX_EXTENSIONS      OFF   CACHE BOOL   "" FORCE)


message(STATUS "main project dir: " ${PROJECT_SOURCE_DIR})

if (CMAKE_CROSSCOMPILING)
	set(THREADS_PTHREAD_ARG
		"PLEASE_FILL_OUT-FAILED_TO_RUN"
		CACHE STRING "Result from TRY_RUN" FORCE)
endif()

if (UNIX)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
endif(UNIX)

add_subdirectory(sdk_core)

