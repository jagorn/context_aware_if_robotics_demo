cmake_minimum_required(VERSION 2.7)

PROJECT(TopologicalGraphGenerator CXX C)

find_package(OpenCV REQUIRED)

get_filename_component(PROJECT_ROOT "${PROJECT_SOURCE_DIR}" ABSOLUTE)

# Build type configuration.
if (CMAKE_BUILD_TYPE)
	string(TOUPPER ${CMAKE_BUILD_TYPE} _build_type)
	set(CMAKE_BUILD_TYPE ${_build_type} CACHE STRING "" FORCE)
endif (CMAKE_BUILD_TYPE)

# Build_type should be DEBUG or RELEASE.
if (CMAKE_BUILD_TYPE STREQUAL "")
	message("No build type specified: forcing to RELEASE.")
	set(CMAKE_BUILD_TYPE "RELEASE" CACHE STRING "" FORCE)
endif (CMAKE_BUILD_TYPE STREQUAL "")

add_definitions(-Wall)
add_definitions(-fPIC)
#add_definitions(-std=c++11)

#32-bit
#add_definitions(-m32)

if (CMAKE_BUILD_TYPE STREQUAL "DEBUG")
	add_definitions(-ggdb)
elseif (CMAKE_BUILD_TYPE STREQUAL "RELEASE")
	add_definitions(-O3)
else (CMAKE_BUILD_TYPE STREQUAL "DEBUG")
	message("Invalid build type specified: doing nothing.")
endif (CMAKE_BUILD_TYPE STREQUAL "DEBUG")

set(EXECUTABLE_OUTPUT_PATH ../bin)
set(LIBRARY_OUTPUT_PATH ../lib)

include_directories(
 ${CMAKE_CURRENT_SOURCE_DIR}
 ${OpenCV_INCLUDE_DIRS}
)


file(GLOB_RECURSE top_graph_gen_src "utils/*.cpp" "*.cpp")

add_library(top_graph_gen SHARED ${top_graph_gen_src}) # .so
target_link_libraries(top_graph_gen pthread ${OpenCV_LIBS})

# binary file
#add_executable(top_graph_gen_bin ${top_graph_gen_src})
#target_link_libraries(top_graph_gen_bin pthread ${OpenCV_LIBS})

set(CMAKE_INSTALL_PREFIX /usr/include)

# Headers
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/ DESTINATION . FILES_MATCHING PATTERN "*.h*")

# Shared object
install(TARGETS top_graph_gen LIBRARY DESTINATION ../lib)
