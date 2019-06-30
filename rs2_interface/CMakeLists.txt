cmake_minimum_required(VERSION 3.5)
project(rs2_interface)

# enable C++11 standard (current directory scope)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON) # prevent fallback
# alternatively:
# set_target_properties( rs2_interface PROPERTIES
#   CXX_STANDARD 11 
#   CXX_STANDARD_REQUIRED ON
# )

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
    "MinSizeRel" "RelWithDebInfo")
endif()

message(STATUS "CMAKE_BUILD_TYPE = ${CMAKE_BUILD_TYPE}")

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-pg")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(OpenCV REQUIRED)
#message(WARNING "OpenCV_INCLUDE_DIRS = ${OpenCV_INCLUDE_DIRS}")
find_package(realsense2 REQUIRED)
find_package(Threads REQUIRED)

# This section is a work-around for bad naming conventions in librealsense Config.cmake file
if(realsense2_FOUND) # this is always true if the *Config.cmake file is present, even if it is empty!
  #                                   v
  list(APPEND realsense2_LIBRARIES    ${realsense2_LIBRARY})
  #list(APPEND realsense2_INCLUDE_DIRS ${realsense_INCLUDE_DIR})
  #                                   X
else()
  message("NOT founded!!")
endif()

# Set the output folder where your program will be created
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
#set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
#set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)


add_library(rs2_interface STATIC
  src/stereo_interface.cpp
)
add_library(rs2_interface::rs2_interface ALIAS rs2_interface)

target_include_directories( rs2_interface
  PUBLIC 
    ${PROJECT_SOURCE_DIR}/include
)

# static lib dependency, reference https://floooh.github.io/2016/01/12/cmake-dependency-juggling.html
target_link_libraries( rs2_interface
  PUBLIC
    realsense2::realsense2
    Threads::Threads
)



# test build generation (optional)

# add_executable(rs2-test-nolib src/test-nolib.cpp)
# target_link_libraries(rs2-test-nolib 
# ${OpenCV_LIBRARIES}
# ${realsense2_LIBRARIES}
# )



include_directories(${OpenCV_INCLUDE_DIRS})

link_libraries(
${OpenCV_LIBRARIES}
rs2_interface::rs2_interface
)

add_executable(rs2-test-d400 src/test-d400.cpp)
add_executable(rs2-test-t265 src/test-t265.cpp)
add_executable(rs2-test-nolib src/test-nolib.cpp)

add_executable(rs2-test-enumerate src/test-enumerate.cpp)

# great examples are here: https://github.com/ttroy50/cmake-examples
