cmake_minimum_required(VERSION 2.8.9)
project(rs2-interface)

find_package(OpenCV REQUIRED)
#message("OpenCV_LIBRARIES = ${OpenCV_LIBRARIES}")
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

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS rs2-interface/src
#  LIBRARIES rs2-interface
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)


set(CMAKE_CXX_STANDARD 11)  # enable C++11 standard
# Set the output folder where your program will be created
#set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/bin)
#set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
#set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

include_directories(
  src
  ${catkin_INCLUDE_DIRS}
#   ${OpenCV_INCLUDE_DIRS} # Not needed for CMake >= 2.8.11
#   ${realsense2_INCLUDE_DIRS}
)

add_executable(rs2-interface 
  src/main.cpp
  src/irstereo.cpp
)
add_executable(rs2-test src/test.cpp)

target_link_libraries(rs2-interface 
${OpenCV_LIBRARIES}
${realsense2_LIBRARIES}
${catkin_LIBRARIES}
Threads::Threads
)

target_link_libraries(rs2-test 
${OpenCV_LIBRARIES}
${realsense2_LIBRARIES}
)
