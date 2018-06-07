cmake_minimum_required(VERSION 3.5)
project(rs2_ros)

# build rs2_interface::rs2_interface target
add_subdirectory(rs2_interface)

# enable C++11 standard (current directory scope)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON) # prevent fallback
# get_property(_variableNames GLOBAL PROPERTY CMAKE_CXX_STANDARD)
# message(WARNING "CMAKE_CXX_STANDARD=${CMAKE_CXX_STANDARD}")

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS
#  LIBRARIES rs2_ros
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

add_executable(rs2_ros 
  src/main.cpp
)

include_directories(${catkin_INCLUDE_DIRS}) # to find ros/ros.h
# alternatively and more specifically:
# target_include_directories(rs2_ros
#   PRIVATE
#     ${catkin_INCLUDE_DIRS}
# )

# get_property(_variableNames TARGET rs2_interface::rs2_interface PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
# list (SORT _variableNames)
# foreach (_variableName ${_variableNames})
#     message(WARNING "${_variableName}=${${_variableName}}")
# endforeach()

target_link_libraries(rs2_ros
  PRIVATE
    ${catkin_LIBRARIES}
    rs2_interface::rs2_interface # this automatically adds including using target property INTERFACE_INCLUDE_DIRECTORIES
)
