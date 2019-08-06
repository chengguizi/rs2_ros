cmake_minimum_required(VERSION 3.5)
project(rs2_ros)

# build rs2_interface::rs2_interface target
add_subdirectory(rs2_interface)

# enable C++11 standard (current directory scope)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON) # prevent fallback
# get_property(_variableNames GLOBAL PROPERTY CMAKE_CXX_STANDARD)
# message(WARNING "CMAKE_CXX_STANDARD=${CMAKE_CXX_STANDARD}")


if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
    "MinSizeRel" "RelWithDebInfo")
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-pg")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  std_msgs
  image_transport
  cv_bridge

  message_generation
)

find_package(OpenCV 3 REQUIRED)

add_message_files(
  FILES
  CameraStats.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS
#  LIBRARIES rs2_ros
 CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)




add_executable(rs2_ros 
  src/main.cpp
  src/camera_manager.cpp
  src/ros_publisher.cpp
  src/exposure_ctl.cpp
)

# add_executable(rs2_fpga_bridge 
#   src/rs2_fpga_bridge.cpp
#   src/exposure_ctl.cpp
# )

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
    ${OpenCV_LIBRARIES}
)

# target_link_libraries(rs2_fpga_bridge
#   PRIVATE
#     ${catkin_LIBRARIES}
#     rs2_interface::rs2_interface # this automatically adds including using target property INTERFACE_INCLUDE_DIRECTORIES
#     ${OpenCV_LIBRARIES}
# )
