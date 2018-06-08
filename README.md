# ROS Node for Realsense D415 Streams
## Dependencies
Realsense SDK 2.0 from [here](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md)

Tested on Ubuntu 16.04, kernel 4.4, 4.10 & 4.13
## How to build
```
(cd into catkin-workspace/src/)
git clone (rs2_ros)
catkin build

source ./devel/setup.bash
rosrun rs2_ros rs2_ros
```
## Debug librealsense
```
export LRS_LOG_LEVEL=DEBUG
(then run the binary normally)
```
## How to clean
Simply remove all files from the ./build/ directory.

## SDK References
[API How To](https://github.com/IntelRealSense/librealsense/wiki/API-How-To)  
[OpenCV](https://github.com/IntelRealSense/librealsense/tree/master/wrappers/opencv)  
[Latency Issues](https://github.com/IntelRealSense/librealsense/issues/1242)
[SDK2.0 Support Models](https://github.com/IntelRealSense/librealsense/blob/cb0c6f0b4cbc13a4af79c016e4cad6db67e3a086/doc/rs400_support.md)