#!/bin/bash

cd "${HOME}/open_mower_ros"
catkin profile remove aarch64-dev
catkin profile add aarch64-dev
catkin profile set aarch64-dev
catkin config --extend ~/open_mower_ros/aarch64/opt/ros/noetic --build-space ~/open_mower_ros/build-aarch64 --devel-space ~/open_mower_ros/devel-aarch64 --install-space ~/open_mower_ros/install-aarch64 --log-space ~/open_mower_ros/logs-aarch64
catkin config --cmake-args -DCMAKE_TOOLCHAIN_FILE=~/open_mower_ros/aarch64/toolchain.cmake -DCMAKE_CROSS_COMPILE_PREFIX=~/open_mower_ros/aarch64 -DCMAKE_BUILD_TYPE=Debug
catkin profile remove aarch64-rel
catkin profile add aarch64-rel
catkin profile set aarch64-rel
catkin config --extend ~/open_mower_ros/aarch64/opt/ros/noetic --build-space ~/open_mower_ros/build-aarch64 --devel-space ~/open_mower_ros/devel-aarch64 --install-space ~/open_mower_ros/install-aarch64 --log-space ~/open_mower_ros/logs-aarch64
catkin config --cmake-args -DCMAKE_TOOLCHAIN_FILE=~/open_mower_ros/aarch64/toolchain.cmake -DCMAKE_CROSS_COMPILE_PREFIX=~/open_mower_ros/aarch64 -DCMAKE_BUILD_TYPE=Release
catkin config --install
catkin profile set aarch64-dev
