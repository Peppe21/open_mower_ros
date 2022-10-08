cd ~/open_mower_ros
catkin profile add aarch64-dev
catkin profile set aarch64-dev
catkin config --extend ~/open_mower_ros/aarch64/opt/ros/noetic --build-space ~/open_mower_ros/aarch64/build --devel-space ~/open_mower_ros/aarch64/devel --install-space ~/open_mower_ros/aarch64/install --log-space ~/open_mower_ros/aarch64/logs
catkin config --cmake-args -DCMAKE_TOOLCHAIN_FILE=~/open_mower_ros/aarch64/toolchain.cmake -DCMAKE_CROSS_COMPILE_PREFIX=~/open_mower_ros/aarch64 -DCMAKE_BUILD_TYPE=Debug
catkin profile add aarch64-rel
catkin profile set aarch64-rel
catkin config --extend ~/open_mower_ros/aarch64/opt/ros/noetic --build-space ~/open_mower_ros/aarch64/build --devel-space ~/open_mower_ros/aarch64/devel --install-space ~/open_mower_ros/aarch64/install --log-space ~/open_mower_ros/aarch64/logs
catkin config --cmake-args -DCMAKE_TOOLCHAIN_FILE=~/open_mower_ros/aarch64/toolchain.cmake -DCMAKE_CROSS_COMPILE_PREFIX=~/open_mower_ros/aarch64 -DCMAKE_BUILD_TYPE=Release
catkin profile set aarch64-dev

