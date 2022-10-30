# Openmower cross compilation for Raspberry pi architecture

This folder contains scripts configs and struture  to setup a cross compilation environment for openmower project.
The approach is based on the following article https://medium.com/@tahsincankose/cross-compiling-ros-project-for-arm-263642b405ac.

This cross compilation method is suited for dev avoiding to transfer huge images or amount of data between each compilation, only devel and/or install folders needs to be transferred back to mower, it is quick to work iteratively. It allows to keep using source scripts on the target like if build was made on the raspi.

NB : This approach doesn't detect dependencies changes. Provided bash script `arch-cross-compile-libs.sh` needs to be updated manually based on compiler dependencies missing errors.

## Principle

1. Deploy OpenMower source as per wiki [Software installation section](https://wiki.openmower.de/index.php?title=System_Image)
3. Tar Dependencies
4. Decompress dependencies into aarch64 sysroot folder
5. Replace absolute references in files 
6. Install compiler tools and catkin (we replace catkin_make to compile ROS)
7. Compile using Catkin and transfer to raspberry. 
8. Re-run setup

## Procedure
### 1. Deploy OpenMower source as per wiki (on raspi)
1. Ensure Section _Install ROS Noetic_ is completed
2. Ensure Section _Installing OpenMower Software_ is completed
3. Create a symlink to noetic installation in `~/open_mower_ros/aarch64`  :
```bash
ln -s /opt ~open_mower_ros/aarch64 
``` 
### 2. Tar Dependencies (on raspi)
~~~bash
source arch-cross-compile-libs.sh
~~~
### 4. Decompress dependencies into aarch64 sysroot folder (on dev machine)
~~~bash
pscp ubuntu@openmower:cross-compile-libs.tar.gz ~/open_mower_ros/aarch64/
tar xzvf ~/open_mower_ros/aarch64/cross-compile-libs.tar.gz
~~~
### 5. Replace absolute references in files (on dev machine)
We need to replace all absolute references in noetic.
For that I used VSCode search using regex and replaced content in files as follows :

| to search | from folder | Replace string|
|:--- | :--- | :---
|(?<!\})\/opt\/ros\/noetic | ./aarch64/opt/									|${CMAKE_CROSS_COMPILE_PREFIX}|
|(?<!\})\/usr\/lib         | ./aarch64/opt/									|${CMAKE_CROSS_COMPILE_PREFIX}/usr/lib|
|(?<!\})\/usr\/include     | ./aarch64/opt/									|${CMAKE_CROSS_COMPILE_PREFIX}/usr/include|
|(?<!\})\/usr\/share       | ./aarch64/opt/									|${CMAKE_CROSS_COMPILE_PREFIX}/usr/share|
|;pthread;                 | ./aarch64/opt/ros/noetic/share/roscpp/cmake/	|;${CMAKE_CROSS_COMPILE_PREFIX}/usr/lib/aarch64-linux-gnu/libpthread.so.0;|

### 6. Install compiler tools and catkin (on dev machine)
We replace catkin_make by catkin build better enforcing isolated build and providing more flexible build profiles
~~~bash
sudo apt-get install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu
sudo apt install python3-catkin-tools
bash aarch64/catkin-profiles-setup.sh
~~~

A few CMakeLists.txt/Package.xml needs to be reviewed for catkin build to execute without dependencies errors.
This is due to isolated buidl constraints :
1. Modify `src/mower_utils/package.xml` add <build_depend>slic3r_coverage_planner</build_depend>



### 7. Compile using Catkin (on dev machine)
Build using toolchain via dev catkin profile
~~~
catkin clean -y
catkin build --profile aarch64-dev
~~~
Archive content and transfer to raspi
~~~
tar --directory=/home/peppe/open_mower_ros --exclude=./{aarch64,logs*,build*,devel,*.tar.gz} -czvf ~/arm_open_mower_ros.tar.gz .
pscp ~/arm_open_mower_ros.tar.gz ubuntu@openmower:
~~~

### 8. Uncompress and source
~~~
tar xzvf ~/arm_open_mower_ros.tar.gz  --directory ~/open_mower_ros/
source ~/.bashrc
~~~