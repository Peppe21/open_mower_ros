# Openmower cross compilation for Raspberry pi architecture

This folder contains scripts configs and struture  to setup a cross compilation environment for openmower project.
The approach is based on the following article https://medium.com/@tahsincankose/cross-compiling-ros-project-for-arm-263642b405ac.

This cross compilation method is mostly intended for making regular code changes, avoiding to transfer huge images or amount of data between each compilations, only devel and/or install folders needs to be transferred back to mower, suited to work iteratively. It allows to keep using source scripts on the target like if build was made on the raspi.

NB : This approach doesn't detect dependencies changes. Script `arch-cross-compile-libs.sh` needs to be updated manually based on dependencies missing errors.

## Principle

1. Deploy OpenMower source (on raspi)
2. Tar Dependencies (on raspi)
3. Decompress dependencies into aarch64 sysroot folder (on dev machine)
4. Replace absolute references in files (on dev machine) 
5. Install compiler tools and catkin (on dev machine) 
6. Compile using Catkin (on dev machine) 
7. Uncompress and source
8. Repeat from 6. as dev requires


## Setup
### 1. Deploy OpenMower source (on raspi)
1. Ensure Section _Install ROS Noetic_ is completed (as per OpenMower [wiki](https://wiki.openmower.de/index.php?title=System_Image))
2. Ensure Section _Installing OpenMower Software_ is completed (as per OpenMower [wiki](https://wiki.openmower.de/index.php?title=System_Image))
3. Create a symlink to noetic installation in `~/open_mower_ros/aarch64`  :
```bash
ln -s /opt ~/open_mower_ros/aarch64 
``` 

### 2. Install compiler tools and catkin (on dev machine)
We replace catkin_make by catkin build providing more flexible build profiles and ensuring systematic build isolation
~~~bash
sudo apt-get install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu
sudo apt install python3-catkin-tools
bash aarch64/catkin-profiles-setup.sh
~~~

## Cross compilation libraries update procedure
### 1. Update dependencies - prepare source (on dev machine)
1. Pack the current project state and copy to raspi
~~~bash
tar -czvf ~/arm_open_mower_ros.tar.gz --directory=$HOME/open_mower_ros --exclude=./{.git*,.catkin*,.vscode,aarch64/opt,aarch64/usr,aarch64/*.tar.*,logs*,build*,devel,*.tar.gz,mower_config.sh} .
pscp ~/arm_open_mower_ros.tar.gz ubuntu@openmower:
~~~
### 2. Update dependencies - deploy source (on raspi)
1. Update project folder
~~~bash
find ~/open_mower_ros -mindepth 1 ! -regex '/home/peppe/open_mower_ros/mower_config.sh' -delete
tar xzvf ~/arm_open_mower_ros.tar.gz  --directory ~/open_mower_ros/
rosdep install --from-paths src --ignore-packages-from-source --default-yes
~~~

### 2. Tar Dependencies (on raspi)
~~~bash
source ~/open_mower_ros/aarch64/arch-cross-compile-libs.sh
~~~
### 3. Copy and decompress dependencies into aarch64 sysroot folder (on dev machine)
~~~bash
pscp ubuntu@openmower:cross-compile-libs.tar.gz ~/open_mower_ros/aarch64/
tar xzvf ~/open_mower_ros/aarch64/cross-compile-libs.tar.gz --directory ~/open_mower_ros/aarch64/
~~~
### 4. Replace absolute references in files (on dev machine)
We need to replace all absolute references in noetic.
For that I used VSCode search using regex and replaced content in files as follows :

| to search | from folder | Replace string|
|:--- | :--- | :---
|(?<!\\})\/opt\/ros\/noetic | ./aarch64/opt/									|${CMAKE_CROSS_COMPILE_PREFIX}|
|(?<!\\})\/usr\/lib         | ./aarch64/opt/									|${CMAKE_CROSS_COMPILE_PREFIX}/usr/lib|
|(?<!\\})\/usr\/include     | ./aarch64/opt/									|${CMAKE_CROSS_COMPILE_PREFIX}/usr/include|
|(?<!\\})\/usr\/share       | ./aarch64/opt/									|${CMAKE_CROSS_COMPILE_PREFIX}/usr/share|
|;pthread;                 | ./aarch64/opt/ros/noetic/share/roscpp/cmake/	|;${CMAKE_CROSS_COMPILE_PREFIX}/usr/lib/aarch64-linux-gnu/libpthread.so.0;|



A few CMakeLists.txt/Package.xml needs to be reviewed for catkin build to execute without dependencies errors.
This is due to isolated buidl constraints :
1. Modify `src/mower_utils/package.xml` add <build_depend>slic3r_coverage_planner</build_depend>

### 6. Compile using Catkin (on dev machine)
Build using toolchain via dev catkin profile
~~~
catkin clean -y
catkin build --profile aarch64-dev
~~~
Archive content and transfer to raspi
~~~
tar --directory=/home/peppe/open_mower_ros --exclude=./{.git*,aarch64,logs*,build*,devel,*.tar.gz,mower_config.sh} -czvf ~/arm_open_mower_ros.tar.gz .
pscp ~/arm_open_mower_ros.tar.gz ubuntu@openmower:
~~~

### 7. Uncompress and source
~~~
tar xzvf ~/arm_open_mower_ros.tar.gz  --directory ~/open_mower_ros/
source ~/.bashrc
~~~