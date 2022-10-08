#Replace CMAKE_CROSS_COMPILE_PREFIX who can be expressed in home based format with a full path equivalent
get_filename_component(CMAKE_CROSS_COMPILE_PREFIX ${CMAKE_CURRENT_LIST_FILE} DIRECTORY )

#File raspberrytoolchain.cmake for ROS and system packages to cross compile.
SET(CMAKE_SYSTEM_NAME Linux)

SET(CMAKE_C_COMPILER aarch64-linux-gnu-gcc) #gcc-aarch64-linux-gnu package
SET(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++) #g++-aarch64-linux-gnu package

# Below call is necessary to avoid non-RT problem.
SET(CMAKE_LIBRARY_ARCHITECTURE aarch64-linux-gnu)

SET(RASPBERRY_ROOT_PATH ${CMAKE_CURRENT_LIST_DIR})

SET(RASPBERRY_NOETIC_PATH ${RASPBERRY_ROOT_PATH}/opt/ros/noetic)

SET(CMAKE_FIND_ROOT_PATH ${RASPBERRY_ROOT_PATH} ${CATKIN_DEVEL_PREFIX})

#If you have installed cross compiler to somewhere else, please specify that path.
SET(COMPILER_ROOT /usr/aarch64-linux-gnu) 

#Have to set this one to BOTH, to allow CMake to find rospack
#This set of variables controls whether the CMAKE_FIND_ROOT_PATH and CMAKE_SYSROOT are used for find_xxx() operations.
SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM BOTH)
SET(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
SET(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
SET(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

#LIST(APPEND CMAKE_PREFIX_PATH ${RASPBERRY_ROOT_PATH}/usr ${RASPBERRY_NOETIC_PATH})
#LIST(REMOVE_DUPLICATES CMAKE_PREFIX_PATH)
#SET(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${RASPBERRY_ROOT_PATH}/usr)

SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --sysroot=${RASPBERRY_ROOT_PATH}" CACHE INTERNAL "" FORCE)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --sysroot=${RASPBERRY_ROOT_PATH}" CACHE INTERNAL "" FORCE)
SET(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} --sysroot=${RASPBERRY_ROOT_PATH}" CACHE INTERNAL "" FORCE)
SET(CMAKE_CXX_LINK_FLAGS "${CMAKE_CXX_LINK_FLAGS} --sysroot=${RASPBERRY_ROOT_PATH}" CACHE INTERNAL "" FORCE)

SET(LD_LIBRARY_PATH ${RASPBERRY_NOETIC_PATH}/lib) 

SET(CMAKE_MODULE_PATH ${RASPBERRY_NOETIC_PATH}/share/cmake_modules/cmake/Modules ${RASPBERRY_ROOT_PATH}/usr)
