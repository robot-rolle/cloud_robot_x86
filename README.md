cloud robot
-----------
this source code is used for v1.0 cloud robot agv design
pelase follow the following rules
* free used for study
* none used for business
--------------------

First before use the code,
1. please update the package 
    * sudo apt-get update
    * sudo apt-get install ros-noetic-mpc-local-planner
    * libeigen3-dev
    * libsdl1.2-dev
    * libsdl2-dev
    * libsdl-image1.2-dev
    * ros-noetic-serial
    * liborocos-bfl-dev
    * ros-noetic-tf2-sensor-msgs
    * ros-noetic-move-base-msgs
    * ros-noetic-costmap-converter
    * ros-noetic-mbf-msgs
    * ros-noetic-mbf-costmap-core
    * ros-noetic-libg2o
    * libpcap-dev
    * ros-noetic-mpc-local-planner
2. please link the eigen3 to the eigen3
    * sudo ln -s /usr/include/eigen3/Eigen  /usr/include/Eigen

-------------
Second please follow the steps to build the code
1. cd src && cd ..
2. catkin_make -DCATKIN_WHITELIST_PACKAGES="mpc_local_planner_msgs"
3. catkin_make -DCATKIN_WHITELIST_PACKAGES="lslidar_msgs"
4. catkin_make -DCATKIN_WHITELIST_PACKAGES=""

----------
