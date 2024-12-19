sudo apt-get  update
sudo apt-get install libsdl1.2-dev
		     libsdl2-dev
		     libsdl-image1.2-dev
		     ros-noetic-serial
		     liborocos-bfl-dev
			ros-noetic-tf2-sensor-msgs
			ros-noetic-move-base-msgs
			ros-noetic-costmap-converter
			ros-noetic-mbf-msgs
			ros-noetic-mbf-costmap-core
			ros-noetic-libg2o
			libpcap-dev
			ros-noetic-mpc-local-planner
sudo ln -s /usr/include/eigen3/Eigen  /usr/include/Eigen
echo " 单独编译mpc_local_msgs 和 lslidar_msgs""
