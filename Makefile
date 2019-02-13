all:
	catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release
controller:
	roslaunch position_controller_cdt position_controller_cdt.launch 
carrot:
	roslaunch lidar_navigation_cdt challenge.launch
