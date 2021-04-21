# kinova_pick_place

Repository for kinova arm j2s7s300 refers to jaco v2 7DOF spherical 3fingers

## Getting Started

The project is based on Ubuntu 18.04, ROS Melodic, gazebo 9.0, wstool and catkin tools. Links to the instructions for installing ROS, Gazebo, etc. are provided below
- [Install ROS](http://wiki.ros.org/melodic/Installation/Ubuntu)
- [Install gazebo 9.0](http://gazebosim.org/tutorials?cat=install&tut=install_ubuntu&ver=9.0)
- [Install wstool](http://wiki.ros.org/wstool#Installation)
- [Install catkin-tools](http://catkin-tools.readthedocs.io/en/latest/installing.html)

### Installation
- Source the ROS environment script
	
	```
	source /opt/ros/melodic/setup.bash
	```
- Create a "kinova_ws" catkin workspace
 
	```
	mkdir -p ~/kinova_ws/src
	cd ~/kinova_ws/
	catkin init
	```
- Fetch the robojanitor source code:

	```
	cd ~/kinova_ws/src
	wstool init .
  	wstool merge https://cdn.jsdelivr.net/gh/Preethi1609/kinova_pick_place@preethi-dev/kinova.rosinstall	
	wstool update
	rosdep install --from-paths . --ignore-src -y
	```
- Build the workspace

	```
	cd ~/kinova_ws
	catkin build
	```

## Visualize The Simulation Environment

### Setup
- Source the catkin workspace 

	```
	cd ~/kinova_ws
	source devel/setup.bash
	```

### Launch Rviz
	
	
	
   ```
      roslaunch kinova_moveit_config demo.launch
   ```	
    
   This opens Rviz to visualize the kinova arm.
    

### Run the pick and place program

1. Run the kinova_pick_and_place_node:

    ```
        rosrun kinova_pick_and_place kinova_pick_and_place_node
    ```
 


