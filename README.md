# kinova_pick_place

Repository for kinova arm j2s7s300 refers to jaco v2 7DOF spherical 3fingers

## Getting Started

The project is based on Ubuntu 18.04, ROS Melodic, wstool and catkin tools. Links to the instructions for installing ROS, Gazebo, etc. are provided below
- [Install ROS](http://wiki.ros.org/melodic/Installation/Ubuntu)
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
      roslaunch kinova_arm_moveit_config demo.launch
   ```	
    
   This opens Rviz to visualize the kinova arm.
    

### Run the programs:

1. Run the kinova_arm_pick_and_place_node:

    ```
        rosrun kinova_arm_pick_and_place kinova_arm_pick_and_place_node
    ```
	Note: Uncomment any pick/place location from the list of 12 different positions 
	
2. Run the kinova_arm_iksolver_node:

    ```
        rosrun kinova_arm_iksolver kinova_arm_iksolver_node
    ```
	Note: IK solution for any end effector position and orientation can be computed by modifying the "eef_pose". A green marker is published at the position if an IK solution was found, a red marker is published if no IK solution was found.

3. Run the kinova_arm_reachability_node:

    ```
        rosrun kinova_arm_reachability kinova_arm_reachability_node 
    ```
	Note: A plot of the reachable workspace in front of the arm can be visualized in Rviz.



