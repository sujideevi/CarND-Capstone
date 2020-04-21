### Udacity Self-Driving Car Nanodegree Capstone project

#### Team
I have completed this project indivdually by referring tutrials provided in previous lessons of Udacity. My details are mentioned below.

Senthil Kumar M(senthil.m63@wipro.com)

### Introduction
This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. I integrated varrious modules of this project with the help of Robot Operating System(ROS). As part of validation process, this code would be first tested on Carla and in Udacity Self driving car on a test track. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

### Implementation Details
This project has been completed in two phases.
* Phase-1: To make the car following the waypoints in the simulator with desired velocity without any speed violation.
* Phase-2: To make the car obeying the traffic signal lights in the simulator.

As part of this project, below 4 nodes have been customized in order to operate the car in simulator as per traffic light.
* waypoint_updater
* dbw_node
* twist_controller
* tl_detector
* tl_classifier


1. waypoint_updater Node - This publishes the fixed number of waypoints ahead of the vehicle with desired target velocities with respect to traffic lights. During first project phase, this node take the coordinates of base waypoint from topic-'/base_waypoints' and current position of car from topic-'/current_pose'. The same has been processed and published the final waypoints over topic-'/final_waypoints'.
   In second phase of the project, the coordinates of signal lights has been taken as input from topic '/traffic_waypoint' and the same has been used for operating the car with respect to traffic lights using the 'tl detector' and 'tl_classifier' node.
   
2. dbw_node - This 'Drive by wire' node consumes the input from topic 'twist_cmd' and compute the appropriate throttle, steering and brake values to the car using various controllers. The computed throttle, brake and steering values would get published in below 3 topics.
* /vehicle/throttle_cmd
* /vehicle/brake_cmd
* /vehicle/steering_cmd

3. twist_controller Node - This node perform the computaion required from controller end and calculate the steering, brake and throttle values with the help of pid.py, yaw_controller.py and low_pass.py.

With the help of 'waypoint_updater Node', 'dbw_node' and 'twist_controller' node, the car can be driven in the simulator lane safely at desired speed just by following the waypoints without considering the status of traffic light. With this first phase of the project has been completed. As part of the second phase, the inputs from '/traffic_waypoint' topic is processed and car would be operated accordingly in order to obey traffic signals.

4. tl_detector Node - This node will collect data from below topics /base_waypoints, /current_pose, /image_color and /vehicle/traffic_lights and publish the waypoint of upcoming red traffic signal over '/traffic_waypoint' topic. These will be then consumed by waypoint updater node and the bake and trottle values will be computed accordingly in order to stop the car for red signal.

5. tl_classifier class - This class implements the traffic light classification model. 'get_classification' in this class will take camera images as input and returns the id corresponding to color state of traffic light.


### Submission Checklist

#### Waypoint following within specified velocity

The code is able to operate the car to follow the waypoint and able to drive within the maximum velocity specified in 'CarND-Capstone/ros/src/waypoint_loader/launch/waypoint_loader.launch'. In both the cases, the car is able to stop and move according to the state of traffic light. The PID controller will stop and restart the vehicle based on the topic '/vehicle/dbw_enabled'

Below is the video taken with maximum velocity set as 20kmph(12.42Mph).

[image1]: ./output/Simulator_20kmph.ogv "Target Speed - 20kmph"

Below is the video taken with maximum velocity set as 40kmph(24.85Mph).

[image1]: ./output/Simulator_40kmph.ogv "Target Speed - 40kmph"


### Port Forwarding
To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.

2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
