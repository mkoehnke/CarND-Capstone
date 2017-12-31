[//]: # (Image References)
[simulator]: ./doc/simulator.png
[sim_red_detected]: ./doc/sim_red_detected.png
[sim_green_detected]: ./doc/sim_green_detected.png


This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

# Individual Submission

Name 				| Udacity Account Email
---------------- | ---------------------
Mathias Koehnke | mathias.koehnke+udacity(at)gmail.com


## Description

![alt-text][simulator]

### Waypoint Updater

This node publishes waypoints from the car's current position to some `x` distance ahead. If an upcoming stop light is detected, the velocity of the waypoints will be adjusted and the car decelerates or accelerates depending on the light state.

It was implemented by defining the following 4 states and changes between those states:

State 				| Description
---------------- | ---------------------
Start Acceleration | This state determines how much acceleration is needed for the car to reach the target velocity. 
Continue Acceleration | This state continues with the acceleration and keeps the target velocity if reached.
Start Deceleration	  | This state determines how much deceleration is needed for the car to stop at the next stop line.
Continue Deceleration | This state continues with the deceleration and keeps zero speed if reached.

### Drive-By-Wire Node / Twist Controller

#### Drive-By-Wire Node

This node represents a drive by wire controller. It receives current and requested steering/velocities, calculates throttle, brake and steering commands and publishes them to the vehicle.

#### Twist Controller

This controller is responsible for acceleration and steering. The acceleration is controlled via PID controller. Steering is calculated using YawController which simply calculates needed angle to keep needed velocity.

### Traffic Light Detection / Classification

This node is responsible for detecting upcoming traffic lights and classify their states (red, yellow, green).

Tensorflow's [Object Detection API](https://github.com/tensorflow/models/tree/master/research/object_detection) was used to detect and classify the traffic lights in the images provided by the camera.

I used a separate repository to setup an environment to train/evaluate the pre-trained (SSD: Single Shot MultiBox Detector) tensorflow model [here](https://github.com/mkoehnke/CarND-Capstone-TrafficLightDetection).

The image dataset is based on images taken from the simulator and bag files provided for this project. It was downloaded from this [repository](https://github.com/coldKnight/TrafficLight_Detection-TensorFlowAPI) and directly [here](https://drive.google.com/file/d/0B-Eiyn-CUQtxdUZWMkFfQzdObUE/view?usp=sharing).

In order to increase the performance of the prediction, the images are preprocessed and scaled so that the width is max 300px.

#### Examples

![alt-text][sim_green_detected]
![alt-text][sim_red_detected]

### Flaws / Todos

- Zero speed is not kept all the time when waiting at the traffic light. The car is accelerating/decelerting occasionally.

- When driving manually off the track, the car is currently not able to steer back to the middle of the road.

## Installation

Please use **one** of the two installation options, either native **or** docker installation.

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

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
1. Download [training bag](https://drive.google.com/file/d/0B2_h37bMVw3iYkdJTlRSUlJIamM/view?usp=sharing) that was recorded on the Udacity self-driving car (a bag demonstraing the correct predictions in autonomous mode can be found [here](https://drive.google.com/open?id=0B2_h37bMVw3iT0ZEdlF4N01QbHc))
2. Unzip the file
```bash
unzip traffic_light_bag_files.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_files/loop_with_traffic_light.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
