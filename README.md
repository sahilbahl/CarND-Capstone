## Udacity self-driving Car Capstone Project : System Integration
In this project, my teammates and I developed a system which integrated multiple modules to drive a real car, Carla, in a test lot. The Udacity self-driving car program takes us down the path towards building a real autonomous vehicle and this project serves as a final reflection on what we have learned from the program. Upon completing the project, I have special thanks to my teammates. Without them, I would have spent more crazy hours scratching my head and going nowhere. It has been a great experience to team up with you guys.

## Project Team Members of "No-LEFT-TURN"
|Name              |Udacity Account Email Address|
|------------------|-----------------------------|
|Sahil Bahl        |bahlsahil28@gmail.com        |
|Akiyuki Ishikawa  |aski.y.ishikawa@gmail.com    |
|Muhsin Mohamn     |askmuhsin@gmail.com          |
|Wei-Fen Lin       |weifen@mijotech.net          |

## Project Overview

   In this project, we have to write ROS nodes to implement core functionality of the autonomous vehicle system, including traffic light detection, control, and waypoint following. The code is tested using a simulator and run on a real car, Carla. The following is a system architecture diagram showing the ROS nodes and topics used in the project.
![](/imgs/system_architecture.png)

   Carla has three subsystems:

   1. Perception - Carla's camera and sensors detect obstacles and traffic lights. In this project, we need to implement a node for traffic light detection. We chose to train a deep learning network model for the traffic light classification.

   2. Planning - The planning subsystem (node waypoint updater) updates the waypoints and the associated target velocities. It consisits of two modules, waypoint_updater and waypont_loader. In this project implementation, a list of final waypoints will be updated using the information from the simulation and the perception module.

   3. Control - The control subsystem actuates the throttle, steering, and brake to navigate the waypoints with the target velocity. Carla is equipped with a drive-by-wire(dbw) system and we are expected to implement the DBW node that generates the throttle, steering, and brake control signals.

## Starter Code Setup
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

## Implementation HighLight
### Waypoint Updater
  The walkthrough section gives a detailed implementation on the first part of waypoint updater. The eventual purpose of this node is to publish a fixed number of waypoints ahead of the vehicle with the correct target velocities, depending on traffic lights and obstacles. The goal for the first version of the node should be simply to subscribe to the topics.
-   `/base_waypoints`
-   `/current_pose`

and publish a list of waypoints to

-   `/final_waypoints`

Once the implementation is done, we can run the simulation and see a line of green dots shown on the simulator, which depicts the generated waypoints.ß
I changed LOOKAHEAD_WPS from to 50 in the final submission to reduce the computation overhead. During debugging,

### Twist Controller
  Within the twist controller package, two modules are required to be implmented, dbw_node.py and twist_controller.py. In dbw_node.py, we need to handle ROS subscribers for  the /current_velocity, /twist_cmd, and /vehicle/dbw_enabled topics and publish thottle, steering and brake signals. During debugging, it might help to reduce the publish rate and see the change in the simulation. However, the DBW system on Carla expects messages at 50Hz, and will disengage (reverting control back to the driver) if control messages are published at less than 10hz. This is a safety feature on the car intended to return control to the driver if the software system crashes.  Therefore, we need to make sure the simulation still works at 50Hz in the final implementation.

### Traffic Light Detection
  The tasks for this package were broken into two parts. In the first part, we need to implement the tl_detector.py module. The walkthrough section gives enough details to implement this module. What is not mentioned in the walkthrough code is the second part, to build a traffic light classifier. Most people used the tensorflow object dection API for this project. There is a very good reference from Alex Lechner at https://github.com/alex-lechner/Traffic-Light-Classification. It gives a detailed tutorial on how to build a traffic light classifier in this project. I followed the same methodoligy to test a couple of pre-trained models in the tensowflow library.
  I end up using the SSD Inception V2 model for this project. Two seperate models are trained for simulator and real-world testing. Both models were trained for 20,000 steps.
