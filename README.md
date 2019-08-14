# Applications for Mico Robot Arm

This repository contains ROS packages that are needed to operate the Mico robot arm developed by Kinova.  The implemented system can be used as a base for extension and implementation of functionalities for the robot arm.

This README contains an overview of the recommended skills paired with useful tutorials/wiki-pages/github repo's that can prove useful to read before working with the nodes in this repository. Subsequently an overview of the implemented system is given and the functionalities of this system are discussed. 

Before doing any further work on this project, it is recommended to have a look at the [Recommended Skills](#recommended-skills) section. It is also advisable that any further work done on this repository is implemented and tested on the desktop that already contains this project instead of reinstalling the project on another computer. However, to facilitate reinstalation, a seperate markdown file is included that addresses the different components that have to be installed in case a [full (re)install](INSTALL.md) is needed.

## Recommended Skills
* Python2.7
* Linux (Ubuntu 16.04)
* ROS (Kinetic)
* XML

## High-level overview of the system
<figure align="center">
	<img src="images/Overview_ROS.png?raw=true" alt="Overview ROS"/>
</figure>


Shown in this figure are the two main subcomponents of this project: [<i>Speech-To-Logic-Form</i>](#speech-to-logic-form) and [<i>Logic-Form-To-Robot-Action</i>](#logic-form-to-robot-action)

## Speech-To-Logic-Form
The purpose of this component is transforming speech, input through a microphone, to a logical format that can be processed easily by the <i>Logic-Form-To-Robot-Action</i> component.

### speech_node

Contains the speech recording component which accepts spoken and typed input.

### speech_recognizer_node

Contains the component that will transform the audio input into a textual representation.

### parser_node
 
This Java package contains the text-to-logic-form parsing component which will tranform text into logic form understandable by the robot arm.

### main_node

This ROS package contains the main node that will be running constantly while the system is being executed, it will direct messages and issue service requests to other components.

## Logic-Form-To-Robot-Action

### observer_node

Contains the oberver component which uses the kinect2 camera to detec AR_tags

### actuator_node

Contains the actuation component that will send commands to the robot through a moveit configuration which was constructed specifically for the Mico robot arm.

Executing catkin_make in the root of this workspace will build the project


## Built With

* [ROS](http://www.ros.org/) - The Robot operating system
* [Catkin](https://www.wiki.ros.org/catkin) - Dependency Management and building nodes
* [Kinova-ROS](https://github.com/Kinovarobotics/kinova-ros) - ROS nodes that facilitate use of Kinova robot arms
* [iai_kinect2](https://github.com/code-iai/iai_kinect2) - Incorporating the kinect2 camera in the Robot Operating System

## Authors

* **Shani Vanlerberghe** - *Deploying speech to logic for in ROS*
* **Pieter-Jan Coenen** - *The text to logic form parser*

## License

## Acknowledgments

