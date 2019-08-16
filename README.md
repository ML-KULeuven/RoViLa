
# Applications for Mico Robot Arm

This repository contains ROS packages that are needed to operate the Mico robot arm developed by Kinova.  The implemented system can be used as a base for extension and implementation of functionalities for the robot arm.

This README contains an overview of the recommended skills paired with useful tutorials/wiki-pages/github repo's that can prove useful to read before working with the nodes in this repository. Subsequently an overview of the implemented system is given and the functionalities of this system are discussed. 

Before doing any further work on this project, it is recommended to have a look at the [Recommended Skills](#recommended-skills) section. It is also advisable that any further work done on this repository is implemented and tested on the desktop that already contains this project instead of reinstalling the project on another computer. However, to facilitate reinstalation, a seperate markdown file is included that addresses the different components that have to be installed in case a [full (re)install](INSTALL.md) is needed.

## Recommended Skills
* Python2.7 --> ROS  does not support python3 in kinetic
* Linux (Ubuntu 16.04)
* ROS (Kinetic)
* XML

## Prerequisites
See [(re)install-instructions](INSTALL.md) to verify that you have everything in case you plan on doing a reinstall.

Other prerequisites:
* pip : ```sudo apt-get install python-pip```

## High-level overview of the system
<!--figure align="center">
	<img src="images/Overview_ROS.png?raw=true" alt="Overview ROS"/>
</figure-->


Shown in this figure are the two main subcomponents of this project: [<i>Speech-To-Logic-Form</i>](#speech-to-logic-form) and [<i>Logic-Form-To-Robot-Action</i>](#logic-form-to-robot-action). The first will retrieve speech input and parse it to logic form by using a pre-trained parser. The second will take this parsed string as input and use observations of the world around the robot to execute the action expressed in the logic form.

## Speech-To-Logic-Form
The purpose of this component is transforming speech, input through a microphone, to a logical format that can be processed easily by the <i>Logic-Form-To-Robot-Action</i> component. 

You can run this component by executing the following command in terminal:

```bash
roslaunch main_node main_process.launch
```

The [main_process.launch file](src/main_node/launch/main_process.launch) contains the ROS nodes that need to be started when the above command is executed. This launch file will launch the following ROS nodes (in order): the <i>main\_node</i>, the <i>speech\_node</i>, the <i>speech\_recognizer\_node</i> and finally the <i>parser\_node</i>.

### main_node

This ROS package contains the main node that will be running constantly while the <i>Speech-To-Logic-Form</i> component is being executed. The purpose of this node is to facilitate communication between nodes. Therefore <i>main\_node</i> will act as a middleman that requests services provided by the other 3 components. Finally, once the services provided by the <i>speech\_node</i>, <i>speech\_recognizer\_node</i> and <i>parser\_node</i> have been called, the main node will publish the parsed logic form as a String on the ```/actuation_command``` topic.

### speech_node

The speech\_node ROS package provides the <i>SpeechStream</i> service to the main node. At the start of the system the main node will request input from the speech service. The speech service will record what the user says and parse this recording into a string array.

```bash
 # request fields
int64 chunk_size
int64 sample_format
int64 channels
int64 frequency
int64 seconds
---
 # response fields
string[] stream
```
 
This ROS package uses the PyAudio python streaming library, to enable speech input from python code.  This library also ensures reliable parsing of the given speech input If this package is not yet installed on your system use the following:
```bash
python2.7 -m pip install pyaudio
```
If the above install failse with an error:  <b>Failed building wheel for pyaudio</b>.
Then install the following packages and try again, this should solve the issue.
```bash
sudo apt install libasound-dev portaudio19-dev libportaudio2 libportaudiocpp0 ffmpeg libav-tools
python2.7 -m pip install pyaudio
```

The launch file of the main node contains parameters that specity the information that is needed to make python receive input from your microphone.

### speech_recognizer_node

The speech\_recognizer\_node ROS package provides the <i>AudioToText</i> service to the main node

```bash
 # request fields
string[] frames
int64 channels
int64 sample_format
int64 frequency
---
 # response fields
string text
```

This ROS package uses SPEECH_RECOGNITION AND POCKETSPHINX

### parser_node
 
The parser\_node ROS package provides the <i>ParseTextToLogicForm</i> service to the main node.

```bash
 # request fields
string text
---
 # response fields
string parsed
```

This ROS package uses a logical parser developed by Pieter-Jan ...

## Logic-Form-To-Robot-Action

### observer_node

Contains the oberver component which uses the kinect2 camera to detec AR_tags

Need to install AR_MARKERS PACKAGES!!

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


# Install instructions

These instructions will give a detailed overview of the steps that were followed to get the different components of this system installed in case anyone would ever need to reinstall the system.

```
ROS kinetic
Kinect camera
Mico SDK
Kinova ROS
	- MoveIT
	- Gazebo
Distributional Clauses
TF2_ROS
Incorporating ROS and MoveIt	
```

## Prerequisites

What things you need to install the software and how to install them

```
Give examples
```

This should contain the following:
- Version of linux, ros, python, c++,...
- pip freeze overview
- dependency management

### Installing

A step by step series of examples that tell you how to get a development env running

Say what the step will be

```
Give the example
```

And repeat

```
until finished
```

End with an example of getting some data out of the system or using it for a little demo

This should contain:
- Ros install instructions
- Handling dependencies



