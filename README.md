# Applications for Mico Robot Arm

This repository contains ROS packages that are needed to operate the Mico robot arm developed by Kinova.  The implemented system can be used as a base for extension and implementation of functionalities for the robot arm.

This README contains an overview of the recommended skills paired with useful tutorials/wiki-pages/github repo's that can prove useful to read before working with the nodes in this repository. Subsequently an overview of the implemented system is given and the functionalities of this system are discussed. 

Before doing any further work on this project, it is recommended to have a look at the [Recommended Skills](#recommended-skills) section. It is also advisable that any further work done on this repository is implemented and tested on the desktop that already contains this project instead of reinstalling the project on another computer. However, to facilitate reinstalation, a seperate markdown file is included that addresses the different components that have to be installed in case a [full (re)install](INSTALL.md) is needed.

## Recommended Skills

## Overview
<p align="center">
	<img src="images/Overview_ROS.png?raw=true" alt="Overview ROS"/>
</p>

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Prerequisites

What things you need to install the software and how to install them

```
Give examples
```

This should contain the following:
- Version of linux, ros, python, c++,...
- pip freeze overview
- dependency management

## Deployment
The src directory contains the following:


main_node
-------
This ROS package contains the main node that will be running constantly while the system is being executed, it will direct messages and issue service requests to other components.

parser_node
----------- 
This Java package contains the text-to-logic-form parsing component which will tranform text into logic form understandable by the robot arm.

speech_node
-----------
Contains the speech recording component which (for now) accepts spoken and typed input.

recognizer_node
---------------
Contains the component that will transform the audio input into a textual representation.



Executing catkin_make in the root of this workspace will build the project


## Built With

* [ROS](http://www.ros.org/) - The Robot operating system
* [Catkin](https://www.wiki.ros.org/catkin) - Dependency Management and building nodes

## Authors

* **Shani Vanlerberghe** - *Deploying speech to logic for in ROS*
* **Pieter-Jan Coenen** - *The text to logic form parser*

## License

## Acknowledgments

