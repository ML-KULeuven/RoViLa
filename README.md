# Project Title

One Paragraph of project description goes here

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

### Installing

A step by step series of examples that tell you how to get a development env running

Say what the step will be

```
Give the example
```

Atnd repeat

```
until finished
```

End with an example of getting some data out of the system or using it for a little demo

This should contain:
- Ros install instructions
- Handling dependencies

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

