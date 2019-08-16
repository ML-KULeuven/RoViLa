# FULL REINSTALLATION INSTRUCTIONS

The steps described in this document are the steps I did when installing everything and making everything work together. These instructions can be followed as a guideline when installing the repository or when upgrading to newer versions.

## INSTALLING ROS-KINETIC
--> Ros-kinetic is the version of ROS for Ubuntu 16.04
If migration is needed then ROS-Melodic should be installed but I cannot guarantee that this will work out of the box
steps:

* Setup the list of package sources that computer accepts software from packages.ros.org
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

* Correctly setup keys to be able to connect to the keyserver to be able to verify that packages are downloaded from trusted sources:
```bash
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

* Update the package index and install the full desktop version of ROS kinetic:
```bash
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full
```

* Initialize rosdep , a tool used to be able to easily install system depedencies needed for the source that you want to compile:
```bash
sudo rosdep init
rosdep update
```

* Add the setup.bash file to the .bashrc script (which is executed every time a new terminal is opened) and source the script. This will enable the kinetic environment each time a new terminal is opened:
```bash
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

* Install dependencies for building packages with python:
```bash
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
```

## INSTALLING KINECT2
The following instructions will enable incorporation of the kinect2 camera in the ROS system. First <i>libfreenect</i> needs to be installed, this is the driver for the kinect2 camera. Subsequently iai_kinect2 needs to be installed. This is a collection of tools and libraries forms a ROS interface to the kinect2 camera. Enabling users to gather video observations from the camera from inside ROS.

### libfreenect2
* Download the libfreenect2 source code from the github repository:
```bash
git clone https://github.com/OpenKinect/libfreenect2.git
cd libfreenect2
```

* Install some necessary build-tools:
```bash
sudo apt-get install build-essential cmake pkg-config
```

* Install libusb, at least version 1.0.20 or higher:
```bash
sudo apt-get install libusb-1.0-0-dev
```

* Install TurboJPEG
```bash
sudo apt-get install libturbojpeg libjpeg-turbo8-dev
```

* Installing OpenGL if possible so that acceleration can be used:
	* First off, check whether the OpenGL version 3.1 is installed on your system:
		* Install mesa-utils to be able to use glxinfo to request OpenGL information:
		```
		sudo apt-get install mesa-utils
		```
	* Then request the OpenGL version installed on your system with:
 	```
	glxinfo | grep "OpenGL version"
	```
		
	The output will show the OpenGL version string which needs to be over 3.1 for you to be able to use OpenGL with kinect2. If it is lower than 3.1 kinect2 can still be used though but skip the next step.
		
	* If the OpenGL version is 3.1 or higher then install libglfw3-dev:
		```
		sudo apt-get install libglfw3-dev
		```
		
* Build the repository in the libfreenect2 root directory:
!! If your were unable to install OpenGL (because version was too low or other reasons) include <b>-DENABLE\_OPENGL=OFF</b> in the cmake variables.
```bash
mkdir build && cd build
cmake .. -DCMAKE\_INSTALL\_PREFIX=$HOME/freenect2 -DENABLE\_CXX11=ON
make
make install
```

Explanation of the cmake variables:
-- DCMAKE\_INSTALL\_PREFIX=$HOME/freenect2  --> make and make install will install the needed librariesfor the kinect camera in the freenect2 directory in the HOME directory. This way, iai_kinect2 will find it by default.
-- DENABLE\_CXX11=ON  --> iai_kinect2 need C++11 to work, this variable enables that.

* Set up udev rules to be able to have access to the kinect2 camera:
```bash
sudo cp ../platform/linux/udev/90-kinect2.rules /etc/udex/rules.d/
```

* Replug the Kinect2 camera and run the test program from inside the libfreenect2 build directory:
```bash
./bin/Protonect
```

If everything went well you should see a window showing 4 camera view, like this:
<figure align="center">
	<img src="images/Protonect.png?raw=true" alt="Overview ROS"/>
</figure>

### iai_kinect2
This package is included as a submodule in this repository and should therefore not be installed seperately. Once you have finished the rest of these instructions, read the instructions under [Cloning this repository](README.md/#cloning-this-repository)

## CONFIGURING MICO ARM
The Mico arm is a robot arm developed by Kinova. To be able to send commands to the robot arm from ROS the firmware on the robot has to be up to date. The versions originally used in this projects will be included here as well as instructions on how to download and install both the SDK and the firmware.

Plug in the robot and attach usb to pc before going forward!!

### Mico SDK
The Mico SDK can be downloaded from the following google drive link:
[Mico SDK](https://drive.google.com/file/d/1RzvwinEiJbrmEMaIkKzHu4qwZ4JjYGXC/view)

* Unpack the zip file:
```bash
unzip Kinova_SDK_MICO_1_5_1.zip -d Kinova_SDK
```

* Execute the 64 bit installer in the ubuntu directory to install the SDK
```
./Kinova_SDK/Ubuntu/16_04/64\ bits/installSDK64.sh
```

--> This will install the preliminaries and the opens an installer to guide the installation of the actual development center.

### Mico Firmware
The firmware for the Mico robot can be downloaded from the following google drive link:
[Mico Firmware](https://drive.google.com/file/d/1DHH9zGHirwnICzReW6tGHddEfr7OUTor/view)

* Unpack the zip file containing the firmware:
```bash
unzip Kinova_Firmware_MICO_6.2.5.zip -d Kinova_Firmware
```

* Open the Development center APP:
```
cd /opt/kinova/GUI/
./developmentcenter.sh
```

* Go to General Settings -> Update
* Enter the serial number of the Robot -> PJ00650019150340001 (for the MICO arm in question)
* Upload the <i>FS 0CPP 0008_6.2.5_mico_6dof.hex</i> file that is in the unzipped Kinova\_Firmware directory. 

* Restart the robot, you should be able to work with the robot through the development center now as well as with the joystick.

## INSTALLING KINOVA ROS
Before being able to build the kinova-ros repository, Gazebo and Moveit need to be installed.

### Installing Gazebo for Kinova robots
* Install gazebo for ROS:
```bash
sudo apt-get install ros-kinetic-gazebo-ros*
```

* Install the ros_control and ros_controllers repositories
```bash
sudo apt-get install ros-kinetic-gazebo-ros-control
sudo apt-get install ros-kinetic-ros-controllers*
sudo apt-get install ros-kinetic-trac-ik-kinematics-plugin
```

### Installing MoveIt! for Kinova robots
* Install MoveIt! for ros:
```bash
sudo apt-get install ros-kinetic-moveit
```

* Install Trac_IK, an Inverse Kinematics solver:
```bash
sudo apt-get install ros-indigo-trac-ik
```

### Installing kinova-ros
Like iai_kinect2, the kinova-ros repository is included as a submodule in this project. Therefore it should not be installed seperatly. Once you have finished the other instructions in this file, read the instructions under [Cloning this repository](README.md/#cloning-this-repository).


## CONFIGURING MICO ARM MOVEIT CONFIGURATION
This repository includes a moveit configuration for the MICO robot arm m1n6s200. This configuration is stored under the dependencies folder: [m1n6s200_moveit_config](dependencies/). 

After cloning this repository this file should be copied to the robot_configurations in the kinova-ros package:
```bash
cd ~/RoViLa
cp -r depencencies/m1n6s200_moveit_config/ src/kinova-ros/kinova_moveit/robot_configs/
```

