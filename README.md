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

