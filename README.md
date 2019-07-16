The ReadMe for this project:


Main_ws
-------
The workspace containing the main node that will be running constantly while the system is being executed, it will direct messages and issue service requests to other components.

Parser_ws
--------- 
The workspace containing the text-to-logic-form parsing component which will tranform text into logic form understandable by the robot arm.

Speech_ws
---------
Contains the speech recording component which (for now) accepts spoken and typed input.

Recognizer_ws
-------------
Contains the component that will transform the audio input into a textual representation.
