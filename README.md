# Jetson-Orin-Nano-6.2-with-realsense-D435

Hello, 
here is a repository for setting up a Realsense Camera {#D435} with Nvidia Jetson Orin Nano 6.2. 

This repository features commands to control Camera actions like Record, Pause, Playback, Save in bag format.

These saved files can be modified using ROSBAG tools.

Record.py has similar functionality to running "rs-record" as in the Realsense official commands which are In C++ but Record.py is a python version of the code which can be saved on local storage and be modified to add any timestamps, headers, etc which the Camera records enabling modification.
