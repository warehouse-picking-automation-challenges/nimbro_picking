
NimbRo Picking APC 2016 Release
===============================

This repository contains the source code used by Team NimbRo Picking
(University of Bonn) in the Amazon Picking Challenge 2016.

Be aware that most of the code is released as-is. That means that this release
mostly displays the state right after the competition.

Publications
------------

For more details about our system, please refer to the following publications:

Max Schwarz, Anton Milan, Arul Selvam Periyasamy, and Sven Behnke:  
*RGB-D Object Detection and Semantic Segmentation for Autonomous Manipulation in Clutter*  
Accepted for International Journal of Robotics Research (IJRR), Sage Publications, to apprear 2017.  
Available at: http://ais.uni-bonn.de/papers/IJRR_2017_Schwarz.pdf

Max Schwarz, Anton Milan, Christian Lenz, Aura Munoz, Arul Selvam Periyasamy, Michael Schreiber, Sebastian Schüller, and Sven Behnke:  
*NimbRo Picking: Versatile Part Handling for Warehouse Automation*  
Accepted for IEEE International Conference on Robotics and Automation (ICRA), Singapore, May 2017.  
Available at: http://ais.uni-bonn.de/papers/ICRA_2017_Schwarz.pdf

Dependencies
------------

The codebase has been developed and tested under Ubuntu 14.04 and ROS Indigo.
For successful compilation, the following Debian packages are required:

```bash
sudo apt-get install ros-indigo-desktop-full ros-indigo-joint-limits-interface \
    ros-indigo-moveit-core ros-indigo-moveit-ros ros-indigo-gazebo-ros-control \
    ros-indigo-joy ros-indigo-ros-controllers gcc-avr avr-libc \
    ros-indigo-universal-robot python-trollius \
    ros-indigo-rqt-gui-cpp ros-indigo-hardware-interface \
    ros-indigo-gazebo-ros \
    libncurses5-dev \
    python-catkin-tools
```

Additionally, the following repositories should be checked out into your catkin
workspace:

 * https://github.com/AIS-Bonn/catch_ros
 * https://github.com/xqms/rosmon

Initial compilation:

```bash
# Sources should lie under src/
. /opt/ros/indigo/setup.bash
catkin init
catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_CXX_FLAGS=-Wall
catkin build
```

Please use `mon launch` instead of `roslaunch` to start launch files, since
we developed and tested our launch files with `rosmon`.

Demo of the motion interpolation and inverse kinematics:
```bash
shell1# mon launch apc_model gazebo.launch # press play after the robot appears
shell2# mon launch apc_launch simulation.launch
shell3# mon launch apc_launch keyframe_editor.launch

# Select "Teleoperation" and "Cartesian" under "arm_with_eef". You can now drag
# the endeffector around.
```

Subdirectories
--------------

```
APC components:
 apc_object_perception: all ROS packages related to the perception pipeline
 nimbro_apc: All other APC-related packages

Other reusable modules:
 depth_filler: Methods for RGB-guided depth inpainting
 dynalib: ROBOTIS Dynamixel servo communication
 nimbro_actuators: Communication with microcontrollers
 nimbro_config_server: Parameter server with live update and tuning GUI
 nimbro_fsm: Simple finite state machine framework
 nimbro_keyframe_server: Keyframe interpolation
 nimbro_realsense: Stereo Intel SR300 driver
 nimbro_robotcontrol: Core hardware controller (here only used for simulation)
 nimbro_vis: Visualization utilities
```

License
-------

Copyright (c) 2017, University of Bonn, Computer Science Institute VI,
Autonomous Intelligent Systems.

The code in this repository is released under the BSD license (see LICENSE),
with the exception of the code in apc_object_perception/apc_densecap/densecap,
which is released under MIT.

Please cite the above publications if you use code from this repository.

Authors
-------

 * Christian Lenz
 * Anton Milan
 * Aura Muñoz
 * Arul Selvam Periyasamy
 * Michael Schreiber
 * Sebastian Schüller
 * Max Schwarz

Contact
-------

In case of any questions, please contact:

 * Max Schwarz (max.schwarz@ais.uni-bonn.de)
 * Christian Lenz (christian.lenz@ais.uni-bonn.de)
 

[//]: # (kate: replace-trailing-space-save false)
