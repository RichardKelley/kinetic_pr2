ROS Kinetic PR2
===============

One of the most frustrating things about working with a PR2 is the
need to work with an old Ubuntu distribution and an old ROS
distribution. This collection of packages aims to fix that by making
the PR2 work with Gazebo and Moveit on Ubuntu 16.04.

Installation
------------

This installation assumes your machine has Ubuntu 16.04 and the Kinetic version of ROS already installed. 
If you haven't installed Kinetic, follow this link for install instructions: http://wiki.ros.org/kinetic/Installation/Ubuntu
Make sure to edit your `.bashrc` file after the install! 

Once ROS is installed, clone this repo, navigate to directory and run: 

 ``./install_pr2_kinetic.sh`` 
 
This will install the necessary packages via apt. Then it will download and run the OMPL install script.

Once OMPL is installed, you can initialize your catkin workspace in your location of choice by: 

``mkdir catkin_ws``. 

Navigate back to this repo and copy over the contents of `src/` to `catkin_ws/src/`: 

``cp -r src/ PATH_TO_WORKSPACE/catkin_ws/src/``

Now in the root of your workspace run:

``catkin_make`` 

to build everything. 

From there you can source `/catkin_ws/devel/setup.bash`, simulate the PR2, and run Moveit.
