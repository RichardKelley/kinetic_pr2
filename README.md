ROS Kinetic PR2
===============

One of the most frustrating things about working with a PR2 is the
need to work with an old Ubuntu distribution and an old ROS
distribution. This collection of packages aims to fix that by making
the PR2 work with Gazebo and Moveit on Ubuntu 16.04.

Installation
------------

You'll need to run the `install_pr2_kinetic.sh` script. This will
install the necessary packages via APT. Then it will download and run
the OMPL install script.

Once OMPL is installed, you can initialize your catkin workspace and
run `catkin_make` to build everything. From there you can source
`devel/setup.bash`, simulate the PR2, and run Moveit.