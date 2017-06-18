sudo apt-get install libfcl0.5 libfcl-0.5-dev scons libbson-dev libglew-dev libglew1.13 ros-kinetic-octomap ros-kinetic-octomap-mapping ros-kinetic-octomap-msgs ros-kinetic-octomap-ros ros-kinetic-octomap-rviz-plugins ros-kinetic-octomap-server ros-kinetic-geometric-shapes ros-kinetic-warehouse-ros ros-kinetic-srdfdom ros-kinetic-object-recognition-msgs ros-kinetic-navigation ros-kinetic-ivcon ros-kinetic-convex-decomposition

# Install OMPL from source and place symbolic links
wget http://ompl.kavrakilab.org/install-ompl-ubuntu.sh
chmod u+x install-ompl-ubuntu.sh
./install-ompl-ubuntu.sh
sudo ln -s /usr/local/include/ompl /opt/ros/kinetic/include/ompl
sudo mkdir -p /opt/ros/kinetic/lib/x86_64-linux-gnu
sudo ln -s /usr/local/lib/libompl.so /opt/ros/kinetic/lib/x86_64-linux-gnu/libompl.so
