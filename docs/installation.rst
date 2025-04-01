Installation
============

Prerequisites
-------------

- ROS Noetic (or compatible distribution)
- Gazebo simulator
- RViz visualization tools
- Python 3

Package Setup
-------------

.. code-block:: bash

   # Clone the repository
   cd ~/catkin_ws/src
   git clone https://github.com/ta04lha/assignment_2.git
   cd assignment_2
   git switch main

   # Build the package
   cd ~/catkin_ws
   catkin_make

   # Source the environment
   echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
   source ~/.bashrc

Dependencies
------------

The package requires these ROS packages:

- ``actionlib``
- ``geometry_msgs``
- ``nav_msgs``
- ``std_srvs``
