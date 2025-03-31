.. Assignment 2 documentation master file, created by
   sphinx-quickstart on Sat Mar 29 15:40:26 2025.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to Assignment 2's documentation!
========================================

.. toctree::
   :maxdepth: 2
   :caption: Contents:
   
   
   installation
   usage
   modules/action_client
   modules/last_target_service
   api
   
   



Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

Action Client Module
====================

.. automodule:: action_client
   :members:
   :undoc-members:
   :show-inheritance:

Class Documentation
-------------------

.. autoclass:: ActionClient
   :members:
   :special-members: __init__

   .. automethod:: run
   .. automethod:: odom_callback
   .. automethod:: feedback_callback
   .. automethod:: send_goal
   .. automethod:: cancel_goal
   .. automethod:: get_input
   
Last Target Service Module
==========================

.. automodule:: Last_target_service
   :members:
   :undoc-members:
   :show-inheritance:

Class Documentation
-------------------

.. autoclass:: ServiceNode
   :members:
   :special-members: __init__

   .. automethod:: goal_callback
   .. automethod:: handle_last_goal_request
   
   
Installation
============

Prerequisites
-------------
- ROS Noetic (or your ROS version)
- Python 3
- Required ROS packages: ``nav_msgs``, ``actionlib_msgs``

Installation Steps
------------------

1. Clone the repository:

   .. code-block:: bash

      git clone <https://github.com/ta04lha/assignment_2.git>
      cd <assignment_2>
      git switch main

2. Build the workspace:

   .. code-block:: bash

      catkin_make
      source devel/setup.bash

3. Ensure the Python files are executable:

   .. code-block:: bash

      chmod +x src/assignment_2/scripts/action_client.py
      chmod +x src/assignment_2/scripts/Last_target_service.py
      
      
Usage
=======

Running the Action Client
--------------------------

1. Start ROS core:

   .. code-block:: bash

      roscore

2. Run the action client:

   .. code-block:: bash

      rosrun assignment_2_2024 action_client.py

3. Available commands:
   - 's': Set a new goal (you'll be prompted for x,y coordinates)
   - 'c': Cancel the current goal
   - 'q': Quit the action client

Running the Last Target Service
--------------------------------

1. Start the service node:

   .. code-block:: bash

      rosrun assignment_2_2024 Last_target_service.py

2. Call the service to get the last target:

   .. code-block:: bash

      rosservice call /get_last_goal
      
      

