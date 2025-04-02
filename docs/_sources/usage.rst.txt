Usage Guide
===========

Starting the System
-------------------

.. code-block:: bash

   roslaunch assignment_2 launch_file.launch

This will:
1. Start the Gazebo simulation
2. Launch RViz visualization
3. Open an interactive terminal for the action client

Action Client Commands
----------------------

+----------+-----------------------------------------------+
| Command  | Description                                   |
+==========+===============================================+
| ``s``    | Set a new goal (prompts for x,y coordinates) |
+----------+-----------------------------------------------+
| ``c``    | Cancel current goal                           |
+----------+-----------------------------------------------+
| ``q``    | Quit the action client                        |
+----------+-----------------------------------------------+

Service Interaction
-------------------

To get the last goal:

.. code-block:: bash

   rosservice call /get_last_goal
