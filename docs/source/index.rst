.. assignment_2 documentation master file, created by
   sphinx-quickstart on Thu May  8 14:49:20 2025.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to assignment_2's documentation!
========================================


.. toctree::
   :maxdepth: 2
   :caption: Contents:

   modules

Project Overview
================

This package contains several ROS nodes and interfaces for controlling and monitoring
a robot as it navigates to goals. Key components include:

- **ActionClient**: Sends goals to a ROS action server, tracks velocity/position.
- **ServiceNode**: Offers a service to retrieve the last sent goal.


API Reference
=============

.. automodule:: action_client
   :members:
   :undoc-members:
   :show-inheritance:

.. automodule:: Last_target_service
   :members:
   :undoc-members:
   :show-inheritance:


Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

