# Assignment 2 (Part I) - Action Client and Service Node for Robot Navigation

## Overview
This project involves creating a ROS package, **`assignment_2`**, that implements two key components for robot navigation by setting a goal to reach:

1. **Action Client Node (node1):**
   - Allows the user to set or cancel a goal.
   - Publishes custom messages (`Robot_info`) on the `/robot_information` topic with the following fields:
     - `float64 x`  
     - `float64 y`  
     - `float64 vel_x`  
     - `float64 vel_z`

2. **Service Node (node2):**
   - Returns the coordinates of the last goal set by the user through the action client node.

---

## Nodes Description

### 1. **Action Client Node**
- **Functionality:**
  - Allows interaction via a terminal to:
    - Set a new goal (`s` key).
    - Cancel an active goal (`c` key).
    - Exit the program (`q` key).
  - After setting a goal, the robot's current position and goal status will be printed upon reaching the goal (`Goal reached!`).
  - Publishes the robot's position and velocity (from `/odom` topic) as a custom message on the `/robot_information` topic.

- **User Interaction:**
  - Press `s` to set a goal (you'll also need to input `x` and `y` coordinates).
  - Press `c` to cancel an active goal.
  - Press `q` to exit.

### 2. **Service Node**
- **Functionality:**
  - Provides a service to retrieve the last goal set.
  - No request input is needed; the service returns a response of type `geometry_msgs/PoseStamped` containing the last goal.

### 3. **Launch File**
- The `launch_file.launch` file:
  - Initializes the action client, service node, and other simulation nodes in the package.
  - Opens a new GNOME terminal for the action client, making user interaction easier.

---

## Installation

### Prerequisites
- Installed ROS.
- Configured ROS workspace (`catkin_ws`).
- Installed Gazebo and RViz.

### 1. Clone the Package
```bash
cd ~/catkin_ws/src
git clone https://github.com/ta04lha/assignment_2.git
cd assignment_2
git switch main
```

### 2. Compile the Package
```bash
cd ~/catkin_ws
catkin_make
```
Ensure there are no errors during compilation.

### 3. Source the Environment
Add the following line to your `~/.bashrc`:
```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```
Apply the changes:
```bash
source ~/.bashrc
```

---

## Running the Package

### 1. Start the Simulation
Run the launch file:
```bash
roslaunch assignment_2 launch_file.launch
```
This opens a second terminal for interaction with the action client node.

### 2. (Optional) Interact with the Service Node
To retrieve the last goal sent to the action server:
```bash
rosservice call /get_last_goal
```
The service returns a `geometry_msgs/PoseStamped` message with the last goal.

---

## Interacting with the Nodes

### Action Client Node
Manage everything in the terminal where the launch file is running:
- **Set Goal:** Press `s` and enter `x` and `y` coordinates.
- **Cancel Goal:** Press `c`.
- **Exit Program:** Press `q`.

The action client will continuously listen for user input while printing status messages.

### Service Node
Open a new terminal and call the service:
```bash
rosservice call /get_last_goal
```
This will output the last set goal.

---

