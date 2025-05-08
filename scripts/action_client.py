#!/usr/bin/env python3

import rospy
import actionlib
from nav_msgs.msg import Odometry
from assignment_2_2024.msg import PlanningAction
from assignment_2.msg import RobotFeedback
from assignment_2_2024.msg import PlanningGoal, PlanningFeedback
from actionlib_msgs.msg import GoalStatus
from std_srvs.srv import Trigger, TriggerResponse


class ActionClient:
    """
    ROS Action Client for sending goals to a navigation server.

    This client allows setting new goals, canceling current goals, and 
    provides continuous feedback about the robot's position and velocity.

    Attributes:
        client (SimpleActionClient): Action client for the '/reaching_goal' action.
        pub_position_vel (Publisher): Publishes robot position and velocity in km/h.
        sub (Subscriber): Subscribes to odometry data.
        goals_reached (int): Counter for reached goals.
        goals_cancelled (int): Counter for cancelled goals.
    """

    def __init__(self):
        """
        Initializes the ActionClient by setting up the action client, 
        publishers, subscribers, and services.
        """
        self.client = actionlib.SimpleActionClient("/reaching_goal", PlanningAction)
        self.pub_position_vel = rospy.Publisher("/robot_information", RobotFeedback, queue_size=10)
        self.goal_service = rospy.Service("/goal_statistics", Trigger, self.goal_statistics_callback)
        self.sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)

        self.goals_reached = 0
        self.goals_cancelled = 0

        rospy.loginfo("Waiting for the action server to be available...")
        self.client.wait_for_server()
        rospy.loginfo("Action server is ready!")

    def run(self):
        """
        Main loop that listens for user input to send, cancel, or quit goals.
        """
        while not rospy.is_shutdown():
            rospy.loginfo_once("Enter 's' to set a goal, 'c' to cancel the current goal, 'q' to quit the action client, or 'CTRL+C' to exit all the simulation")
            command = input("Command (s=set goal, c=cancel goal, q=quit, CTRL+C=exit all): ")

            if command == 's':
                x, y = self.get_input()
                self.send_goal(x, y)
            elif command == 'c':
                self.cancel_goal()
            elif command == 'q':
                rospy.loginfo("Exiting the action client")
                return
            else:
                rospy.logwarn("Invalid command. Please try again.")

    def odom_callback(self, msg):
        """
        Callback function for the /odom topic.

        Publishes the robot's current position and velocity in km/h.

        Args:
            msg (Odometry): Message containing odometry data.
        """
        robot_feedback = RobotFeedback()
        robot_feedback.x = msg.pose.pose.position.x
        robot_feedback.y = msg.pose.pose.position.y
        robot_feedback.vel_x = msg.twist.twist.linear.x * 3.6  # Convert m/s to km/h
        robot_feedback.vel_y = msg.twist.twist.linear.y * 3.6  # Convert m/s to km/h
        robot_feedback.vel_z = msg.twist.twist.angular.z

        self.pub_position_vel.publish(robot_feedback)

    def feedback_callback(self, feedback):
        """
        Callback for receiving feedback during goal execution.

        Args:
            feedback (PlanningFeedback): Feedback message from the action server.
        """
        rospy.loginfo("Feedback received: [%f, %f], Status: %s",
                      feedback.actual_pose.position.x,
                      feedback.actual_pose.position.y,
                      feedback.stat)

    def send_goal(self, x, y):
        """
        Sends a new goal to the action server.

        Args:
            x (float): X coordinate of the goal.
            y (float): Y coordinate of the goal.
        """
        goal = PlanningGoal()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0

        self.client.send_goal(goal, done_cb=self.goal_done_callback, feedback_cb=self.feedback_callback)
        rospy.loginfo("Goal sent to [%f, %f]", x, y)

    def goal_done_callback(self, state, result):
        """
        Callback function triggered when a goal completes.

        Args:
            state (int): Final state of the goal.
            result: Result message (not used here).
        """
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal reached successfully!")
            self.goals_reached += 1
        elif state in [GoalStatus.PREEMPTED, GoalStatus.RECALLED]:
            rospy.loginfo("Goal was cancelled.")
            self.goals_cancelled += 1

    def cancel_goal(self):
        """
        Cancels the currently active goal if it exists.
        """
        state = self.client.get_state()
        rospy.loginfo("Current goal state: %d", state)

        if state in [GoalStatus.ACTIVE, GoalStatus.PENDING]:
            rospy.loginfo("Cancelling the current goal...")
            self.client.cancel_goal()
            rospy.sleep(1.0)

            new_state = self.client.get_state()
            if new_state in [GoalStatus.PREEMPTED, GoalStatus.RECALLED]:
                rospy.loginfo("Goal successfully cancelled")
                self.goals_cancelled += 1
            else:
                rospy.logwarn("Failed to cancel the goal. Current state: %d", new_state)
        else:
            rospy.logwarn("No active goal to cancel.")

    def goal_statistics_callback(self, request):
        """
        Service callback to return statistics about reached and cancelled goals.

        Args:
            request (TriggerRequest): Empty request.

        Returns:
            TriggerResponse: Contains success status and message.
        """
        response = TriggerResponse()
        response.success = True
        response.message = f"Goals Reached: {self.goals_reached}, Goals Cancelled: {self.goals_cancelled}"
        return response

    def get_input(self):
        """
        Prompts the user to input goal coordinates.

        Returns:
            tuple: (x, y) coordinates as floats.
        """
        while True:
            try:
                x = float(input("Enter the x coordinate: "))
                y = float(input("Enter the y coordinate: "))
                return x, y
            except ValueError:
                rospy.logwarn("Invalid input, enter only numbers!")


if __name__ == "__main__":
    rospy.init_node("action_client")
    action_client = ActionClient()
    action_client.run()

