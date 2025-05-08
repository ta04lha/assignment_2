#!/usr/bin/env python3

import rospy
from assignment_2_2024.msg import PlanningActionGoal
from assignment_2.srv import LastTarget, LastTargetResponse
from geometry_msgs.msg import PoseStamped


class ServiceNode:
    """
    A ROS node that provides a service to return the last goal sent to the robot.

    This node subscribes to the `/reaching_goal/goal` topic to monitor the latest goal
    and exposes a service `/get_last_goal` to provide that information on request.

    Attributes:
        last_target (PlanningActionGoal): Stores the most recently received goal.
        sub (rospy.Subscriber): Subscribes to the action goal topic.
        service (rospy.Service): Provides a service to return the last goal.
    """

    def __init__(self):
        """
        Initializes the service node, subscriber, and service server.
        """
        self.last_target = None
        self.sub = rospy.Subscriber("/reaching_goal/goal", PlanningActionGoal, self.goal_callback)
        self.service = rospy.Service("/get_last_goal", LastTarget, self.handle_last_goal_request)

        rospy.loginfo("Service node started. Waiting for requests...")

    def goal_callback(self, msg):
        """
        Callback function that stores the most recently received goal.

        Args:
            msg (PlanningActionGoal): The message containing the new goal.
        """
        self.last_target = msg
        rospy.loginfo("Received a new goal.")

    def handle_last_goal_request(self, req):
        """
        Service handler to return the last received goal.

        Args:
            req (LastTargetRequest): The service request (empty).

        Returns:
            LastTargetResponse: Contains the last goal as a PoseStamped, if available.
        """
        if not self.last_target:
            rospy.logwarn("[SERVICE NODE] No target available")
            return LastTargetResponse()

        response = LastTargetResponse()
        response.last_target = self.last_target.goal.target_pose
        return response


if __name__ == "__main__":
    """
    Initializes the ROS node and starts the service node.
    """
    rospy.init_node("last_target_service")
    service_node = ServiceNode()
    rospy.spin()

