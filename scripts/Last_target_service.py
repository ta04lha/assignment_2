#! /usr/bin/env python3

import rospy
from assignment_2_2024.msg import PlanningActionGoal
from assignment_2.srv import LastTarget, LastTargetResponse
from geometry_msgs.msg import PoseStamped

class ServiceNode:
    def __init__(self):
        # Initialize subscriber and service
        self.last_target = None
        self.sub = rospy.Subscriber("/reaching_goal/goal", PlanningActionGoal, self.goal_callback)
        self.service = rospy.Service("/get_last_goal", LastTarget, self.handle_last_goal_request)

        rospy.loginfo("Service node started. Waiting for requests...")

    def goal_callback(self, msg):
        """Callback to save the last set goal."""
        self.last_target = msg
        rospy.loginfo("Received a new goal.")

    def handle_last_goal_request(self, req):
        """Service handler to return the last set goal."""
        if not self.last_target:
            rospy.logwarn("[SERVICE NODE] No target available")
            return LastTargetResponse()

        response = LastTargetResponse()
        response.last_target = self.last_target.goal.target_pose
        return response

if __name__ == "__main__":
    rospy.init_node("last_target_service")

    service_node = ServiceNode()
    rospy.spin()

