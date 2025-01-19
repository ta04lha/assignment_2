#include <ros/ros.h>
#include <assignment_2/PlanningActionGoal.h>
#include <assignment_2/LastTarget.h>

class ServiceNode {
public:
    ServiceNode() {
        // Initialize subscriber and service
        sub = nh.subscribe("/reaching_goal/goal", 10, &ServiceNode::goalCallback, this);
        service = nh.advertiseService("/get_last_goal", &ServiceNode::handleLastGoalRequest, this);

        ROS_INFO("Service node started. Waiting for requests...");
    }

    void spin() {
        ros::spin();
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::ServiceServer service;
    assignment_2::PlanningActionGoal::ConstPtr last_target; // Store the last target goal

    // Callback to save the last set goal
    void goalCallback(const assignment_2::PlanningActionGoal::ConstPtr& msg) {
        last_target = msg;
        ROS_INFO("Received a new goal.");
    }

    // Service handler to return the last set goal
    bool handleLastGoalRequest(assignment_2::LastTarget::Request& req, assignment_2::LastTarget::Response& res) {
        if (!last_target) {
            ROS_WARN("[SERVICE NODE] No target available");
            return true;
        }

        res.target_pose = last_target->goal.target_pose;
        return true;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "last_target_srvice");

    ServiceNode service_node;
    service_node.spin();

    return 0;
}

