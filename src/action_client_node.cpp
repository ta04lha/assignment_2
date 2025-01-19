#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <nav_msgs/Odometry.h>
#include <assignment_2/PlanningActionGoal.h>
#include <assignment_2/RobotFeedback.h>
#include <iostream>

class ActionClient {
public:
    ActionClient() : client("/reaching_goal", true) {
        pub_position_vel = nh.advertise<assignment_2::RobotFeedback>("/robot_information", 10);
        sub = nh.subscribe("/odom", 10, &ActionClient::odomCallback, this);

        ROS_INFO("Waiting for the action server to be available...");
        client.waitForServer();
    }

    void run() {
        while (ros::ok()) {
            ROS_INFO_ONCE("Enter 's' to set a goal, 'c' to cancel the current goal, 'q' to quit the action client, or 'CTRL+C' to exit all the simulation");
            std::cout << "Command (s=set goal, c=cancel goal, q=quit, CTRL+C=exit all): ";
            char command;
            std::cin >> command;

            switch (command) {
                case 's': {
                    auto [x, y] = getInput();
                    sendGoal(x, y);
                    break;
                }
                case 'c':
                    cancelGoal();
                    break;
                case 'q':
                    ROS_INFO("Exiting the action client");
                    return;
                default:
                    ROS_WARN("Invalid command. Please try again.");
                    break;
            }

            ros::spinOnce();
        }
    }

private:
    ros::NodeHandle nh;
    ros::Publisher pub_position_vel;
    ros::Subscriber sub;
    actionlib::SimpleActionClient<assignment_2::PlanningActionGoal> client;

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        assignment_2::RobotFeedback robotFeedback;
        robotFeedback.x = msg->pose.pose.position.x;
        robotFeedback.y = msg->pose.pose.position.y;
        robotFeedback.vel_x = msg->twist.twist.linear.x;
        robotFeedback.vel_z = msg->twist.twist.angular.z;

        pub_position_vel.publish(RobotFeedback);
    }

    void feedbackCallback(const assignment_2::PlanningFeedbackConstPtr& feedback) {
        if (feedback->stat == "Target reached!") {
            ROS_WARN("Target reached\nActual pose: [%f, %f]\nStatus: %s\n",
                     feedback->actual_pose.position.x,
                     feedback->actual_pose.position.y,
                     feedback->stat.c_str());
            std::cout << "Command (s=set goal, c=cancel goal, q=quit): ";
        }
    }

    void sendGoal(double x, double y) {
        assignment_2::PlanningGoal goal;
        goal.target_pose.pose.position.x = x;
        goal.target_pose.pose.position.y = y;

        goal.target_pose.pose.position.z = 0.0;
        goal.target_pose.pose.orientation.x = 0.0;
        goal.target_pose.pose.orientation.y = 0.0;
        goal.target_pose.pose.orientation.z = 0.0;
        goal.target_pose.pose.orientation.w = 0.0;

        client.sendGoal(goal, boost::bind(&ActionClient::feedbackCallback, this, _1));
        ROS_INFO("Goal sent");
    }

    void cancelGoal() {
        actionlib::SimpleClientGoalState state = client.getState();
        if (state == actionlib::SimpleClientGoalState::ACTIVE || state == actionlib::SimpleClientGoalState::PENDING) {
            ROS_INFO("Cancelling current goal");
            client.cancelGoal();
            ros::Duration(0.5).sleep();

            state = client.getState();
            if (state == actionlib::SimpleClientGoalState::PREEMPTED || state == actionlib::SimpleClientGoalState::RECALLED) {
                ROS_INFO("Goal successfully cancelled");
            } else {
                ROS_WARN("Failed to cancel the goal");
            }
        } else {
            ROS_WARN("No active goal to cancel.");
        }
    }

    std::pair<double, double> getInput() {
        double x, y;
        while (true) {
            try {
                std::cout << "Enter the value for setting the x coordinate: ";
                std::cin >> x;
                std::cout << "Enter the value for setting the y coordinate: ";
                std::cin >> y;
                return {x, y};
            } catch (std::exception& e) {
                ROS_WARN("Input not valid, enter only numbers!");
            }
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "action_client");

    ActionClient action_client;
    action_client.run();
    
    
    
    

    return 0;
}

