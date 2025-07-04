/* 
This code takes a csv file that consist of X,Y,Z cartesian trajectory and plans the path with Moveit.
*/

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

// Load CSV file and return a list of poses
std::vector<geometry_msgs::Pose> loadWaypoints(const std::string& file_path) {
    std::vector<geometry_msgs::Pose> waypoints;
    std::ifstream file(file_path);

    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string value;
        std::vector<double> coords;

        while (std::getline(ss, value, ',')) {
            coords.push_back(std::stod(value));
        }

        geometry_msgs::Pose pose;
        pose.position.x = coords[0];
        pose.position.y = coords[1];
        pose.position.z = coords[2];

        // Fixed orientation 
        pose.orientation.x = 0.0;
        pose.orientation.y = 1.0;
        pose.orientation.z = 0.0;
        pose.orientation.w = 0.0;

        waypoints.push_back(pose);
        
    }

    return waypoints;
}

// Main function
int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_controller");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();


    static const std::string PLANNING_GROUP = "panda_arm";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    move_group.setEndEffectorLink("panda_hand_tcp");


    //moveit::planning_interface::MoveGroupInterface move_group("panda_arm");

    // Load CSV waypoints
    std::string path = ros::package::getPath("ir_project") + "/data/heart.csv";
    std::vector<geometry_msgs::Pose> waypoints = loadWaypoints(path);

    // Move to first waypoint
    ROS_INFO("Moving to the starting point.");
    move_group.setPoseTarget(waypoints[0]);
    move_group.move();
    ROS_INFO("Moved to the starting point.");


    // Plan Cartesian path through the rest
    ROS_INFO("Drawing the heart shape.");
    moveit_msgs::RobotTrajectory trajectory;
    const double eef_step = 0.01;
    const double jump_threshold = 0.0;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO("Cartesian path computed (%.2f%% achieved)", fraction*100);


    

    // Execute the trajectorye
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    move_group.execute(plan);
    

    ros::shutdown();
    return 0;
}


