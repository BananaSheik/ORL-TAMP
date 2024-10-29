#include <memory>
#include <cmath>
#include <random>
#include <iostream>
#include <vector>
#include <fstream> // Add this header for file operations
#include <filesystem> // For filesystem operations (C++17)
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

namespace fs = std::filesystem;

// Function to generate a random double between min and max
double random_double(double min, double max) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(min, max);
    return dis(gen);
}

// Function to generate a random orientation quaternion
geometry_msgs::msg::Quaternion random_orientation() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-1.0, 1.0);

    // Generate a random quaternion
    geometry_msgs::msg::Quaternion orientation;
    orientation.w = dis(gen);
    orientation.x = dis(gen);
    orientation.y = dis(gen);
    orientation.z = dis(gen);

    // Normalize the quaternion manually
    double norm = std::sqrt(orientation.w * orientation.w +
                            orientation.x * orientation.x +
                            orientation.y * orientation.y +
                            orientation.z * orientation.z);
    orientation.w /= norm;
    orientation.x /= norm;
    orientation.y /= norm;
    orientation.z /= norm;

    return orientation;
}

// Function to calculate Euclidean distance between two poses
double pose_distance(const geometry_msgs::msg::Pose& pose1, const geometry_msgs::msg::Pose& pose2) {
    double dx = pose1.position.x - pose2.position.x;
    double dy = pose1.position.y - pose2.position.y;
    double dz = pose1.position.z - pose2.position.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// Function to check if a pose is valid
bool isValidPose(moveit::planning_interface::MoveGroupInterface& move_group_interface, const geometry_msgs::msg::Pose& pose) {
    move_group_interface.setPoseTarget(pose);

    // Create a plan object
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    // Attempt to plan to the target pose
    bool success = (move_group_interface.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success) {
        RCLCPP_ERROR(rclcpp::get_logger("hello_moveit"), "Failed to plan to target pose!");
    }

    return success;
}

int main(int argc, char * argv[])
{
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "hello_moveit",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    // Create a ROS logger
    auto logger = rclcpp::get_logger("hello_moveit");

    // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    MoveGroupInterface move_group_interface(node, "panda_arm");

    // Set planner parameters
    move_group_interface.setPlanningTime(2.0); // Set planning time to 2 seconds
    move_group_interface.setMaxVelocityScalingFactor(1.0); // Maximize velocity
    move_group_interface.setMaxAccelerationScalingFactor(1.0); // Maximize acceleration

    // Number of random poses to generate
    int num_poses = 200; // Change this as needed
    int target_number = 0;
    double dist_threshold = 0.05;

    // Vector to store valid target poses
    std::vector<geometry_msgs::msg::Pose> valid_poses;

    // Generate random target poses and store valid ones
    while (valid_poses.size() < static_cast<size_t>(num_poses)) {
        geometry_msgs::msg::Pose target_pose;
        double r = random_double(0.2, 0.7);
        double theta = random_double(0.0, 2.0 * M_PI);
        target_pose.position.x = r * std::cos(theta);
        target_pose.position.y = r * std::sin(theta);

        // Calculate z based on the inverse of distance from origin
        double distance_to_origin = std::sqrt(target_pose.position.x * target_pose.position.x +
                                              target_pose.position.y * target_pose.position.y);
        double max_radius = 0.7; // Maximum distance for z = 0.1
        double min_radius = 0.2; // Minimum distance for z = 0.9
        double z_range = 0.8;    // Range of z values

        // Calculate z based on distance to origin
        target_pose.position.z = 0.2;
        
        // Generate random orientation quaternion
        target_pose.orientation = random_orientation();

        // Check if the pose is valid and meets distance threshold from other poses
        bool pose_valid = isValidPose(move_group_interface, target_pose);
        bool meets_distance_threshold = true;
        for (const auto& pose : valid_poses) {
            if (pose_distance(target_pose, pose) < dist_threshold) {
                meets_distance_threshold = false;
                break;
            }
        }

        if (pose_valid && meets_distance_threshold) {
            target_number +=1 ;
            valid_poses.push_back(target_pose);
            RCLCPP_WARN(logger, "sampled valid target");
            std::cout<<target_number<<std::endl;
        } else {
            RCLCPP_ERROR(logger, "Failed to sample a valid target pose or doesn't meet distance threshold!");
        }
    }

    std::ofstream outfile("/home/user/ws_moveit2/src/hello_moveit/src/valid_poses3.txt");

    if (outfile.is_open()) {
        for (const auto& pose : valid_poses) {
            outfile << "Position: (" << pose.position.x << ", " << pose.position.y << ", " << pose.position.z << ")\n";
            outfile << "Orientation: (" << pose.orientation.w << ", " << pose.orientation.x << ", " << pose.orientation.y << ", " << pose.orientation.z << ")\n";
        }
        outfile.flush();
        outfile.close();
        RCLCPP_INFO(logger, "Successfully saved valid poses to valid_poses2.txt");
    } else {
        RCLCPP_ERROR(logger, "Failed to open file for writing valid poses");
    }
   
    for (const auto& pose : valid_poses) {
        move_group_interface.setPoseTarget(pose);

        // Create a plan to that target pose
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto success = move_group_interface.plan(plan);
        if (success == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            move_group_interface.execute(plan);
            RCLCPP_ERROR(logger, "yaaay");
        } else {
            RCLCPP_ERROR(logger, "Execution failed for a valid target pose!");
        }
    }

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}