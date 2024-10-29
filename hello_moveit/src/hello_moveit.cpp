#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <chrono>
#include <functional>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <iostream>
#include <cmath>
#include <moveit/planning_scene/planning_scene.h>
#include <cstdlib>  // For rand() and srand()

using namespace std;

double getRandomValue(double min, double max) {
    double scale = rand() / (double)RAND_MAX; // Scale between 0 and 1
    return min + scale * (max - min); // Adjust range
}

class ObjectMotionController : public rclcpp::Node {
public:
    ObjectMotionController() : Node("object_motion_controller") {

        this -> set_parameter(rclcpp::Parameter("use_sim_time" , true));
        
        object_pose_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("object_position", 10);

        target_pose_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "target_pose", 10, std::bind(&ObjectMotionController::targetPoseCallback, this, std::placeholders::_1));

        object_pose_.position.x = -0.198263;
        object_pose_.position.y = 0.488319;
        object_pose_.position.z = 0.2;

        // Delay initialization of MoveGroupInterface to ensure ROS is fully initialized
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&ObjectMotionController::initializeMoveGroup, this));
    }

private:
    void initializeMoveGroup() {
        RCLCPP_INFO(get_logger(), "Initializing MoveGroupInterface");

        static const std::string PLANNING_GROUP = "panda_arm";
        move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), PLANNING_GROUP);

        const moveit::core::JointModelGroup* joint_model_group = move_group_interface_->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
        moveit::core::RobotStatePtr current_state = move_group_interface_->getCurrentState(10);

        move_group_interface_->setPlanningTime(10);
        move_group_interface_->setMaxVelocityScalingFactor(1.0);
        move_group_interface_->setMaxAccelerationScalingFactor(1.0);

        marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
        timer_->cancel();
    }

    void targetPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
        if (!move_group_interface_) {
            RCLCPP_ERROR(get_logger(), "MoveGroupInterface not initialized!");
            return;
        }
        RCLCPP_INFO(get_logger(), "Target Pose: [x: %f, y: %f, z: %f, qx: %f, qy: %f, qz: %f, qw: %f]",
                msg->position.x, msg->position.y, msg->position.z,
                msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);

        move_group_interface_->setPoseTarget(*msg);
        RCLCPP_INFO(get_logger(), "Setting target pose");

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_interface_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if (success) {
            move_group_interface_->move();
            RCLCPP_INFO(get_logger(), "Motion execution succeeded");
            observeEnvironment(msg);
            std::cout << "your object position: (" << object_pose_.position.x << ", " << object_pose_.position.y << ", " << object_pose_.position.z << ")" << std::endl;
            object_pose_publisher_->publish(object_pose_);
            publishMarker();
            storeCurrentJointValues();
            std::cout<<"done bro"<<std::endl;
        } else {
            RCLCPP_ERROR(get_logger(), "Planning failed!");
            object_pose_publisher_->publish(object_pose_);
            publishMarker();
            storeCurrentJointValues();      
        }
    }

    void observeEnvironment(const geometry_msgs::msg::Pose::SharedPtr end_effector_pose) {
        if (!move_group_interface_) {
            return;
        }

        double end_effector_x = end_effector_pose->position.x;
        double end_effector_y = end_effector_pose->position.y;
        double end_effector_z = end_effector_pose->position.z;

        // std::cout<<current_state<<std::endl;
        // std::cout<<& end_effector_state<<std::endl;
        double object_distance_from_origin = sqrt(pow(object_pose_.position.x, 2) +
                                              pow(object_pose_.position.y, 2) +
                                              pow(object_pose_.position.z, 2));
                                              
        if (object_distance_from_origin > 0.6) {
        // Reset object position to its original values
            object_pose_.position.x = -0.198263;
            object_pose_.position.y = 0.488319;
            object_pose_.position.z = 0.2;

            std::cout << "Object position reset to original: (" << object_pose_.position.x << ", " << object_pose_.position.y << ", " << object_pose_.position.z << ")" << std::endl;
        }else{
            double end_effector_x = end_effector_pose->position.x;
        double end_effector_y = end_effector_pose->position.y;
        double end_effector_z = end_effector_pose->position.z;

        double distance = sqrt(pow(end_effector_x - object_pose_.position.x, 2) +
                               pow(end_effector_y - object_pose_.position.y, 2) +
                               pow(end_effector_z - object_pose_.position.z, 2));

        double proximity_threshold = 0.3; // Adjust as needed
        if (distance < proximity_threshold) {
            Eigen::Vector3d direction(
                object_pose_.position.x - end_effector_x,
                object_pose_.position.y - end_effector_y,
                0.0); // Movement in XY plane

            direction.normalize();

            double move_factor = 0.1; // Adjust speed and direction factor
            object_pose_.position.x += direction.x() * move_factor;
            object_pose_.position.y += direction.y() * move_factor;
            object_pose_.position.z = 0.2; // Maintain constant z height

            std::cout << "New object position: (" << object_pose_.position.x << ", " << object_pose_.position.y << ", " << object_pose_.position.z << ")" << std::endl;
            }   
        }   
    }

    void publishMarker() {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "panda_link0";  // Adjust frame ID as needed
        marker.header.stamp = this->now();
        marker.ns = "object";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        std::cout << "marker object position: (" << object_pose_.position.x << ", " << object_pose_.position.y << ", " << object_pose_.position.z << ")" << std::endl;
        marker.pose = object_pose_;

        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;

        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        marker_publisher_->publish(marker);
    }

    void storeCurrentJointValues() {
        if (!move_group_interface_) {
            return;
        }

        const std::vector<double>& joint_values = move_group_interface_->getCurrentJointValues();
        RCLCPP_INFO(this->get_logger(), "Current Joint Values:");
        for (const auto& value : joint_values) {
            RCLCPP_INFO(this->get_logger(), "%f", value);
        }
        // Store the joint values as needed, e.g., write to a file or database
    }

    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr object_pose_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr target_pose_subscription_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
    geometry_msgs::msg::Pose object_pose_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectMotionController>());
    rclcpp::shutdown();
    return 0;
}
