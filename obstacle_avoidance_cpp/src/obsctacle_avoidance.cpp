#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class ObstacleAvoidance : public rclcpp::Node {
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
    float forward_speed; // Forward speed of the robot
    float rotation_speed; // Rotation speed of the robot
    float min_distance_threshold; // Minimum distance to obstacle
    float min_distance; // Minimum distance from laser scan data

public:
    ObstacleAvoidance() : Node("obstacle_avoidance"), forward_speed(0.2), rotation_speed(0.5), min_distance_threshold(0.5) {
        cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&ObstacleAvoidance::laserCallback, this, std::placeholders::_1));
    }

    void avoidObstacle() {
        rclcpp::Rate loop_rate(10);

        while (rclcpp::ok()) {
            auto cmd_vel_msg = std::make_shared<geometry_msgs::msg::Twist>();

            // If obstacle detected within threshold distance, rotate to avoid
            if (min_distance < min_distance_threshold) {
                cmd_vel_msg->linear.x = 0; // Stop forward motion
                cmd_vel_msg->angular.z = rotation_speed; // Rotate to avoid obstacle
            } else {
                cmd_vel_msg->linear.x = forward_speed; // Move forward
                cmd_vel_msg->angular.z = 0; // No rotation
            }

            cmd_vel_pub->publish(*cmd_vel_msg);
            rclcpp::spin_some(this->get_node_base_interface());
            loop_rate.sleep();
        }
    }

    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // Find minimum distance from laser scan data
        min_distance = *std::min_element(msg->ranges.begin(), msg->ranges.end());
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto avoidance = std::make_shared<ObstacleAvoidance>();
    avoidance->avoidObstacle();
    rclcpp::shutdown();
    return 0;
}
