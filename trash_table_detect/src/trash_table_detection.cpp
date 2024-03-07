#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "std_srvs/srv/detail/set_bool__struct.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"


class TrashTableDetect : public rclcpp::Node
{
public:
    TrashTableDetect() : Node("trash_table_detection")
    {
        // Define de service
        detection_srv_ = this->create_service<std_srvs::srv::SetBool>("trash_table_detection_srv", 
            std::bind(&TrashTableDetect::service_callback, this, std::placeholders::_1, std::placeholders::_2));

        // Define the cmd_vel publisher 
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/diffbot_base_controller/cmd_vel_unstamped", 10);

        // Define the odometry subscriber
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/diffbot_base_controller/odom", 10, std::bind(&TrashTableDetect::odom_callback, this, std::placeholders::_1));
    }

private:
    
    // Service callback
    void service_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        // Service logic
        RCLCPP_INFO(this->get_logger(), "Service called");
        response->success = true;
    }

    // Odometry helper function
    long double calculateYaw(long double qx, long double qy, long double qz, long double qw) {
        long double yaw = std::atan2(2 * ((qw * qz) + (qx * qy)), 1 - 2 * ((qy * qy) + (qz * qz)));
        return yaw;
    }

    // Odometry callback 
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {

        if (!msg){
            RCLCPP_ERROR(this->get_logger(), "No data received from Odometry");
        }
        else{
        
            // Getting robot's current position
            cur_pos = msg->pose.pose.position; 

            // Calculate Yaw
            quaternion_x = msg->pose.pose.orientation.x;
            quaternion_y = msg->pose.pose.orientation.y;
            quaternion_z = msg->pose.pose.orientation.z;
            quaternion_w = msg->pose.pose.orientation.w;

            quaternion.x = quaternion_x;
            quaternion.y = quaternion_y;
            quaternion.z = quaternion_z;
            quaternion.w = quaternion_w;

            calculate_yaw = calculateYaw(quaternion_x, quaternion_y, quaternion_z, quaternion_w);
            current_yaw = (calculate_yaw * (180.0 / M_PI));

            RCLCPP_INFO_ONCE(this->get_logger(), "Data received from Odometry: \nYaw: %f", calculate_yaw);
        }

    }

    // ROS 2 definitions 
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr detection_srv_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_; 

    // Odometry callback variables
        
        // Pos related 
    geometry_msgs::msg::Point cur_pos;

        // Yaw related
    double calculate_yaw; 
    double current_yaw; 

        // Calculate Yaw related
    geometry_msgs::msg::Quaternion quaternion;
    long double quaternion_x, quaternion_y, quaternion_z, quaternion_w = 0.0; 

};

int main(int argc, char *argv[])
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create node and run the service
    auto node = std::make_shared<TrashTableDetect>();
    rclcpp::spin(node);

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
