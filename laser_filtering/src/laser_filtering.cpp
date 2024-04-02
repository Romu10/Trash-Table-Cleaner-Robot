#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <functional>


class TrashTableDetect : public rclcpp::Node
{
public:
    TrashTableDetect() : Node("trash_table_detection")
    {
        // Define the laser filtered publisher 
        table_scan_filtered_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/table_scan_filtered", 10);

        // Define scan subscriber
        scan_sub_ = this-> create_subscription<sensor_msgs::msg::LaserScan>("/cleaner_2/scan", 10, 
            std::bind(&TrashTableDetect::scan_callback, this, std::placeholders::_1));

        RCLCPP_INFO_ONCE(this->get_logger(), "Laser Filtering Node Online...");
        
    }

private:
    
    // Laser callback
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
        if (!msg){
            RCLCPP_ERROR(this->get_logger(), "No data received from Scan");
        }
        else{
            // Print out the laser readings received
            laser_quantity = msg->ranges.size();
            RCLCPP_INFO_ONCE(this->get_logger(), "Data received from Scan: \nRanges: %i", laser_quantity);

            // Get the total number of readings from the laser 
            size_t num_readings = msg->ranges.size();

            // Calculate the initial angle and the angle increment
            double angle_min = msg->angle_min;
            double angle_increment = msg->angle_increment;

            // Calculate the min and max angle for the desired ranges in front of the robot
            double angle_min_range = -M_PI / 4.0;  // rad 
            double angle_max_range = M_PI / 4.0;   // rad

            // Create a new msg for LaserScan to save the reading in the desired range
            sensor_msgs::msg::LaserScan laser_in_range_msg;
            laser_in_range_msg.header = msg->header;
            laser_in_range_msg.angle_min = angle_min_range;
            laser_in_range_msg.angle_max = angle_max_range;
            laser_in_range_msg.angle_increment = msg->angle_increment;
            laser_in_range_msg.time_increment = msg->time_increment;
            laser_in_range_msg.scan_time = msg->scan_time;
            laser_in_range_msg.range_min = msg->range_min;
            laser_in_range_msg.range_max = 3.0;

            // Filter the readings inside the desired angle 
            for (size_t i = 0; i < num_readings; ++i)
            {
                // Calculate the angle of the actual laser reading 
                double angle = angle_min + i * angle_increment;

                // Verify if the angle is in the desired range 
                if (angle >= angle_min_range && angle <= angle_max_range)
                {
                    // Process the laser reading in the desired angle 
                    laser_in_range_msg.ranges.push_back(msg->ranges[i]);

                }
            }

            // Publish the message
            table_scan_filtered_->publish(laser_in_range_msg);
        }
    }

    // ROS 2 definitions 
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_; 
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr table_scan_filtered_;

    // scan callback variables

        // distances
    std::vector<float> laser_distances;
        // laser quantity
    int laser_quantity; 

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
