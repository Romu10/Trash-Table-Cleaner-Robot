#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/detail/set_bool__struct.hpp"
#include "std_srvs/srv/set_bool.hpp"

class TrashTableDetect : public rclcpp::Node
{
public:
    TrashTableDetect() : Node("trash_table_detection")
    {
        // Define el servicio
        detection_srv_ = this->create_service<std_srvs::srv::SetBool>("trash_table_detection_srv", 
            std::bind(&TrashTableDetect::service_callback, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    // Callback del servicio
    void service_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        // Lógica del servicio
        RCLCPP_INFO(this->get_logger(), "Service called");
        response->success = true;
    }

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr detection_srv_;
};

int main(int argc, char *argv[])
{
    // Inicializa ROS 2
    rclcpp::init(argc, argv);

    // Crea un nodo y ejecuta el servicio
    auto node = std::make_shared<TrashTableDetect>();
    rclcpp::spin(node);

    // Apaga ROS 2
    rclcpp::shutdown();
    return 0;
}
