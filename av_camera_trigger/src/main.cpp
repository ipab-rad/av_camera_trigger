#include "rclcpp/rclcpp.hpp"
#include "av_camera_trigger/camera_trigger_node.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraTriggerNode>());
    rclcpp::shutdown();
    return 0;
}
