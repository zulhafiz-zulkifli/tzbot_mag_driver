#include "tzbot_mag/tzbot_mag_node.h"

Main::Main(const rclcpp::NodeOptions &options): Node("tzbot_mag", options){
	RCLCPP_INFO(get_logger(),"tzbot_mag node has started.");
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Main>());
    rclcpp::shutdown();
    return 0;
}