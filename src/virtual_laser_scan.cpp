#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class VirtualLaserScan : public rclcpp::Node
{
public:
    VirtualLaserScan() : Node("virtual_laser_scan")
    {
    }

private:
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VirtualLaserScan>());
    rclcpp::shutdown();
    return 0;
}
