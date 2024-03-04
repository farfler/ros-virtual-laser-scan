#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class VirtualLaserCloud : public rclcpp::Node
{
public:
    VirtualLaserCloud() : Node("virtual_laser_cloud")
    {
    }

private:
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VirtualLaserCloud>());
    rclcpp::shutdown();
    return 0;
}
