#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class VirtualLaserCloud : public rclcpp::Node
{
public:
    VirtualLaserCloud() : Node("virtual_laser_cloud")
    {
        this->declare_parameter<std::string>("front_left_laser_scan_topic", "front_left_laser/scan");
        this->get_parameter_or<std::string>("front_left_laser_scan_topic", front_left_laser_scan_topic_, "front_left_laser/scan");
        this->declare_parameter<std::string>("front_right_laser_scan_topic", "front_right_laser/scan");
        this->get_parameter_or<std::string>("front_right_laser_scan_topic", front_right_laser_scan_topic_, "front_right_laser/scan");
        this->declare_parameter<std::string>("rear_left_laser_scan_topic", "rear_left_laser/scan");
        this->get_parameter_or<std::string>("rear_left_laser_scan_topic", rear_left_laser_scan_topic_, "rear_left_laser/scan");
        this->declare_parameter<std::string>("rear_right_laser_scan_topic", "rear_right_laser/scan");
        this->get_parameter_or<std::string>("rear_right_laser_scan_topic", rear_right_laser_scan_topic_, "rear_right_laser/scan");

        this->declare_parameter<std::string>("virtual_laser_cloud_topic_", "virtual_laser/cloud");
        this->get_parameter_or<std::string>("virtual_laser_cloud_topic_", virtual_laser_cloud_topic_, "virtual_laser/cloud");
        this->declare_parameter<std::string>("virtual_laser_cloud_frame_id_", "virtual_laser_frame");
        this->get_parameter_or<std::string>("virtual_laser_cloud_frame_id_", virtual_laser_cloud_frame_id_, "virtual_laser_frame");
    }

private:
    std::string front_left_laser_scan_topic_;
    std::string front_right_laser_scan_topic_;
    std::string rear_left_laser_scan_topic_;
    std::string rear_right_laser_scan_topic_;

    std::string virtual_laser_cloud_topic_;
    std::string virtual_laser_cloud_frame_id_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VirtualLaserCloud>());
    rclcpp::shutdown();
    return 0;
}
