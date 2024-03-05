#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class VirtualLaserScan : public rclcpp::Node
{
public:
    VirtualLaserScan() : Node("virtual_laser_scan")
    {
        this->declare_parameter<std::string>("virtual_laser_scan_topic", "virtual_laser/scan");
        this->get_parameter_or<std::string>("virtual_laser_scan_topic", virtual_laser_scan_topic_, "virtual_laser/scan");
        this->declare_parameter<std::string>("virtual_laser_scan_frame_id", "virtual_laser_frame");
        this->get_parameter_or<std::string>("virtual_laser_scan_frame_id", virtual_laser_scan_frame_id_, "virtual_laser_frame");
        this->declare_parameter<std::string>("virtual_laser_cloud_topic", "virtual_laser/cloud");
        this->get_parameter_or<std::string>("virtual_laser_cloud_topic", virtual_laser_cloud_topic_, "virtual_laser/cloud");
        // this->declare_parameter<std::string>("virtual_laser_cloud_frame_id", "virtual_laser_frame");
        // this->get_parameter_or<std::string>("virtual_laser_cloud_frame_id", virtual_laser_cloud_frame_id_, "virtual_laser_frame");

        this->declare_parameter<float>("virtual_laser_scan_angle_min", -M_PI);
        this->get_parameter_or<float>("virtual_laser_scan_angle_min", virtual_laser_scan_angle_min_, -M_PI);
        this->declare_parameter<float>("virtual_laser_scan_angle_max", M_PI);
        this->get_parameter_or<float>("virtual_laser_scan_angle_max", virtual_laser_scan_angle_max_, M_PI);
        this->declare_parameter<float>("virtual_laser_scan_angle_increment", M_PI / 180.0);
        this->get_parameter_or<float>("virtual_laser_scan_angle_increment", virtual_laser_scan_angle_increment_, M_PI / 180.0);
        this->declare_parameter<float>("virtual_laser_scan_time_increment", 0.0);
        this->get_parameter_or<float>("virtual_laser_scan_time_increment", virtual_laser_scan_time_increment_, 0.0);
        this->declare_parameter<float>("virtual_laser_scan_scan_time", 0.0);
        this->get_parameter_or<float>("virtual_laser_scan_scan_time", virtual_laser_scan_scan_time_, 0.0);
        this->declare_parameter<float>("virtual_laser_scan_range_min", 1.0);
        this->get_parameter_or<float>("virtual_laser_scan_range_min", virtual_laser_scan_range_min_, 1.0);
        this->declare_parameter<float>("virtual_laser_scan_range_max", 20.0);
        this->get_parameter_or<float>("virtual_laser_scan_range_max", virtual_laser_scan_range_max_, 20.0);
    }

private:
    std::string virtual_laser_scan_topic_;
    std::string virtual_laser_scan_frame_id_;
    std::string virtual_laser_cloud_topic_;
    // std::string virtual_laser_cloud_frame_id_;

    float virtual_laser_scan_angle_min_;
    float virtual_laser_scan_angle_max_;
    float virtual_laser_scan_angle_increment_;
    float virtual_laser_scan_time_increment_;
    float virtual_laser_scan_scan_time_;
    float virtual_laser_scan_range_min_;
    float virtual_laser_scan_range_max_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VirtualLaserScan>());
    rclcpp::shutdown();
    return 0;
}
