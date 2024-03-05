#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

class VirtualLaserScan : public rclcpp::Node
{
public:
    VirtualLaserScan() : Node("virtual_laser_scan")
    {
        this->declare_parameter<std::string>("virtual_laser_scan_topic", "virtual_laser/scan");
        this->get_parameter_or<std::string>("virtual_laser_scan_topic", virtual_laser_scan_topic_, "virtual_laser/scan");
        this->declare_parameter<std::string>("virtual_laser_scan_frame_id", "virtual_laser_frame");
        this->get_parameter_or<std::string>("virtual_laser_scan_frame_id", virtual_laser_scan_frame_id_, "virtual_laser_frame");
        this->declare_parameter<std::string>("virtual_point_cloud_topic", "virtual_laser/cloud");
        this->get_parameter_or<std::string>("virtual_point_cloud_topic", virtual_point_cloud_topic_, "virtual_laser/cloud");
        // this->declare_parameter<std::string>("virtual_point_cloud_frame_id", "virtual_laser_frame");
        // this->get_parameter_or<std::string>("virtual_point_cloud_frame_id", virtual_point_cloud_frame_id_, "virtual_laser_frame");

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

        virtual_point_cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            virtual_point_cloud_topic_, 10, std::bind(&VirtualLaserScan::virtual_point_cloud_callback, this, std::placeholders::_1));

        virtual_laser_scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(virtual_laser_scan_topic_, 10);
    }

private:
    void virtual_point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        virtual_point_cloud_ = msg;
        update();
    }

    void update()
    {
        auto virtual_laser_scan = std::make_unique<sensor_msgs::msg::LaserScan>();

        virtual_laser_scan->header.frame_id = virtual_laser_scan_frame_id_;
        virtual_laser_scan->header.stamp = this->now();

        virtual_laser_scan->angle_min = virtual_laser_scan_angle_min_;
        virtual_laser_scan->angle_max = virtual_laser_scan_angle_max_;
        virtual_laser_scan->angle_increment = virtual_laser_scan_angle_increment_;
        virtual_laser_scan->time_increment = virtual_laser_scan_time_increment_;
        virtual_laser_scan->scan_time = virtual_laser_scan_scan_time_;
        virtual_laser_scan->range_min = virtual_laser_scan_range_min_;
        virtual_laser_scan->range_max = virtual_laser_scan_range_max_;

        uint32_t ranges = std::ceil((virtual_laser_scan->angle_max - virtual_laser_scan->angle_min) / virtual_laser_scan->angle_increment);

        virtual_laser_scan->ranges.assign(ranges, std::numeric_limits<double>::infinity());

        for (sensor_msgs::PointCloud2ConstIterator<float> x(*virtual_point_cloud_, "x"), y(*virtual_point_cloud_, "y"), z(*virtual_point_cloud_, "z"); x != x.end(); ++x, ++y, ++z)
        {

            if (std::isnan(*x) || std::isnan(*y) || std::isnan(*z))
            {
                RCLCPP_DEBUG(
                    this->get_logger(),
                    "Ignoring point with NaN in position Point: (%f, %f, %f)",
                    *x, *y, *z);
                continue;
            }

            float range = hypot(*x, *y);

            if (range < virtual_laser_scan_range_min_)
            {
                RCLCPP_DEBUG(
                    this->get_logger(),
                    "Ignoring point for being too close. Point: (%f, %f, %f) Range: %f, Min Range: %f",
                    *x, *y, *z, range, virtual_laser_scan_range_min_);
                continue;
            }

            if (range > virtual_laser_scan_range_max_)
            {
                RCLCPP_DEBUG(
                    this->get_logger(),
                    "Ignoring point for being too far away. Point: (%f, %f, %f) Range: %f, Max Range: %f",
                    *x, *y, *z, range, virtual_laser_scan_range_max_);
                continue;
            }

            float angle = atan2(*y, *x);

            if (angle < virtual_laser_scan_angle_min_ || angle > virtual_laser_scan_angle_max_)
            {
                RCLCPP_DEBUG(
                    this->get_logger(),
                    "Ignoring point for being out of scan range. Point: (%f, %f, %f) Angle: %f, Min Angle: %f, Max Angle: %f",
                    *x, *y, *z, angle, virtual_laser_scan_angle_min_, virtual_laser_scan_angle_max_);
                continue;
            }

            int index = (angle - virtual_laser_scan_angle_min_) / virtual_laser_scan_angle_increment_;

            if (range < virtual_laser_scan->ranges[index])
            {
                virtual_laser_scan->ranges[index] = range;
            }
        }

        virtual_laser_scan_publisher_->publish(std::move(virtual_laser_scan));
    }

    std::string virtual_laser_scan_topic_;
    std::string virtual_laser_scan_frame_id_;
    std::string virtual_point_cloud_topic_;
    // std::string virtual_point_cloud_frame_id_;

    float virtual_laser_scan_angle_min_;
    float virtual_laser_scan_angle_max_;
    float virtual_laser_scan_angle_increment_;
    float virtual_laser_scan_time_increment_;
    float virtual_laser_scan_scan_time_;
    float virtual_laser_scan_range_min_;
    float virtual_laser_scan_range_max_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr virtual_point_cloud_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr virtual_laser_scan_publisher_;

    sensor_msgs::msg::PointCloud2::SharedPtr virtual_point_cloud_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VirtualLaserScan>());
    rclcpp::shutdown();
    return 0;
}
