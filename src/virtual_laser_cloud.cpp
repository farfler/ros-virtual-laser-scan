#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <pcl_conversions/pcl_conversions.h>

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

        front_left_laser_scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            front_left_laser_scan_topic_, 10, std::bind(&VirtualLaserCloud::front_left_laser_scan_callback, this, std::placeholders::_1));
        front_right_laser_scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            front_right_laser_scan_topic_, 10, std::bind(&VirtualLaserCloud::front_right_laser_scan_callback, this, std::placeholders::_1));
        rear_left_laser_scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            rear_left_laser_scan_topic_, 10, std::bind(&VirtualLaserCloud::rear_left_laser_scan_callback, this, std::placeholders::_1));
        rear_right_laser_scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            rear_right_laser_scan_topic_, 10, std::bind(&VirtualLaserCloud::rear_right_laser_scan_callback, this, std::placeholders::_1));

        virtual_point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(virtual_laser_cloud_topic_, 10);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

private:
    void front_left_laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        front_left_laser_scan_ = msg;
        update();
    }

    void front_right_laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        front_right_laser_scan_ = msg;
        update();
    }

    void rear_left_laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        rear_left_laser_scan_ = msg;
        update();
    }

    void rear_right_laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        rear_right_laser_scan_ = msg;
        update();
    }

    void update()
    {
        if (virtual_laser_cloud_frame_id_.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Virtual virtual_laser_cloud_frame_id_ is empty");
            return;
        }

        pcl::PointCloud<pcl::PointXYZRGB> point_cloud;

        if (front_left_laser_scan_)
        {
            geometry_msgs::msg::TransformStamped front_left_transform = tf_buffer_->lookupTransform(
                virtual_laser_cloud_frame_id_, front_left_laser_scan_->header.frame_id, rclcpp::Time(0));
            process(front_left_laser_scan_, front_left_transform, point_cloud);
        }

        if (front_right_laser_scan_)
        {
            geometry_msgs::msg::TransformStamped front_right_transform = tf_buffer_->lookupTransform(
                virtual_laser_cloud_frame_id_, front_right_laser_scan_->header.frame_id, rclcpp::Time(0));
            process(front_right_laser_scan_, front_right_transform, point_cloud);
        }

        if (rear_left_laser_scan_)
        {
            geometry_msgs::msg::TransformStamped rear_left_transform = tf_buffer_->lookupTransform(
                virtual_laser_cloud_frame_id_, rear_left_laser_scan_->header.frame_id, rclcpp::Time(0));
            process(rear_left_laser_scan_, rear_left_transform, point_cloud);
        }

        if (rear_right_laser_scan_)
        {
            geometry_msgs::msg::TransformStamped rear_right_transform = tf_buffer_->lookupTransform(
                virtual_laser_cloud_frame_id_, rear_right_laser_scan_->header.frame_id, rclcpp::Time(0));
            process(rear_right_laser_scan_, rear_right_transform, point_cloud);
        }

        publish(point_cloud);
    }

    void process(const sensor_msgs::msg::LaserScan::SharedPtr &laser_scan, const geometry_msgs::msg::TransformStamped &transform, pcl::PointCloud<pcl::PointXYZRGB> &point_cloud)
    {
        RCLCPP_INFO(this->get_logger(), "Processing virtual laser cloud");
    }

    void publish(pcl::PointCloud<pcl::PointXYZRGB> &point_cloud)
    {
        RCLCPP_INFO(this->get_logger(), "Publishing virtual laser cloud");
    }

    std::string front_left_laser_scan_topic_;
    std::string front_right_laser_scan_topic_;
    std::string rear_left_laser_scan_topic_;
    std::string rear_right_laser_scan_topic_;

    std::string virtual_laser_cloud_topic_;
    std::string virtual_laser_cloud_frame_id_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr front_left_laser_scan_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr front_right_laser_scan_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr rear_left_laser_scan_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr rear_right_laser_scan_subscription_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr virtual_point_cloud_publisher_;

    sensor_msgs::msg::LaserScan::SharedPtr front_left_laser_scan_;
    sensor_msgs::msg::LaserScan::SharedPtr front_right_laser_scan_;
    sensor_msgs::msg::LaserScan::SharedPtr rear_left_laser_scan_;
    sensor_msgs::msg::LaserScan::SharedPtr rear_right_laser_scan_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VirtualLaserCloud>());
    rclcpp::shutdown();
    return 0;
}
