// cam_to_laser.cpp

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <vector>
#include <cmath>
#include <limits>

using std::placeholders::_1;

class CameraToLidarNode : public rclcpp::Node
{
public:
    CameraToLidarNode()
    : Node("camera_to_lidar_node")
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        sub_front_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera_front/camera_front/points", 10,
            std::bind(&CameraToLidarNode::front_callback, this, _1));

        sub_left_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera_left/camera_left/points", 10,
            std::bind(&CameraToLidarNode::left_callback, this, _1));

        sub_right_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera_right/camera_right/points", 10,
            std::bind(&CameraToLidarNode::right_callback, this, _1));

        pub_scan_ = this->create_publisher<sensor_msgs::msg::LaserScan>("simulated_scan", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&CameraToLidarNode::publish_scan, this));
    }

private:
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_front_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_left_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_right_;

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_scan_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<geometry_msgs::msg::Point> merged_points_;

    // Callback wrappers
    void front_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        process_points(msg, "front");
    }

    void left_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        process_points(msg, "left");
    }

    void right_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        process_points(msg, "right");
    }

    void process_points(const sensor_msgs::msg::PointCloud2::SharedPtr msg, const std::string &camera_id)
    {
        sensor_msgs::msg::PointCloud2 cloud_transformed;
        try
        {
            // Transform pointcloud to base_link frame
            cloud_transformed = tf_buffer_->transform(*msg, "base_link", tf2::durationFromSec(0.1));
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "TF transform failed for %s: %s", camera_id.c_str(), ex.what());
            return;
        }

        // Iterate points, project to 2D, apply filtering
        for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_transformed, "x"),
             iter_y(cloud_transformed, "y"), iter_z(cloud_transformed, "z");
             iter_x != iter_x.end();
             ++iter_x, ++iter_y, ++iter_z)
        {
            float x = *iter_x;
            float y = *iter_y;
            float z = *iter_z;

            if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z))
            {
                // Filter out points that are too high or low (optional)
                if (std::abs(z) < 0.5 && x > 0.1 && x < 5.0)
                {
                    geometry_msgs::msg::Point pt;
                    pt.x = x;
                    pt.y = y;
                    pt.z = 0.0; // flatten to 2D
                    merged_points_.push_back(pt);
                }
            }
        }
    }

    void publish_scan()
    {
        if (merged_points_.empty())
            return;  // nothing to publish

        constexpr float angle_min = -M_PI;
        constexpr float angle_max = M_PI;
        constexpr float angle_increment = M_PI / 180.0f; // 1 degree increments
        const int num_bins = static_cast<int>((angle_max - angle_min) / angle_increment);

        std::vector<float> ranges(num_bins, std::numeric_limits<float>::infinity());

        // Fill ranges with closest obstacles
        for (const auto &pt : merged_points_)
        {
            float angle = std::atan2(pt.y, pt.x);
            float range = std::hypot(pt.x, pt.y);

            int index = static_cast<int>((angle - angle_min) / angle_increment);
            if (index >= 0 && index < num_bins)
            {
                if (range < ranges[index])
                {
                    ranges[index] = range;
                }
            }
        }

        sensor_msgs::msg::LaserScan scan_msg;
        scan_msg.header.stamp = this->now();
        scan_msg.header.frame_id = "base_link";
        scan_msg.angle_min = angle_min;
        scan_msg.angle_max = angle_max;
        scan_msg.angle_increment = angle_increment;
        scan_msg.time_increment = 0.0f;
        scan_msg.scan_time = 0.1f; // corresponds to 10 Hz timer
        scan_msg.range_min = 0.1f;
        scan_msg.range_max = 5.0f;
        scan_msg.ranges = ranges;

        pub_scan_->publish(scan_msg);

        // Clear points after publishing
        merged_points_.clear();
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraToLidarNode>());
    rclcpp::shutdown();
    return 0;
}
