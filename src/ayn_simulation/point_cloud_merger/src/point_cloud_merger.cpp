#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class PointCloudMerger : public rclcpp::Node {
public:
    PointCloudMerger() : Node("pointcloud_merger") {
        // Create subscribers
        front_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera_front/camera_front/points", 10,
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { clouds_[0] = *msg; });
            
        left_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera_left/camera_left/points", 10,
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { clouds_[1] = *msg; });
            
        right_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera_right/camera_right/points", 10,
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { clouds_[2] = *msg; });

        // Create publisher and timer
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/merged_points", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // 10Hz
            std::bind(&PointCloudMerger::mergeAndPublish, this));
    }

private:
    void mergeAndPublish() {
        pcl::PointCloud<pcl::PointXYZ> merged_cloud;
        
        for (auto& cloud_msg : clouds_) {
            if (cloud_msg.header.stamp.sec == 0) continue;  // Skip uninitialized
            
            pcl::PointCloud<pcl::PointXYZ> temp_cloud;
            pcl::fromROSMsg(cloud_msg, temp_cloud);
            merged_cloud += temp_cloud;
        }
        
        if (!merged_cloud.empty()) {
            sensor_msgs::msg::PointCloud2 output_msg;
            pcl::toROSMsg(merged_cloud, output_msg);
            output_msg.header.stamp = this->now();
            output_msg.header.frame_id = "base_link";  // Set your target frame
            publisher_->publish(output_msg);
        }
    }

    std::array<sensor_msgs::msg::PointCloud2, 3> clouds_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr front_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr left_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr right_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudMerger>());
    rclcpp::shutdown();
    return 0;
}