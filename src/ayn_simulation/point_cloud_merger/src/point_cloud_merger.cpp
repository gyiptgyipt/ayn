#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class OrientedPointCloudMerger : public rclcpp::Node {
public:
    OrientedPointCloudMerger() : Node("oriented_pointcloud_merger"),
        tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_)
    {
        // Parameter for the common frame
        this->declare_parameter<std::string>("target_frame", "base_link");
        target_frame_ = this->get_parameter("target_frame").as_string();

        // Create subscribers for each camera
        front_sub_ = create_camera_subscriber("/camera_front/camera_front/points", 0);
        left_sub_ = create_camera_subscriber("/camera_left/camera_left/points", 1);
        right_sub_ = create_camera_subscriber("/camera_right/camera_right/points", 2);

        // Publisher for merged cloud
        merged_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/merged_points", 10);
        
        // Publisher for visualization markers (to show camera directions)
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/camera_directions", 10);

        // Processing timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // 10Hz processing
            std::bind(&OrientedPointCloudMerger::process_and_publish, this));
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr create_camera_subscriber(
        const std::string& topic, size_t index) 
    {
        return this->create_subscription<sensor_msgs::msg::PointCloud2>(
            topic, 10,
            [this, index](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                clouds_[index] = *msg;
                received_[index] = true;
            });
    }

    void process_and_publish() {
        if (!all_received()) return;

        pcl::PointCloud<pcl::PointXYZRGB> merged_cloud;
        visualization_msgs::msg::MarkerArray camera_markers;
        
        try {
            // Process each camera's point cloud
            for (size_t i = 0; i < clouds_.size(); ++i) {
                if (clouds_[i].header.stamp.sec == 0) continue;

                // Transform to common frame
                auto transformed = transform_cloud(clouds_[i], target_frame_);
                if (!transformed) continue;

                // Convert to PCL and colorize
                pcl::PointCloud<pcl::PointXYZRGB> colored_cloud;
                pcl::fromROSMsg(*transformed, colored_cloud);
                
                // Assign different colors to each camera's points
                uint8_t r = 0, g = 0, b = 0;
                switch (i) {
                    case 0: r = 255; break;  // Front camera - red
                    case 1: g = 255; break;  // Left camera - green
                    case 2: b = 255; break;  // Right camera - blue
                }
                
                for (auto& point : colored_cloud) {
                    point.r = r;
                    point.g = g;
                    point.b = b;
                }
                
                merged_cloud += colored_cloud;
                
                // Add camera direction marker
                camera_markers.markers.push_back(create_camera_marker(i, clouds_[i].header.frame_id));
            }

            // Publish merged cloud
            if (!merged_cloud.empty()) {
                sensor_msgs::msg::PointCloud2 output;
                pcl::toROSMsg(merged_cloud, output);
                output.header.stamp = this->now();
                output.header.frame_id = target_frame_;
                merged_pub_->publish(output);
            }

            // Publish camera direction markers
            marker_pub_->publish(camera_markers);

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Processing error: %s", e.what());
        }
    }

    std::optional<sensor_msgs::msg::PointCloud2> transform_cloud(
        const sensor_msgs::msg::PointCloud2& cloud, 
        const std::string& target_frame)
    {
        try {
            geometry_msgs::msg::TransformStamped transform = tf_buffer_.lookupTransform(
                target_frame,
                cloud.header.frame_id,
                cloud.header.stamp,
                rclcpp::Duration::from_seconds(0.1));

            sensor_msgs::msg::PointCloud2 transformed;
            tf2::doTransform(cloud, transformed, transform);
            return transformed;
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "TF exception for %s: %s", 
                       cloud.header.frame_id.c_str(), ex.what());
            return std::nullopt;
        }
    }

    visualization_msgs::msg::Marker create_camera_marker(size_t cam_idx, const std::string& frame_id) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = target_frame_;
        marker.header.stamp = this->now();
        marker.ns = "camera_directions";
        marker.id = cam_idx;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        // Set color based on camera
        switch (cam_idx) {
            case 0:  // Front
                marker.color.r = 1.0;
                marker.color.a = 0.7;
                break;
            case 1:  // Left
                marker.color.g = 1.0;
                marker.color.a = 0.7;
                break;
            case 2:  // Right
                marker.color.b = 1.0;
                marker.color.a = 0.7;
                break;
        }
        
        // Set pose and scale
        marker.scale.x = 0.1;  // shaft diameter
        marker.scale.y = 0.2;  // head diameter
        marker.scale.z = 0.5;  // head length
        
        // Get transform from camera to base frame
        try {
            auto transform = tf_buffer_.lookupTransform(
                target_frame_, frame_id, tf2::TimePointZero);
            
            marker.pose.position.x = transform.transform.translation.x;
            marker.pose.position.y = transform.transform.translation.y;
            marker.pose.position.z = transform.transform.translation.z;
            marker.pose.orientation = transform.transform.rotation;
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "Could not get transform for marker: %s", ex.what());
        }
        
        return marker;
    }

    bool all_received() const {
        return std::all_of(received_.begin(), received_.end(), [](bool v) { return v; });
    }

    // Member variables
    std::array<sensor_msgs::msg::PointCloud2, 3> clouds_;
    std::array<bool, 3> received_ = {false, false, false};
    std::string target_frame_;
    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr front_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr left_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr right_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr merged_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OrientedPointCloudMerger>());
    rclcpp::shutdown();
    return 0;
}