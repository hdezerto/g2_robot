// Includes ROS2 and PCL libraries for working with point clouds and ICP.
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>  // ROS2 message type for PointCloud2
#include <pcl/point_cloud.h>                  // PointCloud container
#include <pcl/point_types.h>                  // PointXYZ type for 3D points
#include <pcl_conversions/pcl_conversions.h>  // For converting PCL to ROS PointCloud
#include <pcl/registration/icp.h>             // ICP (Iterative Closest Point) registration

// ICPProcessor class inherits from rclcpp::Node, which represents a ROS 2 node.
class ICPProcessor : public rclcpp::Node {
public:
    // Constructor for the ICPProcessor class
    ICPProcessor() : Node("icp_processor") {
        // Create a subscription to the topic "/nth_simple_pointcloud" for receiving point clouds
        subscription_nth_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/nth_simple_pointcloud", 10,  // Queue size of 10
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                // Callback for when a new point cloud is received, processes it with ICP
                pointcloud_callback(msg, "/nth_corrected_pointcloud", prev_nth_cloud_);
            }
        );
        // Publisher to send the corrected point cloud to "/nth_corrected_pointcloud"
        publisher_nth_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/nth_corrected_pointcloud", 10);

        // Create a second subscription for the "/uncorrected_accumulated_pointcloud"
        subscription_accum_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/uncorrected_accumulated_pointcloud", 10,  // Queue size of 10
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                // Callback for accumulated point cloud
                pointcloud_callback(msg, "/corrected_accumulated_pointcloud", prev_accum_cloud_);
            }
        );
        // Publisher to send the corrected accumulated point cloud to "/corrected_accumulated_pointcloud"
        publisher_accum_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/corrected_accumulated_pointcloud", 10);
    }

private:
    // Declare subscriptions and publishers for handling point cloud data
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_nth_;   // Subscription for nth point cloud
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_nth_;         // Publisher for nth corrected point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr prev_nth_cloud_ {nullptr};  // Pointer to the previous nth cloud (for ICP)

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_accum_;  // Subscription for accumulated point cloud
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_accum_;        // Publisher for corrected accumulated point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr prev_accum_cloud_ {nullptr};  // Pointer to the previous accumulated cloud (for ICP)

    // This method performs the ICP algorithm to align the received point cloud with the previous one
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg, const std::string &output_topic, pcl::PointCloud<pcl::PointXYZ>::Ptr &prev_cloud) {
        // Convert the received ROS message (PointCloud2) to a PCL PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        // If this is the first cloud, store it and skip the ICP process
        if (!prev_cloud) {
            prev_cloud = cloud;
            RCLCPP_INFO(this->get_logger(), "Stored first cloud for topic %s, skipping ICP on first pass.", output_topic.c_str());
            return;
        }

        // Initialize the ICP object (Iterative Closest Point)
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(cloud);     // The new cloud that we want to align
        icp.setInputTarget(prev_cloud); // The previous cloud to align against

        // Set ICP parameters (tuning ICP performance and precision)
        icp.setMaxCorrespondenceDistance(0.05);  // Max distance between correspondences (points)
        icp.setMaximumIterations(50);            // Max iterations before stopping
        icp.setTransformationEpsilon(1e-8);      // Convergence criteria (change in transformation)
        icp.setEuclideanFitnessEpsilon(1e-5);    // Stop if the fitness score is below this threshold

        // Create a container to hold the aligned cloud
        pcl::PointCloud<pcl::PointXYZ> aligned;
        icp.align(aligned);  // Run the ICP algorithm

        // If ICP converged (i.e., alignment was successful)
        if (icp.hasConverged()) {
            // Convert the aligned PCL cloud back to a ROS PointCloud2 message
            sensor_msgs::msg::PointCloud2 output;
            pcl::toROSMsg(aligned, output);
            output.header = msg->header;  // Preserve original message header (timestamps, frame_id)
            output.header.frame_id = msg->header.frame_id;  // Ensure frame_id is passed correctly

            // Publish the aligned cloud to the appropriate topic
            if (output_topic == "/nth_corrected_pointcloud") {
                publisher_nth_->publish(output);  // Publish for nth point cloud
            } else {
                publisher_accum_->publish(output);  // Publish for accumulated point cloud
            }

            // Log a message indicating the cloud was successfully corrected and published
            RCLCPP_INFO(this->get_logger(), "Published corrected point cloud to %s.", output_topic.c_str());
        } else {
            // Log a warning if ICP failed to converge
            RCLCPP_WARN(this->get_logger(), "ICP failed to converge on topic %s.", output_topic.c_str());
        }

        // Update the previous cloud for the next iteration
        prev_cloud = cloud;
    }
};

// Main entry point for the program
int main(int argc, char** argv) {
    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Create and spin the ICPProcessor node (processes callbacks and handles point cloud corrections)
    rclcpp::spin(std::make_shared<ICPProcessor>());

    // Shutdown ROS2 when finished
    rclcpp::shutdown();

    return 0;
}
