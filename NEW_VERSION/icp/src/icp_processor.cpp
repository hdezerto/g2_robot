// Includes ROS2 and PCL libraries for working with point clouds and ICP.
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>  // ROS2 message type for PointCloud2
#include <pcl/point_cloud.h>                  // PointCloud container
#include <pcl/point_types.h>                  // PointXYZ type for 3D points
#include <pcl_conversions/pcl_conversions.h>  // For converting PCL to ROS PointCloud
#include <pcl/registration/icp.h>             // ICP (Iterative Closest Point) registration

#include "tf2_ros/static_transform_broadcaster.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>  // Downsampling with Voxel Grid

// ICPProcessor class inherits from rclcpp::Node, which represents a ROS 2 node.
class ICPProcessor : public rclcpp::Node {
public:
    ICPProcessor() : Node("icp_processor") {
        subscription_nth_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/nth_simple_pointcloud", 10,
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                pointcloud_callback(msg, "/nth_corrected_pointcloud", prev_nth_cloud_);
            }
        );
        publisher_nth_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/nth_corrected_pointcloud", 10);

        subscription_accum_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/uncorrected_accumulated_pointcloud", 10,
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                pointcloud_callback(msg, "/corrected_accumulated_pointcloud", prev_accum_cloud_);
            }
        );
        publisher_accum_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/corrected_accumulated_pointcloud", 10);
    
        //Publish static transform
        tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);
        //Publish dynamic transform
        //tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_nth_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_nth_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr prev_nth_cloud_ {nullptr};

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_accum_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_accum_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr prev_accum_cloud_ {nullptr};

    const float MAX_TRANSLATION = 0.2;    // Maximum translation threshold (meters)
    const float MAX_ROTATION = 0.2;    // Maximum rotation threshold (radians) ~ 10 degrees
    

    pcl::PointCloud<pcl::PointXYZ>::Ptr corrected_accum_cloud_ {nullptr};  // Store corrected accumulated cloud

    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg, 
                             const std::string &output_topic, 
                             pcl::PointCloud<pcl::PointXYZ>::Ptr &prev_cloud) {
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
    
        if (!prev_cloud) {
            prev_cloud = cloud;
            RCLCPP_INFO(this->get_logger(), "Stored first cloud for topic %s, skipping ICP on first pass.", output_topic.c_str());
            return;
        }
    
        // Determine reference cloud for ICP:
        pcl::PointCloud<pcl::PointXYZ>::Ptr reference_cloud;
        
        if (output_topic == "/corrected_accumulated_pointcloud") {
            // Use the previously corrected accumulated cloud, or fallback to prev_cloud if first run
            reference_cloud = (corrected_accum_cloud_) ? corrected_accum_cloud_ : prev_cloud;
        } else {
            // Use the previous LiDAR scan for /nth_corrected_pointcloud
            //reference_cloud = prev_cloud;
            //The bellow was found to work better
            reference_cloud = (corrected_accum_cloud_) ? corrected_accum_cloud_ : prev_cloud;

        }
    
        // Apply voxel grid filtering
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(cloud);
        voxel_filter.setLeafSize(0.05f, 0.05f, 0.5f);
        voxel_filter.filter(*cloud);
    
        // Run ICP
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(cloud);
        icp.setInputTarget(reference_cloud);
        icp.setRANSACIterations(50);
        icp.setMaxCorrespondenceDistance(0.15);
        icp.setMaximumIterations(500);
        icp.setTransformationEpsilon(1e-8);
    
        pcl::PointCloud<pcl::PointXYZ> aligned;
        icp.align(aligned);
    
        if (icp.hasConverged()) {
            Eigen::Matrix4f transformation = icp.getFinalTransformation();
            Eigen::Vector3f translation = transformation.block<3,1>(0,3);
            Eigen::Matrix3f rotation_matrix = transformation.block<3,3>(0,0);
            Eigen::Vector3f euler_angles = rotation_matrix.eulerAngles(2, 1, 0);
    
            float translation_magnitude = translation.norm();
            float rotation_magnitude = euler_angles.norm();
    
            if (translation_magnitude > MAX_TRANSLATION || rotation_magnitude > MAX_ROTATION) {
                RCLCPP_WARN(this->get_logger(), "ICP transformation exceeds limits! Skipping.");
                return;
            }
    
            // Publish transform if it's for accumulated point cloud
            if (output_topic == "/corrected_accumulated_pointcloud") {
                geometry_msgs::msg::TransformStamped transform_stamped;
                transform_stamped.header.frame_id = "map";
                transform_stamped.child_frame_id = "odom";
                transform_stamped.transform.translation.x = translation.x();
                transform_stamped.transform.translation.y = translation.y();
                transform_stamped.transform.translation.z = translation.z();
    
                tf2::Quaternion q;
                q.setRPY(euler_angles.x(), euler_angles.y(), euler_angles.z());
                transform_stamped.transform.rotation.x = q.x();
                transform_stamped.transform.rotation.y = q.y();
                transform_stamped.transform.rotation.z = q.z();
                transform_stamped.transform.rotation.w = q.w();
    
                tf_broadcaster_->sendTransform(transform_stamped);
            }
    
            sensor_msgs::msg::PointCloud2 output;
            pcl::toROSMsg(aligned, output);
            output.header = msg->header;
            output.header.frame_id = msg->header.frame_id;
    
            if (output_topic == "/corrected_accumulated_pointcloud") {
                // Update corrected accumulated cloud
                if (!corrected_accum_cloud_) {
                    corrected_accum_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(aligned);
                } else {
                    *corrected_accum_cloud_ += aligned;  // Accumulate new corrected points
                }
                publisher_accum_->publish(output);
            } else {
                prev_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(aligned);  // Update prev_nth_cloud_
                publisher_nth_->publish(output);
            }
    
            RCLCPP_INFO(this->get_logger(), "Published corrected point cloud to %s.", output_topic.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "ICP failed to converge on topic %s.", output_topic.c_str());
        }
    }


    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ICPProcessor>());
    rclcpp::shutdown();
    return 0;
}
