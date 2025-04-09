// Includes ROS2 and PCL libraries for working with point clouds and ICP.
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>  // ROS2 message type for PointCloud2
#include <pcl/point_cloud.h>                  // PointCloud container
#include <pcl/point_types.h>                  // PointXYZ type for 3D points
#include <pcl_conversions/pcl_conversions.h>  // For converting PCL to ROS PointCloud
#include <pcl/registration/icp.h>             // ICP (Iterative Closest Point) registration
#include <std_msgs/msg/float32.hpp>           // Float32 message type
#include <random>                             // For random selection

#include "tf2_ros/static_transform_broadcaster.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>  // Downsampling with Voxel Grid
Eigen::Matrix4f accumulated_transformation_ = Eigen::Matrix4f::Identity(); // Store cumulative transformation

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
    
        subscription_yaw_ = this->create_subscription<std_msgs::msg::Float32>(
            "/odom/yaw", 10,
            std::bind(&ICPProcessor::yaw_callback, this, std::placeholders::_1)
        );
        subscription_lin_ = this->create_subscription<std_msgs::msg::Float32>(
            "odom/lin", 10,
            std::bind(&ICPProcessor::lin_callback, this, std::placeholders::_1)
        );


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

    // Declare the yaw subscription
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_yaw_;
    // Declare the yaw subscription
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_lin_;


    const float MAX_TRANSLATION = 1.5;    // Maximum translation threshold (meters)
    const float MAX_ROTATION = 1;    // Maximum rotation threshold (radians) ~ 10 degrees

    
    float previous_yaw_{0.0f}; // Initialize to zero or a valid starting value
    float yaw_rate_{0.0f};     // Yaw rate (radians per second)
    float linv_{0.0f};     // linear velocity (m/s)

    const float smoothing_std = 0.25f;  

    float smoothing_factor = smoothing_std;  // Tune this value (e.g., 0.1 for gradual updates)
    


    pcl::PointCloud<pcl::PointXYZ>::Ptr corrected_accum_cloud_ {nullptr};  // Store corrected accumulated cloud

    void yaw_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        float current_yaw = msg->data;
        // Calculate yaw rate: (current_yaw - previous_yaw_) * (1 / dt), dt = 0.1 seconds so multiply by 10
        yaw_rate_ = (current_yaw - previous_yaw_) * 10.0f;
        
        // Update previous_yaw_ for the next callback
        previous_yaw_ = current_yaw;
    
        //RCLCPP_INFO(this->get_logger(), "Current yaw: %.2f, Yaw rate: %.2f", current_yaw, yaw_rate_);
    }

    void lin_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        linv_ = msg->data;
    }

    void publish_previous_cloud(const std::string &output_topic, 
        const sensor_msgs::msg::PointCloud2::SharedPtr msg,
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
        if (output_topic == "/corrected_accumulated_pointcloud") {
            // If no accumulated cloud exists, initialize it with the current cloud
            if (!corrected_accum_cloud_) {
                corrected_accum_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*cloud);
                RCLCPP_INFO(this->get_logger(), "Initialized accumulated cloud with the current cloud.");
            }

            // Publish the last valid accumulated cloud
            sensor_msgs::msg::PointCloud2 ros_cloud;
            pcl::toROSMsg(*corrected_accum_cloud_, ros_cloud);
            ros_cloud.header = msg->header;  // Maintain timestamp consistency
            publisher_accum_->publish(ros_cloud);
            RCLCPP_INFO(this->get_logger(), "Re-published last valid accumulated cloud.");
        }
        else {
            // Publish the last valid nth cloud without updating

            sensor_msgs::msg::PointCloud2 ros_cloud;
            pcl::toROSMsg(*cloud, ros_cloud);
            ros_cloud.header = msg->header;
            publisher_nth_->publish(ros_cloud);
            RCLCPP_INFO(this->get_logger(), "Re-published last valid nth cloud.");

        }
        
    }



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

        // Check yaw rate threshold before running ICP.
        const float YAW_RATE_THRESHOLD = 0.5; // Example threshold in rad/s
        int counter=0;

        //
        if ((std::abs(yaw_rate_) > abs(YAW_RATE_THRESHOLD)||abs(linv_) < 0.01)) {
            RCLCPP_WARN(this->get_logger(), "Yaw rate (%.2f rad/s) exceeds threshold or robot stationary(%.2f m/s): skipping ICP.", yaw_rate_, linv_);
            // For accumulated clouds:
            publish_previous_cloud(output_topic, msg, prev_cloud);
            
            if (std::abs(yaw_rate_) > abs(YAW_RATE_THRESHOLD)) {
            
                smoothing_factor = 0.01*counter;  // Disable smoothing for large yaw rates
                counter+=1;
                if (smoothing_factor > 0.5) {
                    smoothing_factor = 0.5;  // Cap the smoothing factor
                }

            }
            return; // Skip ICP processing
            
        }
        counter=0;
    
        // Determine reference cloud for ICP:
        pcl::PointCloud<pcl::PointXYZ>::Ptr reference_cloud;
        
        if (output_topic == "/corrected_accumulated_pointcloud") {
            // Use the previously corrected accumulated cloud, or fallback to prev_cloud if first run
            reference_cloud = (corrected_accum_cloud_) ? corrected_accum_cloud_ : prev_cloud;
            //reference_cloud = prev_cloud;
        } else {
            // Use the previous LiDAR scan for /nth_corrected_pointcloud
            //reference_cloud = prev_cloud;
            //The bellow was found to work better
            reference_cloud = (corrected_accum_cloud_) ? corrected_accum_cloud_ : prev_cloud;

        }
    
        // Apply voxel grid filtering
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(cloud);
        voxel_filter.setLeafSize(0.03f, 0.03f, 100.0f);
        voxel_filter.filter(*cloud);
    
        // Run ICP
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(cloud);
        icp.setInputTarget(reference_cloud);
        icp.setRANSACIterations(50);
        icp.setMaxCorrespondenceDistance(0.5);
        icp.setMaximumIterations(500);
        icp.setTransformationEpsilon(1e-8);
    
        pcl::PointCloud<pcl::PointXYZ> aligned;
        icp.align(aligned);
    
        if (icp.hasConverged()) {

            // Get the new ICP transformation.
            Eigen::Matrix4f transformation = icp.getFinalTransformation();
        
            // Extract translation and force z to zero.
            Eigen::Vector3f translation = transformation.block<3,1>(0,3);
            translation.z() = 0.0f;  // Ensure no vertical translation
        
            // Extract rotation and convert to Euler angles (Z, Y, X order).
            Eigen::Matrix3f rotation_matrix = transformation.block<3,3>(0,0);
            Eigen::Vector3f euler_angles = rotation_matrix.eulerAngles(2, 1, 0); // [yaw, pitch, roll]
        
            // Only use the yaw (z-axis rotation)
            Eigen::Matrix3f z_only_rot = Eigen::AngleAxisf(euler_angles.x(), Eigen::Vector3f::UnitZ()).toRotationMatrix();
        
            // Build an adjusted transformation with the modified translation and z-axis rotation.
            Eigen::Matrix4f adjusted_transformation = Eigen::Matrix4f::Identity();
            adjusted_transformation.block<3,3>(0,0) = z_only_rot;
            adjusted_transformation.block<3,1>(0,3) = translation;
        
            // Optionally check the magnitude of the adjusted transformation.
            float translation_magnitude = translation.norm();
            float rotation_magnitude = std::abs(euler_angles.x());
            RCLCPP_INFO(this->get_logger(), "Translation mag: %f, Yaw mag: %f", translation_magnitude, rotation_magnitude);
        
            // Only proceed if within limits (your condition here; adjust thresholds as needed). && output_topic != "/corrected_accumulated_pointcloud"
            if (translation_magnitude < MAX_TRANSLATION && rotation_magnitude < MAX_ROTATION )  {
        
                // Smoothing factor: 0.0 means no update; 1.0 means full update.
                
        
                // Get the current accumulated translation and yaw.
                Eigen::Vector3f prev_translation = accumulated_transformation_.block<3,1>(0,3);
                Eigen::Matrix3f prev_rotation = accumulated_transformation_.block<3,3>(0,0);
                float prev_yaw = prev_rotation.eulerAngles(2, 1, 0).x();
        
                // New update from ICP.
                Eigen::Vector3f new_translation = translation;
                float new_yaw = euler_angles.x();  // Only the z-axis rotation
        
                // Smooth the updates by blending the previous state and the new update.
                Eigen::Vector3f smoothed_translation = (1.0f - smoothing_factor*1.5) * prev_translation + smoothing_factor * new_translation;
                float smoothed_yaw = (1.0f - smoothing_factor*1.5) * prev_yaw + smoothing_factor * new_yaw;

                smoothing_factor = smoothing_std;  // Reset to default value
        
                // Build the new (smoothed) accumulated transformation.
                Eigen::Matrix4f smoothed_transform = Eigen::Matrix4f::Identity();
                smoothed_transform.block<3,3>(0,0) = Eigen::AngleAxisf(smoothed_yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();
                smoothed_transform.block<3,1>(0,3) = smoothed_translation;
                accumulated_transformation_ = smoothed_transform;
        
                // Publish the smoothed transform.
                geometry_msgs::msg::TransformStamped transform_stamped;
                transform_stamped.header.frame_id = "map";
                transform_stamped.child_frame_id = "odom";
        
                // Use the smoothed translation; ensure z remains zero.
                transform_stamped.transform.translation.x = smoothed_translation.x();
                transform_stamped.transform.translation.y = smoothed_translation.y();
                transform_stamped.transform.translation.z = 0.0f;
        
                // Build a quaternion from the smoothed yaw.
                tf2::Quaternion q;
                q.setRPY(0.0, 0.0, smoothed_yaw);
                transform_stamped.transform.rotation.x = q.x();
                transform_stamped.transform.rotation.y = q.y();
                transform_stamped.transform.rotation.z = q.z();
                transform_stamped.transform.rotation.w = q.w();
        
                tf_broadcaster_->sendTransform(transform_stamped);
            }
        /*
            
            // Get the new ICP transformation.
            Eigen::Matrix4f transformation = icp.getFinalTransformation();
             // Extract translation and force z to zero.
            Eigen::Vector3f translation = transformation.block<3,1>(0,3);
            translation.z() = 0.0f;  // Zero out vertical translation.
            
            // Extract rotation matrix and convert to Euler angles (Z, Y, X order).
            Eigen::Matrix3f rotation_matrix = transformation.block<3,3>(0,0);
            Eigen::Vector3f euler_angles = rotation_matrix.eulerAngles(2, 1, 0); // [yaw, pitch, roll]
            
            // Only use the yaw (rotation about Z) by creating a rotation matrix that ignores x and y rotations.
            Eigen::Matrix3f z_only_rot = Eigen::AngleAxisf(euler_angles.x(), Eigen::Vector3f::UnitZ()).toRotationMatrix();
            
            // Build an adjusted transformation with the modified translation and only z-axis rotation.
            Eigen::Matrix4f adjusted_transformation = Eigen::Matrix4f::Identity();
            adjusted_transformation.block<3,3>(0,0) = z_only_rot;
            adjusted_transformation.block<3,1>(0,3) = translation;
            
            // Optionally, check the magnitude of the adjusted transformation.
            float translation_magnitude = translation.norm();
            float rotation_magnitude = std::abs(euler_angles.x());
            RCLCPP_INFO(this->get_logger(), "Translation mag: %f, Yaw mag: %f", translation_magnitude, rotation_magnitude);

            Eigen::Vector3f translation = accumulated_transformation_.block<3,1>(0,3);
            Eigen::Matrix3f rotation_matrix = accumulated_transformation_.block<3,3>(0,0);
            Eigen::Vector3f euler_angles = rotation_matrix.eulerAngles(2, 1, 0);
    
            float translation_magnitude = translation.norm();
            float rotation_magnitude = euler_angles.norm();
            RCLCPP_INFO(this->get_logger(), "Translation mag: %f., Rotation mag: %f", translation_magnitude,rotation_magnitude);

    
            if (translation_magnitude <MAX_TRANSLATION || rotation_magnitude < MAX_ROTATION) {

                accumulated_transformation_ = accumulated_transformation_*adjusted_transformation ;
            // Publish transform if it's for accumulated point cloud
            //if (output_topic == "/corrected_accumulated_pointcloud") {
                geometry_msgs::msg::TransformStamped transform_stamped;
                transform_stamped.header.frame_id = "map";
                transform_stamped.child_frame_id = "odom";
                transform_stamped.transform.translation.x = translation.x();
                transform_stamped.transform.translation.y = translation.y();
                transform_stamped.transform.translation.z = 0;//translation.z();
    
                tf2::Quaternion q;
                q.setRPY(euler_angles.x(), euler_angles.y(), euler_angles.z());
                transform_stamped.transform.rotation.x = 0.0;//q.x();
                transform_stamped.transform.rotation.y = 0.0;//q.y();
                transform_stamped.transform.rotation.z = q.z();
                transform_stamped.transform.rotation.w = q.w();
    
                tf_broadcaster_->sendTransform(transform_stamped);
            //}
            }
            
            
            */
            
    
    
            sensor_msgs::msg::PointCloud2 output;
            pcl::toROSMsg(aligned, output);
            output.header = msg->header;
            output.header.frame_id = msg->header.frame_id;
    
            if (output_topic == "/corrected_accumulated_pointcloud") {
                // Update corrected accumulated cloud
                if (!corrected_accum_cloud_) {
                    corrected_accum_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(aligned);
                } else {
                    *corrected_accum_cloud_ = aligned;  // Accumulate new corrected points
                }
                publisher_accum_->publish(output);
                corrected_accum_cloud_ = nullptr;  // Reset the corrected accumulated cloud
            } else {
                prev_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(aligned);  // Update prev_nth_cloud_
                publisher_nth_->publish(output);
            }
    
            RCLCPP_INFO(this->get_logger(), "Published corrected point cloud to %s.", output_topic.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "ICP failed to converge on topic %s.", output_topic.c_str());
            prev_cloud = nullptr;  // Reset the previous cloud to force a new alignment
            corrected_accum_cloud_ = nullptr;  // Reset the corrected accumulated cloud
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