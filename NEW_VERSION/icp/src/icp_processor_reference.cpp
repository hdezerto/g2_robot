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
                pointcloud_callback(msg, "/nth_corrected_pointcloud",prev_cloud);
            }
        );
        publisher_nth_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/nth_corrected_pointcloud", 10);

        // Subscribe to the reference cloud topic
        subscription_reference_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/reference_cloud", 1,
            std::bind(&ICPProcessor::reference_callback, this, std::placeholders::_1)
        );
    
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

    // Subscriber for the reference cloud.
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_reference_;


    // Declare the yaw subscription
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_yaw_;
    // Declare the yaw subscription
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_lin_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr reference_cloud_{std::make_shared<pcl::PointCloud<pcl::PointXYZ>>()};
    pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud {nullptr};

    const float MAX_TRANSLATION = 1.5;    // Maximum translation threshold (meters)
    const float MAX_ROTATION = 1;    // Maximum rotation threshold (radians) ~ 10 degrees

    
    float previous_yaw_{0.0f}; // Initialize to zero or a valid starting value
    float yaw_rate_{0.0f};     // Yaw rate (radians per second)
    float linv_{0.0f};     // linear velocity (m/s)

    const float smoothing_std = 0.2f; 
    // Check yaw rate threshold before running ICP.
    const float YAW_RATE_THRESHOLD = 0.45; // Example threshold in rad/s
    int counter=0;
    int stationary_counter=0;
    
    float smoothing_factor = smoothing_std;  // Tune this value (e.g., 0.1 for gradual updates)
    

    // Callback for the reference cloud subscriber.
    void reference_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // Convert the incoming PointCloud2 message into a PCL point cloud and store it.
        pcl::fromROSMsg(*msg, *reference_cloud_);
        RCLCPP_INFO(this->get_logger(), "Reference cloud updated with %zu points", reference_cloud_->size());
    }


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
        // Publish the last valid nth cloud without updating
        if (!cloud) {
            RCLCPP_WARN(this->get_logger(), "No valid cloud to publish for topic %s.", output_topic.c_str());
            return;
        }
        
        sensor_msgs::msg::PointCloud2 ros_cloud;
        pcl::toROSMsg(*cloud, ros_cloud);
        ros_cloud.header = msg->header;
        publisher_nth_->publish(ros_cloud);
        RCLCPP_INFO(this->get_logger(), "Re-published last valid nth cloud.");
        
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



        //||abs(linv_) < 0.01
        if ((std::abs(yaw_rate_) > abs(YAW_RATE_THRESHOLD))) {
            RCLCPP_WARN(this->get_logger(), "Yaw rate (%.2f rad/s) exceeds threshold or robot stationary(%.2f m/s): skipping ICP.", yaw_rate_, linv_);
            // For accumulated clouds:
            publish_previous_cloud(output_topic, msg, prev_cloud);
            
            if (std::abs(yaw_rate_) > abs(YAW_RATE_THRESHOLD)) {
                /*
                                                smoothing_factor = 0.01*counter;  // Disable smoothing for large yaw rates
                counter+=1;
                if (smoothing_factor > 0.5) {
                    smoothing_factor = 0.5;  // Cap the smoothing factor
                }
                */

            }
            return; // Skip ICP processing
            
        }
        else {
            smoothing_factor = smoothing_std;  // Reset to default value
        }
        counter=0;
        if (std::abs(linv_) < 0.01) {
            stationary_counter++;
            if (stationary_counter%3 == 0) {
                RCLCPP_WARN(this->get_logger(), "Robot has been stationary for three collections (%.2f m/s): performing ICP.", linv_);
                

            } 
            RCLCPP_WARN(this->get_logger(), "Robot is stationary (%.2f m/s): skiping first ICP.", linv_);
            publish_previous_cloud(output_topic, msg, prev_cloud);
            return; // Skip ICP processing
        }
        else {
            stationary_counter=0;  // Reset to default value
        }

        

        // Check reference cloud availability
        if (reference_cloud_ && !reference_cloud_->empty()) {
            //target_cloud = reference_cloud_;
        } else {
            return;
        }

    
        // Apply voxel grid filtering
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(cloud);
        voxel_filter.setLeafSize(0.01f, 0.01f, 100.0f);
        voxel_filter.filter(*cloud);
    
        // Run ICP
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(cloud);
        icp.setInputTarget(reference_cloud_);
        icp.setRANSACIterations(50);
        icp.setMaxCorrespondenceDistance(0.5);
        icp.setMaximumIterations(500);
        icp.setTransformationEpsilon(1e-8);
    
        pcl::PointCloud<pcl::PointXYZ> aligned;
        icp.align(aligned);
    
        if (icp.hasConverged()) {
            float icpfitness = icp.getFitnessScore();
            RCLCPP_INFO(this->get_logger(), "Fittness score: %f,", icpfitness);

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
        
            // Only proceed if within limits (your condition here; adjust thresholds as needed).
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
            
    
    
            sensor_msgs::msg::PointCloud2 output;
            pcl::toROSMsg(aligned, output);
            output.header = msg->header;
            output.header.frame_id = msg->header.frame_id;
    
            publisher_nth_->publish(output);
            *prev_cloud = aligned;  // Update the previous cloud with the current one

            RCLCPP_INFO(this->get_logger(), "Published corrected point cloud to %s.", output_topic.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "ICP failed to converge on topic %s.", output_topic.c_str());
            // Publish the last valid nth cloud without updating
            publish_previous_cloud(output_topic, msg, prev_cloud);
            //prev_cloud = nullptr;  // Reset the previous cloud to force a new alignment
            //corrected_accum_cloud_ = nullptr;  // Reset the corrected accumulated cloud
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