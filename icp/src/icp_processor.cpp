// Includes ROS2 and PCL libraries for working with point clouds and ICP.
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>  // ROS2 message type for PointCloud2
#include <pcl/point_cloud.h>                  // PointCloud container
#include <pcl/point_types.h>                  // PointXYZ type for 3D points
#include <pcl_conversions/pcl_conversions.h>  // For converting PCL to ROS PointCloud
#include <pcl/registration/icp.h>             // ICP (Iterative Closest Point) registration
#include <std_msgs/msg/float32.hpp>           // Float32 message type
#include <random>             
#include <mutex>
#include <boost/make_shared.hpp>              // <-- for boost::make_shared
#include "std_msgs/msg/bool.hpp"  // Note: use std_msgs, not standard_msgs


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

        reference_asker_ = this->create_publisher<std_msgs::msg::Bool>("/give_reference", 10);

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
        subscription_correction_ = this->create_subscription<std_msgs::msg::Float32>(
            "/correction", 10,
            std::bind(&ICPProcessor::corr_callback, this, std::placeholders::_1)
        );


        //Publish static transform
        tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);
        //Publish dynamic transform
        //tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_nth_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_nth_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reference_asker_;

    // Subscriber for the reference cloud.
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_reference_;

    // ...
    std::mutex reference_mutex_;
    // For random selection

    // Declare the yaw subscription
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_yaw_;
    // Declare the yaw subscription
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_lin_;
    // Declare the correction subscription
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_correction_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr reference_cloud_{std::make_shared<pcl::PointCloud<pcl::PointXYZ>>()};
    // Store the reference keyframes
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> reference_keyframes_;

    
    pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud {nullptr};

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_;


    const float MAX_TRANSLATION = 0.43;    // Maximum translation threshold (meters)
    const float MAX_ROTATION = 0.80;    // Maximum rotation threshold (radians) ~ 10 degrees
    const float voxel_leaf_size_ = 0.01f; // Voxel grid leaf size (meters)
    const float max_corr_dist_ = 0.3f; // Maximum correspondence distance (meters)
    long unsigned int MAX_COR = 20; // Maximum number of correspondences to consider
    const int max_iter_ = 50; // Maximum number of ICP iterations

    float corr_{0.0f}; // correction value
    float previous_yaw_{0.0f}; // Initialize to zero or a valid starting value
    float yaw_rate_{0.0f};     // Yaw rate (radians per second)
    float linv_{0.0f};     // linear velocity (m/s)

    const float smoothing_std = 0.2f; 
    // Check yaw rate threshold before running ICP.
    const float YAW_RATE_THRESHOLD = 0.2; // Example threshold in rad/s
    int not_enough_counter_=0;
    int stationary_counter=0;
    bool no_icp_=false; //used to disable icp when needed

    float smoothing_factor = smoothing_std;  // Tune this value (e.g., 0.1 for gradual updates)
    

   void reference_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {

        auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::fromROSMsg(*msg, *cloud);

        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, 1000.0f);
        voxel_filter.setInputCloud(cloud);
        voxel_filter.filter(*cloud);

        std::lock_guard<std::mutex> lock(reference_mutex_);

        bool reset_icp = false;

        

        if (reference_keyframes_.empty()) {
            // First reference cloud
            RCLCPP_INFO(this->get_logger(), "First reference cloud of size : %zu points, added to list.", cloud->size());
            reference_keyframes_.push_back(std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*cloud));

        } else {
            // Check if the cloud is already in the reference
            bool cloud_exists = false;
            for (const auto& ref : reference_keyframes_) {
                if (ref->size() == cloud->size()) {
                    cloud_exists = true;
                    break;
                }
            }
            
            if (!cloud_exists) {
                not_enough_counter_=0;
                RCLCPP_INFO(this->get_logger(), "New reference cloud of size : %zu points, received.", cloud->size());


                /*
                
                //#############################################################
                // Check if the new cloud is similar to the last reference cloud
                //correspondence check:
                int best_match_index = -1;
                size_t max_correspondences = 0;

                // Temporary correspondence estimation object
                pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> est;
                est.setInputSource(cloud);  // The new input cloud


                for (size_t i = 0; i < reference_keyframes_.size(); ++i) {
                    auto& ref = reference_keyframes_[i];
                    RCLCPP_WARN(this->get_logger(), "In loop");

                    if (!ref || ref->empty()) {
                        RCLCPP_WARN(this->get_logger(), "Keyframe %zu is empty or invalid!", i);
                        continue;
                    }

                    est.setInputTarget(ref);
                    pcl::Correspondences correspondences;
                    est.determineCorrespondences(correspondences, max_corr_dist_);  // Use your matching threshold here

                    RCLCPP_INFO(this->get_logger(), "Keyframe %zu has %zu correspondences, of so many points: %zu ", i, correspondences.size(),ref->size());

                    if (correspondences.size() > max_correspondences) {
                        max_correspondences = correspondences.size();
                        best_match_index = static_cast<int>(i);
                    }
                }

                if (MAX_COR/2 > max_correspondences) {
                    RCLCPP_WARN(this->get_logger(), "No valid correspondences found with any reference for new keyframe, just adding.");
                    //push back the corrected keyframe
                    reference_keyframes_.push_back(std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*cloud));
                    return;
                }

                reference_cloud_ = reference_keyframes_[best_match_index];
                RCLCPP_WARN(this->get_logger(), "New reference scan selected keyframe %d with %zu correspondences as ICP target.", best_match_index, max_correspondences);
                //#############################################################
                // Run ICP
                //pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_;
                icp_.setInputSource(cloud);
                icp_.setInputTarget(reference_cloud_);
                icp_.setMaxCorrespondenceDistance(max_corr_dist_);
                icp_.setMaximumIterations(max_iter_);
                icp_.setTransformationEpsilon(1e-8);
            
                auto aligned = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
                icp_.align(*aligned);

                //#############################################################
                //push back the corrected keyframe
                
                
                reference_keyframes_.push_back(std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*aligned));
                */
                reference_keyframes_.push_back(std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*cloud));

                reset_icp = true;
            } else {
                RCLCPP_WARN(this->get_logger(), "Reference cloud size is equal to some previous cloud size: %zu points.", cloud->size());
                return;
            }
        }

        if (reset_icp) {
            // Hard reset ICP
            //icp_ = pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>();
            RCLCPP_WARN(this->get_logger(), "Resetting ICP with new reference cloud.");
            icp_.setMaxCorrespondenceDistance(max_corr_dist_);
            icp_.setMaximumIterations(500);
            icp_.setInputTarget(reference_keyframes_.back());
            icp_.setTransformationEpsilon(1e-8);
        }
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

    void corr_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        corr_ = msg->data;
        //accumulated_transformation_(0,3) += -corr_;
        no_icp_=true;

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
            return; // Skip ICP processing
        }
        
        
        if (std::abs(linv_) < 0.01) {
            stationary_counter++;
            if (stationary_counter%4==0) {
                RCLCPP_WARN(this->get_logger(), "Robot has been stationary for 4 collections (%.2f m/s): performing ICP.", linv_);
                stationary_counter=0;  // Reset to default value
                
            }
            else{
                RCLCPP_WARN(this->get_logger(), "Robot is stationary (%.2f m/s): skiping %d st/nd ICP.", linv_,stationary_counter+1);
                publish_previous_cloud(output_topic, msg, prev_cloud);
                return; // Skip ICP processing
            }
        }
        else {
            stationary_counter=0;  // Reset to default value
        }

        

        for (auto& pt : cloud->points) {
            pt.z = 0.0f;
        }

    
        // Apply voxel grid filtering
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(cloud);
        voxel_filter.setLeafSize(voxel_leaf_size_,voxel_leaf_size_, 1000.0f);
        voxel_filter.filter(*cloud);

        //correspondence check:
        int best_match_index = -1;
        size_t max_correspondences = 0;

        // Temporary correspondence estimation object
        pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> est;
        est.setInputSource(cloud);  // The new input cloud

        std::lock_guard<std::mutex> lock(reference_mutex_);
        // Iterate through keyframes
        for (size_t i = 0; i < reference_keyframes_.size(); ++i) {
            auto& ref = reference_keyframes_[i];

            if (!ref || ref->empty()) {
                RCLCPP_WARN(this->get_logger(), "Keyframe %zu is empty or invalid!", i);
                continue;
            }

            est.setInputTarget(ref);
            pcl::Correspondences correspondences;
            est.determineCorrespondences(correspondences, max_corr_dist_);  // Use your matching threshold here

            RCLCPP_INFO(this->get_logger(), "Keyframe %zu has %zu correspondences, of so many points: %zu ", i, correspondences.size(),ref->size());

            if (correspondences.size() > max_correspondences) {
                max_correspondences = correspondences.size();
                best_match_index = static_cast<int>(i);
            }
        }
        /*
        if (best_match_index == -1) {
            RCLCPP_WARN(this->get_logger(), "No valid correspondences found with any reference.");
            publish_previous_cloud(output_topic, msg, prev_cloud);
            return;
        }
        */

        if (max_correspondences < MAX_COR) {
            RCLCPP_WARN(this->get_logger(), "%zu correspondences found, not enough.",max_correspondences);
            //publish_previous_cloud(output_topic, msg, cloud);t
            not_enough_counter_++;

            if (not_enough_counter_%3==0){
                RCLCPP_WARN(this->get_logger(), "Not enough correspondences found continually, asking for new ref.");
                not_enough_counter_=0;  // Reset to default value
                // Somewhere in your code, when you want to publish 'true' or 'false':
                std_msgs::msg::Bool msg;
                msg.data = true;  // or false
                reference_asker_->publish(msg);
            }
            return;
        }

        reference_cloud_ = reference_keyframes_[best_match_index];
        RCLCPP_INFO(this->get_logger(), "Selected keyframe %d with %zu correspondences as ICP target.", best_match_index, max_correspondences);



    
        // Run ICP
        //pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_;
        icp_.setInputSource(cloud);
        icp_.setInputTarget(reference_cloud_);
        icp_.setRANSACIterations(max_iter_);
        icp_.setMaxCorrespondenceDistance(max_corr_dist_);
        icp_.setMaximumIterations(500);
        icp_.setTransformationEpsilon(1e-8);
    
        pcl::PointCloud<pcl::PointXYZ> aligned;
        icp_.align(aligned);

        //TODO figure out way to pause ICP when in top box, whislt also performing the correction at least once
        /*
        if (corr_==0.12){
            no_icp_=false;
        }
            && !no_icp_
        */
        
        if (icp_.hasConverged() ) {
            
            /*
                        float icpfitness = icp.getFitnessScore();
            
            //RCLCPP_INFO(this->get_logger(), "Fittness score: %f,", icpfitness);
            if (icpfitness < 0.1) {
                RCLCPP_WARN(this->get_logger(), "ICP fitness score is too low: %f, skipping update.", icpfitness);
                // Publish the latest cloud without updating
                publish_previous_cloud(output_topic, msg, cloud);
                return; // Skip ICP processing
            }

            */


            // Get the new ICP transformation.
            Eigen::Matrix4f transformation = icp_.getFinalTransformation();
        
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

            //
            bool over_rotation= false;
            if ((translation_magnitude < MAX_TRANSLATION && rotation_magnitude >MAX_ROTATION)) {
                over_rotation= true;
            }




            if ((translation_magnitude < MAX_TRANSLATION && rotation_magnitude < MAX_ROTATION)||over_rotation) {
        
                // Smoothing factor: 0.0 means no update; 1.0 means full update.
                
        
                // Get the current accumulated translation and yaw.
                Eigen::Vector3f prev_translation = accumulated_transformation_.block<3,1>(0,3);
                Eigen::Matrix3f prev_rotation = accumulated_transformation_.block<3,3>(0,0);
                float prev_yaw = prev_rotation.eulerAngles(2, 1, 0).x();
        
                // New update from ICP.
                Eigen::Vector3f new_translation = translation;
                float new_yaw ;
                if (over_rotation){
                    new_yaw= prev_yaw;  // Retain previous yaw
                }
                else{
                    new_yaw = euler_angles.x();  // Only the z-axis rotation
                }
                
                

            
                // Build the new (smoothed) accumulated transformation.
                Eigen::Matrix4f smoothed_transform = Eigen::Matrix4f::Identity();
                Eigen::Vector3f smoothed_translation;
                float smoothed_yaw;
                
               
                // Smooth the updates by blending the previous state and the new update.
                smoothed_translation = (1.0f - smoothing_factor) * prev_translation + smoothing_factor * new_translation;
                smoothed_yaw = (1.0f - smoothing_factor) * prev_yaw + smoothing_factor * new_yaw;

                smoothing_factor = smoothing_std;  // Reset to default value         
                RCLCPP_INFO(this->get_logger(), "Applied Standard transformation: translation (%.2f, %.2f, %.2f), yaw (%.2f)", smoothed_translation.x(), smoothed_translation.y(), smoothed_translation.z(), smoothed_yaw);
        

                smoothed_transform.block<3,1>(0,3) = smoothed_translation;
                smoothed_transform.block<3,3>(0,0) = Eigen::AngleAxisf(smoothed_yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();
                accumulated_transformation_ = smoothed_transform;


                // Publish the smoothed transform.
                geometry_msgs::msg::TransformStamped transform_stamped;
                transform_stamped.header.frame_id = "map";
                transform_stamped.child_frame_id = "odom";
        
                // Use the smoothed translation; ensure z remains zero.
                transform_stamped.transform.translation.x = smoothed_translation.x() - corr_;
                transform_stamped.transform.translation.y = smoothed_translation.y();
                transform_stamped.transform.translation.z = 0.0f;
        

                //TODO figure out way to pause ICP when in top box, whislt also performing the correction at least once
                /*
                if (corr_==0.25){
                    no_icp_=true;
                }
                */
                
                // Build a quaternion from the smoothed yaw.
                tf2::Quaternion q;
                q.setRPY(0.0, 0.0, smoothed_yaw);
                transform_stamped.transform.rotation.x = q.x();
                transform_stamped.transform.rotation.y = q.y();
                transform_stamped.transform.rotation.z = q.z();
                transform_stamped.transform.rotation.w = q.w();
        
                tf_broadcaster_->sendTransform(transform_stamped);

                sensor_msgs::msg::PointCloud2 output;
                pcl::toROSMsg(aligned, output);
                output.header = msg->header;
                output.header.frame_id = msg->header.frame_id;
        
                publisher_nth_->publish(output);
                *prev_cloud = aligned;  // Update the previous cloud with the current one
    
                RCLCPP_INFO(this->get_logger(), "Published corrected point cloud to %s.", output_topic.c_str());

            }else {
                RCLCPP_WARN(this->get_logger(), "ICP transform too large filtering out.");
                // Publish the last valid nth cloud without updating
                //publish_previous_cloud(output_topic, msg, prev_cloud);
                //prev_cloud = nullptr;  // Reset the previous cloud to force a new alignment
                //corrected_accum_cloud_ = nullptr;  // Reset the corrected accumulated cloud
            }

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
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<ICPProcessor>();
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}