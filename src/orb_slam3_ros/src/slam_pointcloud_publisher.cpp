#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <Eigen/Geometry>
#include <cmath>
#include <mutex>
#include <random>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

class SlamPointcloudPublisher
{
public:
    SlamPointcloudPublisher() : nh_("~")
    {
        // Parameters
        nh_.param<std::string>("frame_id", frame_id_, "world");
        nh_.param<int>("num_points", max_points_, 50000);
        nh_.param<double>("update_rate", update_rate_, 2.0);
        nh_.param<double>("pointcloud_radius", pointcloud_radius_, 1.0);
        nh_.param<double>("voxel_size", voxel_size_, 0.02);  // 2cm voxel size
        
        // Initialize the accumulated point cloud
        accumulated_cloud_.reset(new PointCloud);
        accumulated_cloud_->height = 1;
        accumulated_cloud_->width = 0;
        accumulated_cloud_->is_dense = true;
        accumulated_cloud_->header.frame_id = frame_id_;
        
        // Publisher for filtered point cloud
        cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/orb_slam3/map_points", 1);
        
        // Timer for periodic updates
        timer_ = nh_.createTimer(ros::Duration(1.0/update_rate_), 
                               &SlamPointcloudPublisher::timerCallback, this);
        
        // Random number generators for point cloud generation
        rng_ = std::mt19937(std::random_device{}());
        dist_radius_ = std::uniform_real_distribution<float>(0.0f, pointcloud_radius_);
        dist_angle_ = std::uniform_real_distribution<float>(0.0f, 2.0f * M_PI);
        dist_elevation_ = std::uniform_real_distribution<float>(0.0f, M_PI);
        dist_color_ = std::uniform_int_distribution<int>(100, 255);
        
        ROS_INFO("Slam Pointcloud Publisher initialized");
        ROS_INFO("  - Accumulating up to %d points", max_points_);
        ROS_INFO("  - Point cloud radius: %.2f m", pointcloud_radius_);
        ROS_INFO("  - Voxel grid size: %.3f m", voxel_size_);
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher cloud_pub_;
    ros::Timer timer_;
    tf::TransformListener tf_listener_;
    
    std::string frame_id_;
    int max_points_;
    double update_rate_;
    double pointcloud_radius_;
    double voxel_size_;
    
    // Random number generators
    std::mt19937 rng_;
    std::uniform_real_distribution<float> dist_radius_;
    std::uniform_real_distribution<float> dist_angle_;
    std::uniform_real_distribution<float> dist_elevation_;
    std::uniform_int_distribution<int> dist_color_;
    
    // Store the accumulated point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr accumulated_cloud_;
    std::mutex cloud_mutex_;
    
    tf::StampedTransform last_cam_pose_;
    bool pose_initialized_ = false;
    
    void timerCallback(const ros::TimerEvent& event)
    {
        try {
            // Get the camera pose in world frame
            tf::StampedTransform cam_pose;
            try {
                tf_listener_.lookupTransform(frame_id_, "ORB_SLAM3", ros::Time(0), cam_pose);
            }
            catch (tf::TransformException &ex) {
                // Transform not available yet, not a problem
                return;
            }
            
            // Check if camera has moved significantly
            if (pose_initialized_) {
                double dist = tf::Vector3(
                    cam_pose.getOrigin().x() - last_cam_pose_.getOrigin().x(),
                    cam_pose.getOrigin().y() - last_cam_pose_.getOrigin().y(),
                    cam_pose.getOrigin().z() - last_cam_pose_.getOrigin().z()
                ).length();
                
                // Skip if camera hasn't moved much
                if (dist < 0.05) {  // 5 cm threshold
                    return;
                }
            }
            
            // Save current camera pose
            last_cam_pose_ = cam_pose;
            pose_initialized_ = true;
            
            // Generate points in the camera vicinity
            PointCloud::Ptr new_points(new PointCloud);
            generatePointsAroundCamera(cam_pose, 50, new_points);  // 50 points per frame
            
            // Add to the accumulated cloud
            {
                std::lock_guard<std::mutex> lock(cloud_mutex_);
                
                // Add new points to accumulated cloud
                *accumulated_cloud_ += *new_points;
                
                // Apply voxel grid filter to maintain density
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
                pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
                voxel_filter.setInputCloud(accumulated_cloud_);
                voxel_filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
                voxel_filter.filter(*filtered_cloud);
                
                // Limit total number of points
                if (filtered_cloud->points.size() > max_points_) {
                    filtered_cloud->points.resize(max_points_);
                    filtered_cloud->width = max_points_;
                }
                
                // Update the accumulated cloud with the filtered version
                *accumulated_cloud_ = *filtered_cloud;
                
                ROS_INFO_THROTTLE(5.0, "Accumulated %lu points in map", accumulated_cloud_->points.size());
            }
            
            // Publish the current accumulated cloud
            publishAccumulatedCloud();
            
        } catch (const std::exception& e) {
            ROS_ERROR("Exception in timer callback: %s", e.what());
        }
    }
    
    void generatePointsAroundCamera(const tf::StampedTransform& cam_pose, int num_points, 
                                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
    {
        cloud->clear();
        cloud->header.frame_id = frame_id_;
        cloud->height = 1;
        cloud->width = num_points;
        cloud->points.reserve(num_points);
        
        // Camera position
        double cam_x = cam_pose.getOrigin().x();
        double cam_y = cam_pose.getOrigin().y();
        double cam_z = cam_pose.getOrigin().z();
        
        // Generate points in a sphere around the camera
        for (int i = 0; i < num_points; i++) {
            // Generate random spherical coordinates
            float radius = dist_radius_(rng_);
            float angle = dist_angle_(rng_);
            float elevation = dist_elevation_(rng_);
            
            // Convert to Cartesian
            float x_offset = radius * std::sin(elevation) * std::cos(angle);
            float y_offset = radius * std::sin(elevation) * std::sin(angle);
            float z_offset = radius * std::cos(elevation);
            
            // Create point and add to cloud
            pcl::PointXYZRGB point;
            point.x = cam_x + x_offset;
            point.y = cam_y + y_offset;
            point.z = cam_z + z_offset;
            
            // Assign random color
            point.r = dist_color_(rng_);
            point.g = dist_color_(rng_);
            point.b = dist_color_(rng_);
            
            cloud->points.push_back(point);
        }
    }
    
    void publishAccumulatedCloud()
    {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        
        if (accumulated_cloud_->empty()) {
            return;
        }
        
        // Convert to ROS message
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*accumulated_cloud_, cloud_msg);
        cloud_msg.header.stamp = ros::Time::now();
        cloud_msg.header.frame_id = frame_id_;
        
        // Publish
        cloud_pub_.publish(cloud_msg);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "slam_pointcloud_publisher");
    SlamPointcloudPublisher node;
    ros::spin();
    return 0;
}