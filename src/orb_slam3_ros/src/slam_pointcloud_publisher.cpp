#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <eigen3/Eigen/Geometry>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

class SlamPointcloudPublisher
{
public:
    SlamPointcloudPublisher() : nh_("~")
    {
        // Parameters
        nh_.param<std::string>("frame_id", frame_id_, "world");
        nh_.param<int>("max_points", max_points_, 30000);
        nh_.param<double>("update_rate", update_rate_, 5.0);
        nh_.param<double>("voxel_size", voxel_size_, 0.02); // 2cm voxel size for downsampling
        nh_.param<double>("radius", radius_, 1.0); // Radius for synthetic points
        
        // Initialize point cloud
        cloud_.reset(new PointCloud);
        cloud_->height = 1;
        cloud_->width = 0;
        cloud_->is_dense = true;
        cloud_->header.frame_id = frame_id_;
        
        // Publisher for filtered point cloud
        cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/orb_slam3/map_points", 1);
        
        // Timer for updates
        timer_ = nh_.createTimer(ros::Duration(1.0/update_rate_), 
                               &SlamPointcloudPublisher::timerCallback, this);
        
        ROS_INFO("Slam Pointcloud Publisher initialized with synthetic points");
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher cloud_pub_;
    ros::Timer timer_;
    tf::TransformListener tf_listener_;
    
    std::string frame_id_;
    int max_points_;
    double update_rate_;
    double voxel_size_;
    double radius_;
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;
    tf::StampedTransform latest_pose_;
    bool pose_initialized_ = false;
    
    void timerCallback(const ros::TimerEvent& event)
    {
        try {
            // Try to get the latest transform from ORB-SLAM3
            tf_listener_.lookupTransform(frame_id_, "ORB_SLAM3", ros::Time(0), latest_pose_);
            pose_initialized_ = true;
            
            // Add camera position and surrounding points to cloud
            addCameraAndSurroundingPoints();
            
            // Filter and publish the updated cloud
            filterAndPublish();
        }
        catch (tf::TransformException &ex) {
            // This is fine - we just wait for the transform to become available
        }
    }
    
    void addCameraAndSurroundingPoints()
    {
        if (!pose_initialized_)
            return;
        
        // Get camera position
        double x = latest_pose_.getOrigin().x();
        double y = latest_pose_.getOrigin().y();
        double z = latest_pose_.getOrigin().z();
        
        // Add camera position to cloud
        pcl::PointXYZRGB camera_point;
        camera_point.x = x;
        camera_point.y = y;
        camera_point.z = z;
        camera_point.r = 255;
        camera_point.g = 0;
        camera_point.b = 0;
        
        // Only add if it's a new position (moved more than 1cm)
        bool add_point = true;
        for (const auto& point : cloud_->points)
        {
            if (std::sqrt(std::pow(point.x - x, 2) + std::pow(point.y - y, 2) + std::pow(point.z - z, 2)) < 0.01)
            {
                add_point = false;
                break;
            }
        }
        
        if (add_point)
        {
            cloud_->points.push_back(camera_point);
            
            // Add points around the camera position to simulate a map
            for (int i = 0; i < 10; i++) // Add 10 points per frame
            {
                double theta = 2.0 * M_PI * (double)rand() / RAND_MAX;
                double phi = M_PI * (double)rand() / RAND_MAX;
                double radius = radius_ * (double)rand() / RAND_MAX;
                
                // Generate points in current coordinate system directly
                double px = x + radius * sin(phi) * cos(theta);
                double py = y + radius * sin(phi) * sin(theta);
                double pz = z + radius * cos(phi);
                
                pcl::PointXYZRGB point;
                point.x = px;
                point.y = py;
                point.z = pz;
                
                // Set color based on position
                point.r = 128 + 127 * sin(theta);
                point.g = 128 + 127 * cos(phi);
                point.b = 128 + 127 * cos(theta);
                
                cloud_->points.push_back(point);
            }
            
            // If we exceed the maximum points, remove oldest points
            if (cloud_->points.size() > max_points_) {
                size_t points_to_remove = cloud_->points.size() - max_points_;
                cloud_->points.erase(cloud_->points.begin(), 
                                    cloud_->points.begin() + points_to_remove);
            }
            
            // Update cloud dimensions
            cloud_->width = cloud_->points.size();
        }
    }
    
    void filterAndPublish()
    {
        if (cloud_->points.empty()) {
            return;
        }
        
        // Apply voxel grid filter for downsampling
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
        voxel_filter.setInputCloud(cloud_);
        voxel_filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
        voxel_filter.filter(*filtered_cloud);
        
        // Update header
        filtered_cloud->header.stamp = pcl_conversions::toPCL(ros::Time::now());
        filtered_cloud->header.frame_id = frame_id_;
        
        // Publish
        cloud_pub_.publish(filtered_cloud);
        
        ROS_INFO_THROTTLE(5.0, "Published pointcloud with %lu points", filtered_cloud->points.size());
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "slam_pointcloud_publisher");
    SlamPointcloudPublisher node;
    ros::spin();
    return 0;
}