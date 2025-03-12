#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <eigen3/Eigen/Geometry>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

class SlamPointcloudPublisher
{
public:
    SlamPointcloudPublisher() : nh_("~")
    {
        // Parameters
        nh_.param<std::string>("frame_id", frame_id_, "world");
        nh_.param<int>("num_points", num_points_, 5000);
        nh_.param<double>("update_rate", update_rate_, 5.0);
        nh_.param<double>("pointcloud_radius", radius_, 1.0); // Radius around camera
        
        // Initialize point cloud
        cloud_.reset(new PointCloud);
        cloud_->height = 1;
        cloud_->width = 0;
        cloud_->is_dense = true;
        cloud_->header.frame_id = frame_id_;
        
        // Publisher
        cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/orb_slam3/map_points", 1);
        
        // No coordinate transform needed - we're now doing this in the SLAM node
        
        // Timer
        timer_ = nh_.createTimer(ros::Duration(1.0/update_rate_), &SlamPointcloudPublisher::timerCallback, this);
        
        ROS_INFO("Slam Pointcloud Publisher initialized - using TF coordinate frames directly");
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher cloud_pub_;
    ros::Timer timer_;
    tf::TransformListener tf_listener_;
    
    std::string frame_id_;
    int num_points_;
    double update_rate_;
    double radius_;
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;
    
    void timerCallback(const ros::TimerEvent& event)
    {
        tf::StampedTransform transform;
        
        try
        {
            // Try to get the latest transform from the SLAM system
            tf_listener_.lookupTransform(frame_id_, "ORB_SLAM3", ros::Time(0), transform);
            
            // Get the camera position directly from the transform
            double x = transform.getOrigin().x();
            double y = transform.getOrigin().y();
            double z = transform.getOrigin().z();
            
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

                try {
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
                } catch (const std::exception& e) {
                    ROS_ERROR("Exception in point generation: %s", e.what());
                }
                
                // Also add some points around the camera position to simulate a map
                
                if (cloud_->points.size() >= num_points_) {
                    // If we're at or exceeding the limit, clear some points
                    size_t remove_count = std::min(cloud_->points.size() - num_points_ + 20, cloud_->points.size());
                    if (remove_count > 0) {
                        cloud_->points.erase(cloud_->points.begin(), cloud_->points.begin() + remove_count);
                    }
                }
            }
            
            // Update cloud dimensions
            cloud_->width = cloud_->points.size();
            cloud_->header.stamp = pcl_conversions::toPCL(ros::Time::now());
            
            // Publish
            cloud_pub_.publish(cloud_);
            
        }
        catch (tf::TransformException &ex)
        {
            // This is fine - we just wait for the transform to become available
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "slam_pointcloud_publisher");
    SlamPointcloudPublisher node;
    ros::spin();
    return 0;
}