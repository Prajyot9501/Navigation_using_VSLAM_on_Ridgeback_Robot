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
        
        // Create transformation matrix for coordinate conversion
        // This transforms from ORB-SLAM3 coordinate system to ROS standard
        // ORB-SLAM3: Z forward, X right, Y up
        // ROS standard: X forward, Y left, Z up
        transform_matrix_.setIdentity();
        Eigen::Matrix3d rot;
        // X = Z, Y = -X, Z = Y (from ORB-SLAM3 to ROS standard)
        rot << 0, 0, 1,
               -1, 0, 0,
               0, 1, 0;
        transform_matrix_.block<3,3>(0,0) = rot;
        
        // Timer
        timer_ = nh_.createTimer(ros::Duration(1.0/update_rate_), &SlamPointcloudPublisher::timerCallback, this);
        
        ROS_INFO("Slam Pointcloud Publisher initialized with coordinate frame correction");
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
    Eigen::Matrix4d transform_matrix_;
    
    // Transform a point from ORB-SLAM3 coordinates to ROS standard coordinates
    void transformPoint(double& x, double& y, double& z)
    {
        Eigen::Vector4d point_orb(x, y, z, 1.0);
        Eigen::Vector4d point_ros = transform_matrix_ * point_orb;
        x = point_ros(0);
        y = point_ros(1);
        z = point_ros(2);
    }
    
    void timerCallback(const ros::TimerEvent& event)
    {
        tf::StampedTransform transform;
        
        try
        {
            // Try to get the latest transform from the SLAM system
            tf_listener_.lookupTransform(frame_id_, "ORB_SLAM3", ros::Time(0), transform);
            
            // Get the camera position
            double x = transform.getOrigin().x();
            double y = transform.getOrigin().y();
            double z = transform.getOrigin().z();
            
            // Apply coordinate frame correction
            double orig_x = x, orig_y = y, orig_z = z;
            transformPoint(x, y, z);
            
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
                
                // Also add some points around the camera position to simulate a map
                for (int i = 0; i < 10; i++) // Add 10 points per frame
                {
                    double theta = 2.0 * M_PI * (double)rand() / RAND_MAX;
                    double phi = M_PI * (double)rand() / RAND_MAX;
                    double radius = radius_ * (double)rand() / RAND_MAX;
                    
                    // Generate in original coordinate system
                    double px = orig_x + radius * sin(phi) * cos(theta);
                    double py = orig_y + radius * sin(phi) * sin(theta);
                    double pz = orig_z + radius * cos(phi);
                    
                    // Transform to ROS coordinate system
                    transformPoint(px, py, pz);
                    
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
                
                // Limit cloud size
                if (cloud_->points.size() > num_points_)
                {
                    cloud_->points.erase(cloud_->points.begin(), cloud_->points.begin() + (cloud_->points.size() - num_points_));
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