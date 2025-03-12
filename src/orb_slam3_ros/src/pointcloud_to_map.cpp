#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <tf/transform_listener.h>

// Octomap headers
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

// Eigen for transformations
#include <eigen3/Eigen/Geometry>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

class PointCloudToMap
{
public:
    PointCloudToMap() : nh_("~")
    {
        // Get parameters
        nh_.param<std::string>("frame_id", frame_id_, "world");
        nh_.param<double>("occupancy_grid_resolution", occupancy_grid_resolution_, 0.05); // 5cm per cell
        nh_.param<double>("octomap_resolution", octomap_resolution_, 0.1); // 10cm per voxel
        nh_.param<double>("z_min", z_min_, -0.1); // Ground plane threshold
        nh_.param<double>("z_max", z_max_, 2.0);  // Maximum height to consider
        
        // Subscribers
        pointcloud_sub_ = nh_.subscribe("/orb_slam3/map_points", 1, &PointCloudToMap::pointcloudCallback, this);
        
        // Publishers
        occupancy_grid_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);
        octomap_pub_ = nh_.advertise<octomap_msgs::Octomap>("/octomap", 1, true);
        filtered_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/filtered_cloud", 1);
        
        // Initialize octomap
        octomap_ = std::make_shared<octomap::OcTree>(octomap_resolution_);
        
        // Initialize occupancy grid
        occupancy_grid_.header.frame_id = frame_id_;
        occupancy_grid_.info.resolution = occupancy_grid_resolution_;
        
        ROS_INFO("PointCloud to Map converter initialized with:");
        ROS_INFO("  - Occupancy grid resolution: %.3f m/cell", occupancy_grid_resolution_);
        ROS_INFO("  - Octomap resolution: %.3f m/voxel", octomap_resolution_);
    }
    
private:
    ros::NodeHandle nh_;
    ros::Subscriber pointcloud_sub_;
    ros::Publisher occupancy_grid_pub_;
    ros::Publisher octomap_pub_;
    ros::Publisher filtered_cloud_pub_;
    tf::TransformListener tf_listener_;
    
    std::string frame_id_;
    double occupancy_grid_resolution_;
    double octomap_resolution_;
    double z_min_, z_max_;
    
    nav_msgs::OccupancyGrid occupancy_grid_;
    std::shared_ptr<octomap::OcTree> octomap_;
    
    void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
    {
        ROS_INFO("Received pointcloud with %d points", cloud_msg->width * cloud_msg->height);
        
        // Convert to PCL pointcloud
        PointCloud::Ptr cloud(new PointCloud);
        pcl::fromROSMsg(*cloud_msg, *cloud);
        
        // Skip if cloud is empty
        if (cloud->empty())
        {
            ROS_WARN("Received empty pointcloud");
            return;
        }
        
        // Filter the cloud
        PointCloud::Ptr filtered_cloud = filterPointCloud(cloud);
        
        // Publish filtered cloud for debugging
        sensor_msgs::PointCloud2 filtered_cloud_msg;
        pcl::toROSMsg(*filtered_cloud, filtered_cloud_msg);
        filtered_cloud_msg.header = cloud_msg->header;
        filtered_cloud_pub_.publish(filtered_cloud_msg);
        
        // Update octomap
        updateOctomap(filtered_cloud);
        
        // Update and publish 2D occupancy grid
        updateOccupancyGrid(filtered_cloud);
        
        // Publish octomap
        publishOctomap();
        
        ROS_INFO("Map update complete");
    }
    
    PointCloud::Ptr filterPointCloud(const PointCloud::Ptr& cloud)
    {
        // Apply voxel grid filter for downsampling
        PointCloud::Ptr voxel_filtered(new PointCloud);
        pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
        voxel_filter.setInputCloud(cloud);
        voxel_filter.setLeafSize(occupancy_grid_resolution_ / 2, occupancy_grid_resolution_ / 2, occupancy_grid_resolution_ / 2);
        voxel_filter.filter(*voxel_filtered);
        
        // Apply statistical outlier removal
        PointCloud::Ptr outlier_filtered(new PointCloud);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
        sor.setInputCloud(voxel_filtered);
        sor.setMeanK(50);  // Consider 50 neighbors
        sor.setStddevMulThresh(1.0);  // Standard deviation threshold
        sor.filter(*outlier_filtered);
        
        // Height filter (remove points too high or too low)
        PointCloud::Ptr height_filtered(new PointCloud);
        pcl::PassThrough<pcl::PointXYZRGB> pass;
        pass.setInputCloud(outlier_filtered);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(z_min_, z_max_);
        pass.filter(*height_filtered);
        
        ROS_INFO("Filtered pointcloud from %lu to %lu points", 
                cloud->size(), height_filtered->size());
        
        return height_filtered;
    }
    
    void updateOctomap(const PointCloud::Ptr& cloud)
    {
        // Clear current octomap
        octomap_->clear();
        
        // Insert points into octomap
        for (const auto& point : cloud->points)
        {
            // Convert to octomap point (note the coordinate system)
            octomap::point3d p(point.x, point.y, point.z);
            
            // Insert point as occupied
            octomap_->updateNode(p, true);
        }
        
        // Update inner occupancy and free space
        octomap_->updateInnerOccupancy();
        
        ROS_INFO("Octomap updated with %lu points", cloud->size());
    }
    
    void updateOccupancyGrid(const PointCloud::Ptr& cloud)
    {
        // Find the bounds of the pointcloud
        double min_x = std::numeric_limits<double>::max();
        double min_y = std::numeric_limits<double>::max();
        double max_x = -std::numeric_limits<double>::max();
        double max_y = -std::numeric_limits<double>::max();
        
        for (const auto& point : cloud->points)
        {
            min_x = std::min(min_x, (double)point.x);
            min_y = std::min(min_y, (double)point.y);
            max_x = std::max(max_x, (double)point.x);
            max_y = std::max(max_y, (double)point.y);
        }
        
        // Add padding
        const double padding = 1.0; // 1 meter padding
        min_x -= padding;
        min_y -= padding;
        max_x += padding;
        max_y += padding;
        
        // Setup occupancy grid metadata
        occupancy_grid_.header.stamp = ros::Time::now();
        occupancy_grid_.info.width = std::ceil((max_x - min_x) / occupancy_grid_resolution_);
        occupancy_grid_.info.height = std::ceil((max_y - min_y) / occupancy_grid_resolution_);
        occupancy_grid_.info.origin.position.x = min_x;
        occupancy_grid_.info.origin.position.y = min_y;
        occupancy_grid_.info.origin.position.z = 0.0;
        occupancy_grid_.info.origin.orientation.w = 1.0;
        
        // Resize and initialize grid with unknown (-1)
        occupancy_grid_.data.resize(occupancy_grid_.info.width * occupancy_grid_.info.height, -1);
        
        // Fill grid with pointcloud data
        for (const auto& point : cloud->points)
        {
            // Skip points outside the Z range (floor level ± threshold)
            if (point.z < z_min_ || point.z > z_max_)
                continue;
                
            // Convert point to grid cell
            int grid_x = std::floor((point.x - min_x) / occupancy_grid_resolution_);
            int grid_y = std::floor((point.y - min_y) / occupancy_grid_resolution_);
            
            // Check bounds
            if (grid_x >= 0 && grid_x < occupancy_grid_.info.width &&
                grid_y >= 0 && grid_y < occupancy_grid_.info.height)
            {
                // Mark as occupied (100)
                int index = grid_y * occupancy_grid_.info.width + grid_x;
                occupancy_grid_.data[index] = 100;
            }
        }
        
        // Simple dilation to connect nearby occupied cells
        nav_msgs::OccupancyGrid dilated = occupancy_grid_;
        for (size_t y = 1; y < occupancy_grid_.info.height - 1; ++y)
        {
            for (size_t x = 1; x < occupancy_grid_.info.width - 1; ++x)
            {
                int index = y * occupancy_grid_.info.width + x;
                
                // If cell is unknown, check neighbors
                if (occupancy_grid_.data[index] == -1)
                {
                    // Check 4-connected neighbors
                    bool has_occupied_neighbor = false;
                    for (int dy = -1; dy <= 1; ++dy)
                    {
                        for (int dx = -1; dx <= 1; ++dx)
                        {
                            if (dx == 0 && dy == 0) continue; // Skip self
                            
                            int nx = x + dx;
                            int ny = y + dy;
                            int nindex = ny * occupancy_grid_.info.width + nx;
                            
                            if (nx >= 0 && nx < occupancy_grid_.info.width &&
                                ny >= 0 && ny < occupancy_grid_.info.height &&
                                occupancy_grid_.data[nindex] == 100)
                            {
                                has_occupied_neighbor = true;
                                break;
                            }
                        }
                        if (has_occupied_neighbor) break;
                    }
                    
                    // Mark as occupied if it has an occupied neighbor
                    if (has_occupied_neighbor)
                    {
                        dilated.data[index] = 100;
                    }
                }
            }
        }
        
        // Use the dilated grid
        occupancy_grid_ = dilated;
        
        // Set unknown cells to free space (0)
        for (auto& cell : occupancy_grid_.data)
        {
            if (cell == -1)
                cell = 0;
        }
        
        // Publish occupancy grid
        occupancy_grid_pub_.publish(occupancy_grid_);
        
        ROS_INFO("Occupancy grid updated with size %dx%d", 
                 occupancy_grid_.info.width, occupancy_grid_.info.height);
    }
    
    void publishOctomap()
    {
        // Convert to ROS message
        octomap_msgs::Octomap octomap_msg;
        octomap_msg.header.frame_id = frame_id_;
        octomap_msg.header.stamp = ros::Time::now();
        
        // Use full probability octomap
        octomap_msgs::fullMapToMsg(*octomap_, octomap_msg);
        
        // Publish
        octomap_pub_.publish(octomap_msg);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud_to_map");
    PointCloudToMap converter;
    ros::spin();
    return 0;
}