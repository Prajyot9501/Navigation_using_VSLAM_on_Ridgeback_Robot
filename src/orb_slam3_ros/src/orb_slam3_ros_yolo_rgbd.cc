#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_broadcaster.h>

// PCL includes for point cloud publishing
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Include Sophus for SE3 handling
#include <sophus/se3.hpp>

// Include System.h with full path
#include "System.h"

using namespace std;
using namespace ORB_SLAM3;

class ImageGrabber
{
public:
    ImageGrabber(System* pSLAM) : mpSLAM(pSLAM), mbMaskReceived(false) {
        // Initialize node handle and point cloud publisher
        ros::NodeHandle nh;
        pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/orb_slam3/map_points", 1);
        ROS_INFO("Point cloud publisher initialized on topic: /orb_slam3/map_points");
    }

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImageConstPtr& msgD);
    void MaskCallback(const sensor_msgs::ImageConstPtr& msgMask);
    void PublishMapPoints();
    void PublishTestPointCloud();

    System* mpSLAM;
    tf::TransformBroadcaster br;
    
    cv::Mat mCurrentMask;
    bool mbMaskReceived;
    std::mutex mMaskMutex;
    ros::Publisher pointcloud_pub;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD_YOLO");
    ros::start();
    ros::NodeHandle nh;
    ROS_INFO("Starting RGBD_YOLO node");

    // Display library paths for debugging
    char* ld_library_path = getenv("LD_LIBRARY_PATH");
    if (ld_library_path) {
        ROS_INFO("LD_LIBRARY_PATH: %s", ld_library_path);
    } else {
        ROS_WARN("LD_LIBRARY_PATH is not set!");
    }

    if(argc != 4)
    {
        cerr << endl << "Usage: rosrun orb_slam3_ros orb_slam3_ros_yolo_rgbd path_to_vocabulary path_to_settings use_viewer" << endl;        
        ros::shutdown();
        return 1;
    }

    // Check that vocabulary file exists
    ifstream f(argv[1]);
    if (!f.good()) {
        ROS_ERROR("Vocabulary file not found at: %s", argv[1]);
        ros::shutdown();
        return 1;
    }

    // Check that settings file exists
    ifstream f2(argv[2]);
    if (!f2.good()) {
        ROS_ERROR("Settings file not found at: %s", argv[2]);
        ros::shutdown();
        return 1;
    }

    // Create SLAM system with error handling
    ROS_INFO("Initializing ORB-SLAM3 system with:");
    ROS_INFO("  Vocabulary: %s", argv[1]);
    ROS_INFO("  Settings: %s", argv[2]);
    ROS_INFO("  Viewer: %s", std::stoi(argv[3]) ? "Enabled" : "Disabled");
    
    System* pSLAM = nullptr;
    
    try {
        pSLAM = new System(argv[1], argv[2], System::RGBD, bool(std::stoi(argv[3])));
        ROS_INFO("ORB-SLAM3 system initialized successfully");
    }
    catch(const std::exception& e) {
        ROS_ERROR("Failed to initialize ORB-SLAM3: %s", e.what());
        ros::shutdown();
        return 1;
    }
    
    ImageGrabber igb(pSLAM);

    // Dynamic mask subscriber
    ROS_INFO("Subscribing to dynamic mask topic...");
    ros::Subscriber mask_sub = nh.subscribe("/dynamic_mask", 1, &ImageGrabber::MaskCallback, &igb);

    ROS_INFO("Setting up RGB and depth subscribers...");
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth_registered/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub, depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD, &igb, _1, _2));
    ROS_INFO("RGB and depth subscribers set up");

    // Publish an initial test point cloud to verify topic setup
    igb.PublishTestPointCloud();

    ROS_INFO("Starting ROS spin");
    ros::spin();

    // Stop all threads
    ROS_INFO("Shutting down ORB-SLAM3 system");
    if (pSLAM) {
        pSLAM->Shutdown();
        
        // Save camera trajectory
        ROS_INFO("Saving camera trajectory");
        pSLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
        
        delete pSLAM;
    }

    ros::shutdown();
    return 0;
}

void ImageGrabber::PublishTestPointCloud()
{
    // Create a simple test point cloud (similar to Python version)
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud->header.frame_id = "world";
    cloud->header.stamp = pcl_conversions::toPCL(ros::Time::now());
    
    // Generate a grid of points (-5 to 5 in x and y, z=0)
    for (float x = -2.0f; x <= 2.0f; x += 0.1f) {
        for (float y = -2.0f; y <= 2.0f; y += 0.1f) {
            pcl::PointXYZRGB p;
            p.x = x;
            p.y = y;
            p.z = 0.0f;
            
            // Rainbow coloring
            p.r = static_cast<uint8_t>(255 * (x + 2.0f) / 4.0f);
            p.g = static_cast<uint8_t>(255 * (y + 2.0f) / 4.0f);
            p.b = 255;
            
            cloud->points.push_back(p);
        }
    }
    
    // Set cloud width/height and publish
    cloud->width = cloud->points.size();
    cloud->height = 1;
    
    // Convert to ROS message
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header.frame_id = "world";
    output.header.stamp = ros::Time::now();
    
    // Publish
    ROS_INFO("Publishing test point cloud with %ld points", cloud->points.size());
    pointcloud_pub.publish(output);
}

void ImageGrabber::MaskCallback(const sensor_msgs::ImageConstPtr& msgMask)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msgMask);
        ROS_INFO_THROTTLE(5.0, "Received dynamic mask frame");
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception in MaskCallback: %s", e.what());
        return;
    }
    
    std::unique_lock<std::mutex> lock(mMaskMutex);
    mCurrentMask = cv_ptr->image.clone();
    mbMaskReceived = true;
}

void ImageGrabber::PublishMapPoints()
{
    if (!mpSLAM) {
        ROS_ERROR("SLAM system is not initialized");
        PublishTestPointCloud();  // Fallback to test cloud
        return;
    }

    // Create PCL point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud->header.frame_id = "world";
    cloud->header.stamp = pcl_conversions::toPCL(ros::Time::now());
    
    try {
        // Try GetAllMapPoints() first
        vector<MapPoint*> vMPs;
        try {
            vMPs = mpSLAM->GetTrackedMapPoints();
            ROS_INFO_THROTTLE(1.0, "Retrieved %ld map points", vMPs.size());
        }
        catch (const std::exception& e) {
            ROS_WARN("Exception in GetAllMapPoints: %s", e.what());
            ROS_WARN("Trying GetTrackedMapPoints instead");
            
            try {
                vMPs = mpSLAM->GetTrackedMapPoints();
                ROS_INFO_THROTTLE(1.0, "Retrieved %ld tracked map points", vMPs.size());
            }
            catch (const std::exception& e) {
                ROS_ERROR("Exception in GetTrackedMapPoints: %s", e.what());
                PublishTestPointCloud();  // Fallback to test cloud
                return;
            }
        }
        
        if (vMPs.empty()) {
            ROS_WARN_THROTTLE(5.0, "No map points available yet, publishing test cloud instead");
            PublishTestPointCloud();
            return;
        }
        
        int validPoints = 0;
        for (MapPoint* pMP : vMPs) {
            if (!pMP) continue;
            
            Eigen::Vector3f pos3d;
            try {
                pos3d = pMP->GetWorldPos();
            }
            catch (const std::exception& e) {
                continue;
            }
            
            // Skip invalid points
            if (pos3d.x() == 0.0f && pos3d.y() == 0.0f && pos3d.z() == 0.0f ||
                std::isnan(pos3d.x()) || std::isnan(pos3d.y()) || std::isnan(pos3d.z())) {
                continue;
            }
            
            // Create colored point
            pcl::PointXYZRGB p;
            p.x = pos3d.x();
            p.y = pos3d.y();
            p.z = pos3d.z();
            
            // Color based on height (blue-green-yellow-red gradient)
            float height = pos3d.z();
            if (height < -1.0f) {
                p.r = 0; p.g = 0; p.b = 255;  // Blue for low points
            } else if (height < 0.0f) {
                p.r = 0; p.g = 255 * (height + 1.0f); p.b = 255 * (1.0f - (height + 1.0f));
            } else if (height < 1.0f) {
                p.r = 255 * height; p.g = 255; p.b = 0;
            } else {
                p.r = 255; p.g = 255 * (2.0f - height) * 0.5f; p.b = 0;  // Red for high points
            }
            
            cloud->points.push_back(p);
            validPoints++;
        }
        
        ROS_INFO_THROTTLE(1.0, "Added %d valid points to point cloud", validPoints);
        
        if (validPoints == 0) {
            ROS_WARN_THROTTLE(5.0, "No valid points found, publishing test cloud instead");
            PublishTestPointCloud();
            return;
        }
        
        // Set cloud width/height
        cloud->width = cloud->points.size();
        cloud->height = 1;
        
        // Convert to ROS message
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*cloud, output);
        output.header.frame_id = "world";
        output.header.stamp = ros::Time::now();
        
        // Publish
        pointcloud_pub.publish(output);
    }
    catch (const std::exception& e) {
        ROS_ERROR("Exception in PublishMapPoints: %s", e.what());
        PublishTestPointCloud();  // Fallback to test cloud
    }
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImageConstPtr& msgD)
{
    if (!mpSLAM) {
        ROS_ERROR("SLAM system is not initialized");
        return;
    }

    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception in GrabRGBD for RGB: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception in GrabRGBD for depth: %s", e.what());
        return;
    }

    // Log the image dimensions once
    static bool first_frame = true;
    if (first_frame) {
        ROS_INFO("RGB image dimensions: %dx%d, type: %d", 
                 cv_ptrRGB->image.cols, cv_ptrRGB->image.rows, cv_ptrRGB->image.type());
        ROS_INFO("Depth image dimensions: %dx%d, type: %d", 
                 cv_ptrD->image.cols, cv_ptrD->image.rows, cv_ptrD->image.type());
        first_frame = false;
    }

    // Get the current dynamic mask
    cv::Mat dynamicMask;
    {
        std::unique_lock<std::mutex> lock(mMaskMutex);
        if (mbMaskReceived)
        {
            dynamicMask = mCurrentMask.clone();
            ROS_INFO_THROTTLE(5.0, "Using received dynamic mask");
        }
        else
        {
            // If no mask received yet, create an empty mask
            dynamicMask = cv::Mat::zeros(cv_ptrRGB->image.size(), CV_8UC1);
            ROS_INFO_THROTTLE(5.0, "No dynamic mask received yet, using empty mask");
        }
    }

    // Preprocess the dynamic mask (ensure it's binary)
    if (!dynamicMask.empty())
    {
        cv::threshold(dynamicMask, dynamicMask, 127, 255, cv::THRESH_BINARY);
    }

    // Create modified ORB-SLAM3 input
    cv::Mat rgbImageWithMask = cv_ptrRGB->image.clone();
    
    // Apply mask to image
    if (!dynamicMask.empty() && dynamicMask.size() == rgbImageWithMask.size())
    {
        // Dark red overlay on dynamic regions
        rgbImageWithMask.setTo(cv::Scalar(0, 0, 100), dynamicMask);
        ROS_INFO_THROTTLE(5.0, "Applied dynamic mask to RGB image");
    }
    
    // Pass the masked RGB image to ORB-SLAM3
    ROS_INFO_THROTTLE(1.0, "Calling ORB-SLAM3 TrackRGBD...");
    
    Sophus::SE3f Tcw_sophus;
    try {
        Tcw_sophus = mpSLAM->TrackRGBD(rgbImageWithMask, cv_ptrD->image, cv_ptrRGB->header.stamp.toSec());
    }
    catch(const std::exception& e) {
        ROS_ERROR("Exception in TrackRGBD: %s", e.what());
        return;
    }
    
    // Check if tracking was successful (valid pose)
    if(!Tcw_sophus.matrix().isZero(0))
    {
        ROS_INFO_THROTTLE(1.0, "ORB-SLAM3 tracking successful");
        
        // Convert Sophus::SE3f to tf transform
        try {
            // Get rotation matrix and translation vector from Sophus::SE3f
            Eigen::Matrix3f R = Tcw_sophus.rotationMatrix().inverse();
            Eigen::Vector3f t = -(R * Tcw_sophus.translation());
            
            // Convert to tf format
            tf::Matrix3x3 M(
                R(0,0), R(0,1), R(0,2),
                R(1,0), R(1,1), R(1,2),
                R(2,0), R(2,1), R(2,2)
            );
            
            tf::Vector3 V(t(0), t(1), t(2));
            
            tf::Transform transform = tf::Transform(M, V);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "ORB_SLAM3"));
            
            // Publish world to camera transform (inverse of camera to world)
            tf::Transform world_camera_tf = tf::Transform(M, V);
            br.sendTransform(tf::StampedTransform(world_camera_tf, ros::Time::now(), "world", "camera_link"));
        }
        catch(const std::exception& e) {
            ROS_ERROR("Exception in transform calculation: %s", e.what());
        }
        
        // Publish point cloud (only every 10 frames to reduce CPU load)
        static int frameCount = 0;
        if(++frameCount % 10 == 0) {
            try {
                PublishMapPoints();
                frameCount = 0;
            }
            catch(const std::exception& e) {
                ROS_ERROR("Exception in PublishMapPoints: %s", e.what());
            }
        }
    }
    else
    {
        ROS_WARN_THROTTLE(1.0, "ORB-SLAM3 tracking failed, pose matrix is zero");
    }
}