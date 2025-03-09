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

#include <System.h>

using namespace std;
using namespace ORB_SLAM3;

class ImageGrabber
{
public:
    ImageGrabber(System* pSLAM) : mpSLAM(pSLAM), mbMaskReceived(false) {
        // Initialize node handle and point cloud publisher
        ROS_INFO("Initializing point cloud publisher...");
        pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/orb_slam3/map_points", 1);
        ROS_INFO("Point cloud publisher initialized on topic: /orb_slam3/map_points");
    }

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImageConstPtr& msgD);
    void MaskCallback(const sensor_msgs::ImageConstPtr& msgMask);
    void PublishMapPoints();

    System* mpSLAM;
    tf::TransformBroadcaster br;
    
    cv::Mat mCurrentMask;
    bool mbMaskReceived;
    std::mutex mMaskMutex;

private:
    ros::NodeHandle nh;
    ros::Publisher pointcloud_pub;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD_YOLO");
    ros::start();
    ROS_INFO("Starting RGBD_YOLO node");

    if(argc != 4)
    {
        cerr << endl << "Usage: rosrun orb_slam3_ros orb_slam3_ros_yolo_rgbd path_to_vocabulary path_to_settings use_viewer" << endl;        
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ROS_INFO("Initializing ORB-SLAM3 system...");
    System SLAM(argv[1], argv[2], System::RGBD, bool(std::stoi(argv[3])));
    ROS_INFO("ORB-SLAM3 system initialized");

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;
    
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

    ROS_INFO("Starting ROS spin");
    ros::spin();

    // Stop all threads
    ROS_INFO("Shutting down ORB-SLAM3 system");
    SLAM.Shutdown();

    // Save camera trajectory
    ROS_INFO("Saving camera trajectory");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::MaskCallback(const sensor_msgs::ImageConstPtr& msgMask)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msgMask);
        ROS_INFO("Received dynamic mask frame");
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
    ROS_INFO("*** PublishMapPoints called ***");
    
    // Create PCL point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud->header.frame_id = "world";
    cloud->header.stamp = pcl_conversions::toPCL(ros::Time::now());
    
    try {
        // Get all tracked map points
        ROS_INFO("Getting tracked map points...");
        const vector<MapPoint*> vMPs = mpSLAM->GetTrackedMapPoints();
        ROS_INFO("Retrieved %ld tracked map points", vMPs.size());
        
        int validPoints = 0;
        for(size_t i = 0; i < vMPs.size(); i++)
        {
            MapPoint* pMP = vMPs[i];
            if(!pMP) {
                continue;
            }
            
            // Now we correctly use Eigen::Vector3f
            Eigen::Vector3f pos3d;
            try {
                pos3d = pMP->GetWorldPos();
            } catch(const std::exception& e) {
                ROS_ERROR("Exception getting world position: %s", e.what());
                continue;
            }
            
            // Skip invalid points
            if(pos3d.x() == 0.0f && pos3d.y() == 0.0f && pos3d.z() == 0.0f) {
                continue;
            }
            
            // Create colored point
            pcl::PointXYZRGB p;
            p.x = pos3d.x();
            p.y = pos3d.y();
            p.z = pos3d.z();
            
            // Default color (white)
            p.r = 255;
            p.g = 255;
            p.b = 255;
            
            // Add to cloud
            cloud->points.push_back(p);
            validPoints++;
        }
        
        ROS_INFO("Added %d valid points to point cloud", validPoints);
    } catch(const std::exception& e) {
        ROS_ERROR("Exception in PublishMapPoints: %s", e.what());
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
    ROS_INFO("Publishing point cloud with %ld points", cloud->points.size());
    pointcloud_pub.publish(output);
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImageConstPtr& msgD)
{
    ROS_INFO("Received RGB-D frame pair");
    
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

    // Get the current dynamic mask
    cv::Mat dynamicMask;
    {
        std::unique_lock<std::mutex> lock(mMaskMutex);
        if (mbMaskReceived)
        {
            dynamicMask = mCurrentMask.clone();
            ROS_INFO("Using received dynamic mask");
        }
        else
        {
            // If no mask received yet, create an empty mask
            dynamicMask = cv::Mat::zeros(cv_ptrRGB->image.size(), CV_8UC1);
            ROS_INFO("No dynamic mask received yet, using empty mask");
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
        ROS_INFO("Applied dynamic mask to RGB image");
    }
    
    // Pass the masked RGB image to ORB-SLAM3
    ROS_INFO("Calling ORB-SLAM3 TrackRGBD...");
    Sophus::SE3f Tcw_sophus = mpSLAM->TrackRGBD(rgbImageWithMask, cv_ptrD->image, cv_ptrRGB->header.stamp.toSec());
    
    // Check if tracking was successful (valid pose)
    if(!Tcw_sophus.matrix().isZero(0))
    {
        ROS_INFO("ORB-SLAM3 tracking successful, publishing transform and point cloud");
        
        // Convert Sophus::SE3f to tf transform
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
        
        // Publish point cloud
        PublishMapPoints();
    }
    else
    {
        ROS_WARN("ORB-SLAM3 tracking failed, pose matrix is zero");
    }
}