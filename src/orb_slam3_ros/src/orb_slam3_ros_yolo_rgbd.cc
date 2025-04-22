#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_broadcaster.h>

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Include Sophus for SE3 handling
#include <sophus/se3.hpp>

#include <System.h>
#include <signal.h>
#include <stdexcept>

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM) : mpSLAM(pSLAM), mbMaskReceived(false) {}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImageConstPtr& msgD);
    void MaskCallback(const sensor_msgs::ImageConstPtr& msgMask);

    ORB_SLAM3::System* mpSLAM;
    tf::TransformBroadcaster br;
    
    cv::Mat mCurrentMask;
    bool mbMaskReceived;
    std::mutex mMaskMutex;
};

// Flag to indicate if a crash or shutdown is in progress
volatile sig_atomic_t fatal_error_in_progress = 0;

// Error handling function
void handleError(const char* msg) {
    ROS_ERROR("ORB-SLAM3 ERROR: %s", msg);
    // Just log for now, but keep running
}

// Signal handler for critical errors
void crashHandler(int sig) {
    // Make sure we only handle the signal once
    if (fatal_error_in_progress) {
        raise(sig);
    }
    fatal_error_in_progress = 1;
    
    // Log the signal
    ROS_ERROR("Received signal %d, attempting clean shutdown...", sig);
    
    // Shutdown ROS
    ros::shutdown();
}

int main(int argc, char **argv)
{
    signal(SIGSEGV, crashHandler);  // Segmentation fault
    signal(SIGABRT, crashHandler);  // Abort
    signal(SIGFPE, crashHandler);   // Floating point exception
    ros::init(argc, argv, "RGBD_YOLO");
    ros::start();

    if(argc != 4)
    {
        cerr << endl << "Usage: rosrun orb_slam3_ros orb_slam3_ros_yolo_rgbd path_to_vocabulary path_to_settings use_viewer" << endl;        
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::RGBD, bool(std::stoi(argv[3])));

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;
    
    // Dynamic mask subscriber
    ros::Subscriber mask_sub = nh.subscribe("/dynamic_mask", 1, &ImageGrabber::MaskCallback, &igb);

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth_registered/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub, depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD, &igb, _1, _2));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
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
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    std::unique_lock<std::mutex> lock(mMaskMutex);
    mCurrentMask = cv_ptr->image.clone();
    mbMaskReceived = true;
}

// Modify the void ImageGrabber::GrabRGBD function by adding a try-catch block:
void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImageConstPtr& msgD)
{
    // Wrap the entire function in a try-catch block
    try {
        // Copy the ros image message to cv::Mat.
        cv_bridge::CvImageConstPtr cv_ptrRGB;
        try {
            cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge RGB exception: %s", e.what());
            return;
        }

        cv_bridge::CvImageConstPtr cv_ptrD;
        try {
            cv_ptrD = cv_bridge::toCvShare(msgD);
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge Depth exception: %s", e.what());
            return;
        }

        // Get the current dynamic mask
        cv::Mat dynamicMask;
        {
            std::unique_lock<std::mutex> lock(mMaskMutex);
            if (mbMaskReceived) {
                dynamicMask = mCurrentMask.clone();
            } else {
                // If no mask received yet, create an empty mask
                dynamicMask = cv::Mat::zeros(cv_ptrRGB->image.size(), CV_8UC1);
            }
        }

        // Preprocess the dynamic mask (ensure it's binary)
        if (!dynamicMask.empty()) {
            cv::threshold(dynamicMask, dynamicMask, 127, 255, cv::THRESH_BINARY);
        }

        // Create modified ORB-SLAM3 input
        cv::Mat rgbImageWithMask = cv_ptrRGB->image.clone();
        
        // Apply mask to image (optional - helps with visualization)
        if (!dynamicMask.empty() && dynamicMask.size() == rgbImageWithMask.size()) {
            // Dark red overlay on dynamic regions
            rgbImageWithMask.setTo(cv::Scalar(0, 0, 100), dynamicMask);
        }
        
        // Pass the masked RGB image to ORB-SLAM3 with error handling
        Sophus::SE3f Tcw_sophus;
        try {
            Tcw_sophus = mpSLAM->TrackRGBD(rgbImageWithMask, cv_ptrD->image, cv_ptrRGB->header.stamp.toSec());
        }
        catch (const std::exception& e) {
            ROS_ERROR("ORB-SLAM3 tracking error: %s", e.what());
            return;  // Skip this frame and continue with the next one
        }
        catch (...) {
            ROS_ERROR("Unknown ORB-SLAM3 tracking error occurred");
            return;  // Skip this frame and continue with the next one
        }
        
        // Check if tracking was successful (valid pose)
        if(!Tcw_sophus.matrix().isZero(0)) {
            try {
                // Get rotation matrix and translation vector from camera-to-world transform
                Eigen::Matrix3f Rwc = Tcw_sophus.rotationMatrix().inverse();
                Eigen::Vector3f twc = -(Rwc * Tcw_sophus.translation());
                
                // Define the coordinate transformation from ORB-SLAM3 to ROS standard
                // ORB-SLAM3: Z forward, X right, Y down
                // ROS: X forward, Y left, Z up
                Eigen::Matrix3f R_coord_transform;
                R_coord_transform << 0, 0, 1,
                                    -1, 0, 0,
                                     0, -1, 0;
                
                // Apply the coordinate transformation
                Eigen::Matrix3f R_final = R_coord_transform * Rwc;
                Eigen::Vector3f t_final = R_coord_transform * twc;
                
                // Check for NaN values in rotation and translation
                if (R_final.hasNaN() || t_final.hasNaN()) {
                    ROS_WARN("NaN values detected in transformation, skipping TF broadcast");
                    return;
                }
                
                // Convert to tf format
                tf::Matrix3x3 M(
                    R_final(0,0), R_final(0,1), R_final(0,2),
                    R_final(1,0), R_final(1,1), R_final(1,2),
                    R_final(2,0), R_final(2,1), R_final(2,2)
                );
                
                tf::Vector3 V(t_final(0), t_final(1), t_final(2));
                
                tf::Transform transform = tf::Transform(M, V);
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "ORB_SLAM3"));
            }
            catch (const std::exception& e) {
                ROS_ERROR("TF broadcast error: %s", e.what());
            }
            catch (...) {
                ROS_ERROR("Unknown TF broadcast error occurred");
            }
        }
    }
    catch (const std::exception& e) {
        ROS_ERROR("Critical error in GrabRGBD: %s", e.what());
        // Continue processing next frame
    }
    catch (...) {
        ROS_ERROR("Unknown critical error in GrabRGBD");
        // Continue processing next frame
    }
}