#include "stereo-slam-node.hpp"

#include<opencv2/core/core.hpp>

#include<sensor_msgs/msg/image.h>
#include<std_msgs/msg/header.hpp>
#include<cv_bridge/cv_bridge.h>

#include <tf2_ros/transform_broadcaster.h>

#include <vector>

using std::placeholders::_1;
using std::placeholders::_2;

StereoSlamNode::StereoSlamNode(ORB_SLAM3::System* pSLAM, const string &strSettingsFile, const string &strDoRectify)
:   Node("ORB_SLAM3_ROS2"),
    m_SLAM(pSLAM),
    tf_broadcaster(std::make_shared<tf2_ros::TransformBroadcaster>(this))  // Initialize tf2 broadcaster
{
    stringstream ss(strDoRectify);
    ss >> boolalpha >> doRectify;

    if (doRectify){

        cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
        if(!fsSettings.isOpened()){
            cerr << "ERROR: Wrong path to settings" << endl;
            assert(0);
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0){
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            assert(0);
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);
    }

    //left_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(shared_ptr<rclcpp::Node>(this), "camera/left");
    //right_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(shared_ptr<rclcpp::Node>(this), "camera/right");
    left_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(this, "/camera/camera/infra1/image_rect_raw");
    right_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(this, "/camera/camera/infra2/image_rect_raw");

    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(approximate_sync_policy(10), *left_sub, *right_sub);
    syncApproximate->registerCallback(&StereoSlamNode::GrabStereo, this);

// --------------------------------------------------------------------------------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------------------------------------------------------------------------------
    img_pub = this->create_publisher<sensor_msgs::msg::Image>("/orbslam3/image_raw", 10);
    odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("/orbslam3/odometry", 10);
    pointcloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/orbslam3/points", 10);
    //tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this->shared_from_this());
// --------------------------------------------------------------------------------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------------------------------------------------------------------------------
}

StereoSlamNode::~StereoSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void StereoSlamNode::GrabStereo(const ImageMsg::SharedPtr msgLeft, const ImageMsg::SharedPtr msgRight)
{
    // Copy the ros rgb image message to cv::Mat.
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Copy the ros depth image message to cv::Mat.
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    if (doRectify){
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
        //m_SLAM->TrackStereo(imLeft, imRight, Utility::StampToSec(msgLeft->header.stamp));
// --------------------------------------------------------------------------------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------------------------------------------------------------------------------
        ORB_SLAM3::ROS2_OUTPUT DATA;
        DATA = m_SLAM->TrackStereo_ROS(imLeft, imRight, Utility::StampToSec(msgLeft->header.stamp));
        std_msgs::msg::Header header;
        header.frame_id = "camera_link";
        header.stamp = this->now();  // Get current ROS2 time
        try {
            // Convert and publish the image
            auto img_msg = cv_bridge::CvImage(header, "rgb8", DATA.image).toImageMsg();
            img_pub->publish(*img_msg);
        }
        catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge c2r exception: %s", e.what());
            return;
        }
 
        PublishOdometry(DATA.pos);
        PublishTransform(DATA.pos);
        PublishPointCloud(DATA.pointcloud);
// --------------------------------------------------------------------------------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------------------------------------------------------------------------------
    }
    else
    {
        //m_SLAM->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image, Utility::StampToSec(msgLeft->header.stamp));
// --------------------------------------------------------------------------------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------------------------------------------------------------------------------
        ORB_SLAM3::ROS2_OUTPUT DATA;
        DATA = m_SLAM->TrackStereo_ROS(cv_ptrLeft->image, cv_ptrRight->image, Utility::StampToSec(msgLeft->header.stamp));
        std_msgs::msg::Header header;
        header.frame_id = "camera_link";
        header.stamp = this->now();  // Get current ROS2 time
        try {
            // Convert and publish the image
            auto img_msg = cv_bridge::CvImage(header, "rgb8", DATA.image).toImageMsg();
            img_pub->publish(*img_msg);
        }
        catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge c2r exception: %s", e.what());
            return;
        }

        PublishOdometry(DATA.pos);
        PublishTransform(DATA.pos);
        PublishPointCloud(DATA.pointcloud);
// --------------------------------------------------------------------------------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------------------------------------------------------------------------------
    }

}

void StereoSlamNode::PublishOdometry(const Sophus::SE3f& pos)
{
    nav_msgs::msg::Odometry odom_msg;

    // Header
    odom_msg.header.stamp = this->get_clock()->now();
    odom_msg.header.frame_id = "base_footprint"; // Or your desired frame ID map
    //odom_msg.child_frame_id = "base_footprint"; // Or your desired child frame ID 

    // Position
    const Eigen::Vector3f& translation = pos.translation();
    odom_msg.pose.pose.position.x = translation.x();
    odom_msg.pose.pose.position.y = translation.y();
    odom_msg.pose.pose.position.z = translation.z();

    // Orientation
    Eigen::Quaternionf quat(pos.rotationMatrix());
    odom_msg.pose.pose.orientation.x = quat.x();
    odom_msg.pose.pose.orientation.y = quat.y();
    odom_msg.pose.pose.orientation.z = quat.z();
    odom_msg.pose.pose.orientation.w = quat.w();

    // Publish
    odom_pub->publish(odom_msg);
}

void StereoSlamNode::PublishTransform(const Sophus::SE3f& pos)
{
    geometry_msgs::msg::TransformStamped transformStamped;

    transformStamped.header.stamp = this->now();
    transformStamped.header.frame_id = "odom_orb";  // Parent frame ID
    transformStamped.child_frame_id = "base_footprint";  // Child frame ID

    transformStamped.transform.translation.x = pos.translation().x();
    transformStamped.transform.translation.y = pos.translation().y();
    transformStamped.transform.translation.z = pos.translation().z();

    Eigen::Quaternionf quat(pos.rotationMatrix());
    transformStamped.transform.rotation.x = quat.x();
    transformStamped.transform.rotation.y = quat.y();
    transformStamped.transform.rotation.z = quat.z();
    transformStamped.transform.rotation.w = quat.w();

    tf_broadcaster->sendTransform(transformStamped);
}

void StereoSlamNode::PublishPointCloud(const std::vector<Eigen::Vector3f>& pointcloud)
{
    sensor_msgs::msg::PointCloud2 pointcloud_msg;

    // Set header
    pointcloud_msg.header.stamp = this->now();
    pointcloud_msg.header.frame_id = "base_footprint";

    // Set fields
    pointcloud_msg.height = 1;
    pointcloud_msg.width = pointcloud.size();
    pointcloud_msg.is_dense = false;
    pointcloud_msg.is_bigendian = false;

    // Set fields metadata
    sensor_msgs::msg::PointField x_field;
    x_field.name = "x";
    x_field.offset = 0;
    x_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    x_field.count = 1;

    sensor_msgs::msg::PointField y_field;
    y_field.name = "y";
    y_field.offset = 4;
    y_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    y_field.count = 1;

    sensor_msgs::msg::PointField z_field;
    z_field.name = "z";
    z_field.offset = 8;
    z_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    z_field.count = 1;

    pointcloud_msg.fields.push_back(x_field);
    pointcloud_msg.fields.push_back(y_field);
    pointcloud_msg.fields.push_back(z_field);

    // Resize data array
    pointcloud_msg.point_step = 12;
    pointcloud_msg.row_step = pointcloud_msg.point_step * pointcloud_msg.width;
    pointcloud_msg.data.resize(pointcloud_msg.row_step);

    // Fill data array
    uint8_t* ptr = pointcloud_msg.data.data();
    for (const auto& point : pointcloud)
    {
        *reinterpret_cast<float*>(ptr) = point.z();
        ptr += 4;
        *reinterpret_cast<float*>(ptr) = -point.x();
        ptr += 4;
        *reinterpret_cast<float*>(ptr) = -point.y();
        ptr += 4;
    }

    // Publish the point cloud message
    pointcloud_pub->publish(pointcloud_msg);
}
