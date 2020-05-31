/*
 * @Author: 南山二毛
 * @Date: 2019-12-29 11:18:52
 * @LastEditTime : 2020-05-28 21:18:16
 * @LastEditors  : 南山二毛
 * @Description: In User Settings Edit
 * @FilePath: /catkin_tagslam/src/semantic_ros/include/Node.h
 */ 

#ifndef Semantic_ROS_NODE_H_
#define Semantic_ROS_NODE_H_
#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <ros/ros.h>
#include <ros/time.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseStamped.h> 

#include "RunDetect.h" // 目标检测运行线程====
#include "MapDrawer.h"   // 显示地图类

class Node
{
  public:
    Node (ros::NodeHandle &node_handle, std::string config_file_path);
    ~Node ();

  protected:
    bool Update (ros::Time current_stamp); // 更新ros topic  
    void RGBDCalculate(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp, Eigen::Matrix4f T_room_camera_f); //接收信息并进行处理
 
    ros::Time current_frame_time_;
    std::string color_img_topic_;
    std::string depth_img_topic_;
    std::string package_path_;
    Eigen::Matrix4d base2camera_T_;
  private:
    void PublishRenderedImage (cv::Mat image);
    std::shared_ptr< Semantic_ros::RunDetect > mpRunDetect;         // 目标检测运行线程
    Semantic_ros::MapDrawer* mpMapDrawer;
    double fx_;
    double fy_;
    double cx_;
    double cy_; 
    // void PublishMapPoints (std::vector<ORB_SLAM2::MapPoint*> map_points);
    // void PublishPositionAsTransform (cv::Mat position);
    // void PublishPositionAsPoseStamped(cv::Mat position);

    // // void isArrivedCallback(const costmap_lrgbd_ros::Arrived::ConstPtr arrived_msg); 
    // // void ParamsChangedCallback(orb_slam2_ros::dynamic_reconfigureConfig &config, uint32_t level); 
    // tf::Transform TransformFromMat (cv::Mat position_mat);
    // sensor_msgs::PointCloud2 MapPointsToPointCloud (std::vector<ORB_SLAM2::MapPoint*> map_points);
 
    image_transport::Publisher rendered_image_publisher_;
    // ros::Publisher map_points_publisher_;
    // ros::Publisher pose_publisher_;
    // ros::Subscriber is_arrived_sub_; 
    std::shared_ptr<image_transport::ImageTransport> it_;
    // std::string name_of_node_;
    // ros::NodeHandle node_handle_;
    // std::string rospackage_path_;
    // std::string map_frame_id_param_;
    // std::string camera_frame_id_param_;
    // std::string map_file_name_param_;
    // std::string voc_file_name_param_;
    // std::string settings_file_name_param_;
    // std::string folder_path_;
    // bool close_system_;
    // bool load_map_param_;
    // bool publish_pointcloud_param_;
    // bool publish_pose_param_;
    // int min_observations_per_point_;
};

#endif //Semantic_ROS_NODE_H_

