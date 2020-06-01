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
#include <ros/console.h>  
#include <tf_conversions/tf_eigen.h> 
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
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h> 
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <rviz_visual_tools/imarker_simple.h>
#include "RunDetect.h" // 目标检测运行线程====
#include "MapDrawer.h"   // 显示地图类
#include "ObjectDatabase.h"

class Node
{
  public:
    Node (ros::NodeHandle &node_handle, std::string config_file_path);
    ~Node ();

    // 消息驱动函数,整个算法的入口
    void RGBDCalculate(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp, Eigen::Matrix4f T_room_camera_f); //接收信息并进行处理

    // 该函数需要在主函数的while中才可以正常运行,放置与回调函数中会出现阻塞现象,原因还需要继续讨论???
    bool Update (); // 更新所有的ros topic  
    Eigen::Matrix4d base2camera_T_; // 相机坐标系到室内坐标系的变换

  protected: 
    void PublishRenderedImage ();  // 发布检测到的图像
    void trigger_voice_sub(std_msgs::Bool::ConstPtr msg);  //接受触发语音解说消息
    void publishLabelHelper(const Eigen::Affine3d& pose, const std::string& label);

    ros::Time current_frame_time_;
    std::string color_img_topic_;
    std::string depth_img_topic_;
    std::string package_path_;
    cv::Mat image_detect_;
    std::vector<Semantic_ros::Cluster> clusters_;

    // ros 发布和接收
    rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
    ros::Publisher voice_string_pub_;
    ros::Subscriber trigger_voice_sub_;
    image_transport::Publisher rendered_image_publisher_; 
    std::shared_ptr<image_transport::ImageTransport> it_; 

  private:  
    bool pub_voice_flag_;
    std::shared_ptr< Semantic_ros::RunDetect > mpRunDetect;         // 目标检测运行线程
    Semantic_ros::MapDrawer* mpMapDrawer;
    double fx_;
    double fy_;
    double cx_;
    double cy_;
    std::string RVIZ_MARKER_TOPIC_;   
};

#endif //Semantic_ROS_NODE_H_

