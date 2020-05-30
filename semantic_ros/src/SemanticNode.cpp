/*
 * @Author: 南山二毛
 * @Date: 2019-12-29 11:15:26
 * @LastEditTime : 2020-05-28 22:25:19
 * @LastEditors  : 南山二毛
 * @Description: In User Settings Edit
 * @FilePath: catkin_tagslam/src/semantic_ros/src/SemanticNode.cpp
 */

#include "SemanticNode.h"


SemanticNode::SemanticNode (ros::NodeHandle &node_handle, std::string config_file_path) : Node (node_handle, config_file_path) {
  std::cout << "Subscriber color_img_topic: " << color_img_topic_ << "." << std::endl;
  std::cout << "Subscriber depth_img_topic: " << depth_img_topic_ << "." << std::endl;
  rgb_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle, color_img_topic_, 1);
  depth_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle, depth_img_topic_, 1);

  sync_ = new message_filters::Synchronizer<sync_pol> (sync_pol(10), *rgb_subscriber_, *depth_subscriber_);
  sync_->registerCallback(boost::bind(&SemanticNode::ImageCallback, this, _1, _2));
  std::cout << "Start to subscribe topic. " << std::endl;
 
}


SemanticNode::~SemanticNode () {
  delete rgb_subscriber_;
  delete depth_subscriber_;
  delete sync_;
}


void SemanticNode::ImageCallback (const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImageConstPtr& msgD) {
  // Copy the ros image message to cv::Mat. 
  cv_bridge::CvImageConstPtr cv_ptrRGB;
  try {
      cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }

  cv_bridge::CvImageConstPtr cv_ptrD;
  try {
    cv_ptrD = cv_bridge::toCvShare(msgD);
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }

  current_frame_time_ = msgRGB->header.stamp;

  
  RGBDCalculate(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec()); // 主要处理程序入口
  
  // if(Update(cv_ptrRGB->header.stamp)){   // 发布ROS消息话题
  //   std::cout << "关闭ROS系统!" << std::endl; 
  // }
}


