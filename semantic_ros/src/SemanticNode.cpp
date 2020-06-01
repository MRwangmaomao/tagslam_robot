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

  Eigen::Matrix4d T_room_camera_d;
  Eigen::Matrix4f T_room_camera_f;
   
  try
  {
      //得到child_frame坐标系原点，在frame坐标系下的坐标与旋转角度
      listener_.lookupTransform("/room_frame", "/robot_base",ros::Time(0), transform_room_robot_);                   
  }
  catch (tf::TransformException &ex)
  {
      ROS_ERROR("%s",ex.what());
      // ros::Duration(1.0).sleep(); 
  } 
  
  Eigen::Matrix4d T_room_robot(Eigen::Matrix4d::Identity());   
  T_room_robot(0,3) = transform_room_robot_.getOrigin().getX(); 
  T_room_robot(1,3) = transform_room_robot_.getOrigin().getY(); 
  T_room_robot(2,3) = transform_room_robot_.getOrigin().getZ(); 
  Eigen::Quaterniond q_room_robot; 
  q_room_robot.w() = static_cast<double>(transform_room_robot_.getRotation().getW());
  q_room_robot.x() = static_cast<double>(transform_room_robot_.getRotation().getX());
  q_room_robot.y() = static_cast<double>(transform_room_robot_.getRotation().getY());
  q_room_robot.z() = static_cast<double>(transform_room_robot_.getRotation().getZ()); 
  
  T_room_robot.block(0,0,3,3) = q_room_robot.toRotationMatrix(); 
  // T_room_camera_d = T_room_robot*base2camera_T_;
  for(int i = 0; i < 16; i++)
      T_room_camera_f(i) = base2camera_T_(i);
  //  std::cout <<"T_room_camera_f: " <<  T_room_camera_f << std::endl;
  
 

  RGBDCalculate(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec(), T_room_camera_f); // 主要处理程序入口 
}


