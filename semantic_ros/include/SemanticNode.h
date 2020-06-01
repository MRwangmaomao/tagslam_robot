/*
 * @Author: 南山二毛
 * @Date: 2019-12-29 11:15:10
 * @LastEditTime : 2020-05-28 22:19:14
 * @LastEditors  : 南山二毛
 * @Description: In User Settings Edit
 * @FilePath: /catkin_tagslam/src/semantic_ros/include/SemanticNode.h
 */ 

#ifndef Semantic_ROS_RGBDODE_H_
#define Semantic_ROS_RGBDODE_H_

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <eigen3/Eigen/Core>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> 
#include "Node.h"


class SemanticNode : public Node
{
  public:
    SemanticNode (ros::NodeHandle &node_handle, std::string config_file_path);
    ~SemanticNode ();
    void ImageCallback (const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);
    
  private:
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Subscriber<sensor_msgs::Image> *rgb_subscriber_;
    message_filters::Subscriber<sensor_msgs::Image> *depth_subscriber_;
    message_filters::Synchronizer<sync_pol> *sync_;
    tf::TransformListener listener_;
    tf::StampedTransform transform_room_robot_;   //定义存放变换关系的变量 
};

#endif //Semantic_ROS_RGBDODE_H_
