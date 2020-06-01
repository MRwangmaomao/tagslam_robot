/*
 * @Author: 南山二毛
 * @Date: 2019-12-29 01:47:26
 * @LastEditTime : 2020-05-28 00:33:39
 * @LastEditors  : 南山二毛
 * @Description: In User Settings Edit
 * @FilePath: catkin_tagslam/src/semantic_ros/src/kinect_camera_node.cpp
 */

#include<iostream> 
#include <ros/ros.h> 

#include "SemanticNode.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "Semantic_ros");
    if(argc != 2)
    {
        std::cerr << std::endl << "Usage: ./semantic_ros_node path_to_settings. You need add it. " << std::endl;
        return 1;
    }
    
    ros::start();
    ros::NodeHandle nh; 
    ROS_INFO("------------------------start semantic ros------------------------------"); 
    SemanticNode semantic_node(nh, argv[1]);
     tf::TransformBroadcaster br; 
  
    ros::Rate r(3); // 10 hz
    while (ros::ok())
    { 
        // br.sendTransform(tf::StampedTransform(tf::Transform(
        //             tf::Quaternion(0.951, 0.016, 0.006, -0.308),
        //             tf::Vector3(-0.028,0.250,1.395)) , ros::Time::now(), "room_frame", "camera_semantic"));
        semantic_node.Update();
        ros::spinOnce();     //轮转一次,返回
        r.sleep();        //休眠
    } 
    ros::shutdown();
    ROS_INFO("------------------------end semantic ros------------------------------");
    return 0;
}

