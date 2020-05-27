/*
 * @Author: 王培荣
 * @Date: 2020-01-04 11:36:39
 * @LastEditTime : 2020-01-04 11:44:02
 * @LastEditors  : Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /catkin_ws/src/orbslam_semantic_nav_ros/costmap/src/record_path_node.cpp
 */

#include <ros/ros.h>
#include <ros/console.h> 
#include <iostream>
#include <string> 
#include <fstream>
#include <tf2_msgs/TFMessage.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_listener.h>  
#include <math.h>                                                                                                                    
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <termio.h>

using namespace cv;

using namespace std;
std::ofstream gtfile;

int scanKeyboard()
{
    int in;
    struct termios new_settings;
    struct termios stored_settings;
    tcgetattr(0,&stored_settings);
    new_settings = stored_settings;
    new_settings.c_lflag &= (~ICANON);
    new_settings.c_cc[VTIME] = 0;
    tcgetattr(0,&stored_settings);
    new_settings.c_cc[VMIN] = 1;
    tcsetattr(0,TCSANOW,&new_settings);
    
    in = getchar();
    
    tcsetattr(0,TCSANOW,&stored_settings);
    return in;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "record_path");
    ros::start();
    ros::NodeHandle nh;
    std::cout << "\n-------------------------- start --------------------------\n" << std::endl;
    gtfile.open("/home/scjy/code/catkin_tagslam/src/path_plan_control/config/record_path.txt",std::ios::in);
    if(!gtfile){
        std::cout << "Unable to open otfile";
        exit(1);
    }  

    tf::StampedTransform transform_room_robot;   //定义存放变换关系的变量 
    tf::TransformListener listener;
    int key;

    // ------------------------------------------------------
    // 接收机器人在室内坐标系的TF坐标
    // ------------------------------------------------------
    while(1){ 
        try
        {
            //得到child_frame坐标系原点，在frame坐标系下的坐标与旋转角度
            listener.lookupTransform("/room_frame", "/robot_base",ros::Time(0), transform_room_robot);                   
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep(); 
            continue;
        }
        double siny_cosp = +2.0 * (transform_room_robot.getRotation().getW() * transform_room_robot.getRotation().getZ() + 
            transform_room_robot.getRotation().getX() * transform_room_robot.getRotation().getY());
        double cosy_cosp = +1.0 - 2.0 * (transform_room_robot.getRotation().getY() * transform_room_robot.getRotation().getY()
            + transform_room_robot.getRotation().getZ()  * transform_room_robot.getRotation().getZ() );
        double yaw = atan2(siny_cosp, cosy_cosp);
        double pos_x = transform_room_robot.getOrigin().getX();
        double pos_y = transform_room_robot.getOrigin().getY();
        key = scanKeyboard();
        if(key == 27) // ESC退出
            break;
        switch(key){
            case 'r':
                std::cout << "record a point:  "; 
                std::cout << pos_x << "  " << pos_y << "  "  << yaw << std::endl;
                gtfile << to_string(pos_x) << " " << to_string(pos_y) << " " << to_string(yaw) << std::endl; 
                break;
            case 's':
                std::cout << "save and close file! " << std::endl;
                gtfile.close();
                break;
        } 
        ros::spinOnce();
    }  
    ros::shutdown();
    gtfile.close();
    return 0;
}

 
