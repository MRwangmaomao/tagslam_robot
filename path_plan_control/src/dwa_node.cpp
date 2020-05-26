// 南山二毛 2020.5.22日 更新
// 机器人轨迹控制程序

#include <ros/ros.h>
#include <ros/console.h>

#include <string>
#include <vector>
#include <queue>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/image_encodings.h> 
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> 
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <path_plan_control/nav_one_point.h>
#include "path_plan_control/dwa_planning.h"
#include "aubo_arm_usr/armmovemotion.h" 
// #include "aubo_arm_usr/armmovemotion.h"

enum robot_move_state {Running=0, Waiting=1};

DWAPlanning dwa_planer; 
Eigen::Matrix4d robot_pose;
Eigen::Matrix4d robot_dest;
std::string robot_pose_topic = "/tagslam/odom/body_rig";
std::string robot_dest_topic = "robot_dest";
ros::Publisher speed_pub; 
ros::Subscriber stop_sub;
ros::ServiceServer nav_dest_point;
ros::ServiceClient arm_move_motion_client;

int current_state = Waiting;
float tolerance_max = 0.0;

// void robot_pose_callback(const nav_msgs::OdometryConstPtr msg){
//     double go_v = 0, turn_v = 0;
//     std::cout << "接受到位姿消息" << std::endl; 
//     robot_pose(0,3) = msg->pose.pose.position.x;
//     robot_pose(1,3) = msg->pose.pose.position.y;
//     robot_pose(2,3) = msg->pose.pose.position.z; 
//     Eigen::Quaterniond robot_Q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, 
//         msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
//     robot_pose.block(0,0,3,3) = robot_Q.toRotationMatrix();
//     if(current_state == Running){                   // 移动状态
//         if(!dwa_planer.isArriveDestination()){      // 判断是否到达
//             ROS_INFO_STREAM("进行一次轨迹控制");
//     //         // dwa_planer.move(0, robot_pose, go_v, turn_v);   // 未到达则进行DWA控制
//     //         geometry_msgs::Twist pub_speed;                 // 发送速度执行
//     //         pub_speed.linear.x = go_v;
//     //         pub_speed.angular.z = turn_v;
//     //         // ROS_INFO_STREAM("go_v: " << go_v <<"    turn_v: " << turn_v);
//     //         speed_pub.publish(pub_speed); 
//         }  
//         else{                                        
//             current_state = Waiting; // 到达目的地，当前状态为等待状态
//         }  
//     }
// }
static void sleep_ms(unsigned int secs)

{

    struct timeval tval;

    tval.tv_sec=secs/1000;

    tval.tv_usec=(secs*1000)%1000000;

    select(0,NULL,NULL,NULL,&tval);

}

void stop_nav_sub(std_msgs::Bool::ConstPtr msg){
    if(msg->data)
        stop_nav_sub = Waiting;
}

bool nav_dest_res(path_plan_control::nav_one_point::Request &req,
        path_plan_control::nav_one_point::Response &res)
{
    // ------------------------------------------------------
    // 初始化
    // ------------------------------------------------------
    tf::StampedTransform transform_room_robot;   //定义存放变换关系的变量 
    tf::TransformListener listener;
    double go_v = 0, turn_v = 0;
    geometry_msgs::Twist pub_speed;                 // 发送速度执行
    int sim_period = static_cast<int>(dwa_planer.getSimPeriod() * 1000);
     
    std::cout << "sim_period：" <<  sim_period << " ms."<< std::endl;
    // ------------------------------------------------------
    // 移动机械臂和kinect到达寻路状态
    // ------------------------------------------------------
    aubo_arm_usr::armmovemotion arm_move_motion_srv;
    arm_move_motion_srv.request.move_state = 1;
    if (arm_move_motion_client.call(arm_move_motion_srv))
    {
        // 注意我们的response部分中的内容只包含一个变量response，另，注>意将其转变成字符串
        // ROS_INFO("Response from server: %d", arm_move_motion_srv.response.is_ok);
    }
    else
    {
        ROS_ERROR("Failed to call service aubo_arm");
        return 1;
    }  
    ROS_INFO("机械臂准备完毕。\n");

    // ------------------------------------------------------
    // 更新路标点 
    // ------------------------------------------------------
    current_state = Running; //修改机器人状态
    dwa_planer.robot_waypoint_(0) =  req.goal_x;  // 更新目标路标点
    dwa_planer.robot_waypoint_(1) =  req.goal_x;  
    dwa_planer.robot_waypoint_(2) =  req.goal_yaw;  
    dwa_planer.setAngleThresh(req.angle_tolerance); // 更行定位容许误差
    dwa_planer.setDistanceThresh(req.distance_tolerance); // 更行角度容许误差
    
    // ------------------------------------------------------
    // 等待到达目的地
    // ------------------------------------------------------
    while(current_state == Running)
    {  
        // ------------------------------------------------------
        // 接收机器人在室内坐标系的TF坐标
        // ------------------------------------------------------
        try
        {
            //得到child_frame坐标系原点，在frame坐标系下的坐标与旋转角度
            listener.lookupTransform("/room_frame", "robot_base",ros::Time(0), transform_room_robot);                   
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            pub_speed.linear.x = 0.0;
            pub_speed.angular.z = 0.0; 
            speed_pub.publish(pub_speed); 
            continue;
        }

        // ------------------------------------------------------
        // 更新机器人的位姿
        // ------------------------------------------------------
        double siny_cosp = +2.0 * (transform_room_robot.getRotation().getW() * transform_room_robot.getRotation().getZ() + 
            transform_room_robot.getRotation().getX() * transform_room_robot.getRotation().getY());
        double cosy_cosp = +1.0 - 2.0 * (transform_room_robot.getRotation().getY() * transform_room_robot.getRotation().getY()
             + transform_room_robot.getRotation().getZ()  * transform_room_robot.getRotation().getZ() );
        double yaw = atan2(siny_cosp, cosy_cosp);

        dwa_planer.setRobotPose(transform_room_robot.getOrigin().getX(), transform_room_robot.getOrigin().getY(), yaw);
          
        std::cout << "输出当前方位角" << yaw << ";  当前坐标:" << transform_room_robot.getOrigin().getX() << "," << transform_room_robot.getOrigin().getY() << ";" << std::endl;
        
        // ------------------------------------------------------
        // 判断是否到达位置，并进行一次控制
        // ------------------------------------------------------
        if(!dwa_planer.isArriveDestination())
        {
            dwa_planer.move(go_v, turn_v); // 进行一次dwa控制
            pub_speed.linear.x = go_v;
            pub_speed.angular.z = turn_v;
            ROS_INFO_STREAM("go_v: " << go_v << "    turn_v: " << turn_v);
            speed_pub.publish(pub_speed); 
        }
        else{
            go_v = 0;
            turn_v = 0;
            pub_speed.linear.x = go_v;
            pub_speed.angular.z = turn_v; 
            speed_pub.publish(pub_speed);  
            std::cout << "Arrvied destation！" << std::endl;
            current_state = Waiting; // 到达目的地，当前状态为等待状态
        } 
        sleep_ms(sim_period);
        // ros::spinOnce();
    } 
    pub_speed.linear.x = 0.0;
    pub_speed.linear.y = 0.0;
    pub_speed.linear.z = 0.0;
    pub_speed.angular.x = 0.0; 
    pub_speed.angular.y = 0.0; 
    pub_speed.angular.z = 0.0; 
    speed_pub.publish(pub_speed); 
    std::cout << "导航完成，速度设置为0." << std::endl;
    res.is_ok = true;
    return true;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "DWA_control");
    ros::start();
    ros::NodeHandle nh; 

    std::cout << "\n-------------------------- start DWA Control --------------------------\n" << std::endl;
    // ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    std::string config_file;
    std::string dwa_file;  
 
    if(argc >=2){
        config_file = argv[1]; 
    }
    else{
        std::cout<<"args too small.\n";
        exit(0);
    } 
    ROS_INFO_STREAM("Setting file path is: " << config_file);
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);

    dwa_planer.init(config_file);
    
    arm_move_motion_client = nh.serviceClient<aubo_arm_usr::armmovemotion>("/arm_move_motion");
    
    nav_dest_point = nh.advertiseService("/nav_dest", nav_dest_res); 
    
    speed_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    stop_sub = nh.subscribe("stop_nav", stop_nav_sub);

    ros::spin();
    ROS_INFO("shutting down!");
    ros::shutdown();
    return 0;
}
