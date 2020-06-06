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
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h> 
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Bool.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <path_plan_control/nav_one_point.h>
#include "path_plan_control/dwa_planning.h"
#include "path_plan_control/lrgbd2xz.h"
#include "aubo_arm_usr/armmovemotion.h"  

enum robot_move_state {Running=0, Waiting=1};

DWAPlanning dwa_planer;
LRGBDCostMap costmaper;
Eigen::Vector3d robot_pose;  
ros::Publisher speed_pub; 
ros::Publisher path_pub;
ros::Subscriber stop_sub;
ros::ServiceServer nav_dest_point;
ros::ServiceClient arm_move_motion_client;

int current_state = Waiting;
float tolerance_max = 0.0;

// 毫秒延时
static void sleep_ms(unsigned int secs) 
{ 
    struct timeval tval; 
    tval.tv_sec=secs/1000; 
    tval.tv_usec=(secs*1000)%1000000; 
    select(0,NULL,NULL,NULL,&tval); 
}

// 判断是否进行横向控制           
bool isXAxisErrorCorrect(double current_x, double error_limit){

    if( (dwa_planer.robot_waypoint_(0)-current_x) > error_limit || (current_x - dwa_planer.robot_waypoint_(0)) > error_limit){
        return true;
    }
    else{
        return false;
    }
}

// 更新机器人位姿
bool updateRobotPose(){
    // ------------------------------------------------------
    // 接收机器人在室内坐标系的TF坐标
    // ------------------------------------------------------
    geometry_msgs::Twist pub_speed;                 // 发送速度执行
    tf::TransformListener listener;
    tf::StampedTransform transform_room_robot;   //定义存放变换关系的变量 
    try
    {
        //得到child_frame坐标系原点，在frame坐标系下的坐标与旋转角度
        listener.lookupTransform("/room_frame", "/robot_base",ros::Time(0), transform_room_robot);                   
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        pub_speed.linear.x = 0.0;
        pub_speed.angular.z = 0.0; 
        speed_pub.publish(pub_speed); 
        return false;
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
    robot_pose(0) = transform_room_robot.getOrigin().getX();
    robot_pose(1) = transform_room_robot.getOrigin().getY();
    robot_pose(2) = yaw;
    std::cout << "输出当前方位角" << yaw << ";  当前坐标:" << transform_room_robot.getOrigin().getX() << "," << transform_room_robot.getOrigin().getY() << ";" << std::endl;
    return true;
}

// 横向控制   这个是一个阻塞程序，后面需要改进为线程控制
void correctXAxisControl(){

    geometry_msgs::Twist pub_speed;                 // 发送速度执行
    double go_v = 0, turn_v = 0;
    double angle_error = 0.0;
    double x_axis_distance_error = 0.0;

    // 需要调的参数
    int control_period = 50; // 控制周期50ms 确保高精度控制
    double turn_high_speed = 1.0;
    double turn_low_speed = 0.2;
    double go_high_speed = 0.3;
    double go_low_speed = 0.1;
    double turn_change_angle = 0.5;   // 旋转分段控制阈值
    double go_change_distance = 0.05; // 直线分段控制阈值
    double angle_accurete_control_error = 0.05; // 角度控制精度
    double distance_accurete_control_error = 0.02; // 位移控制精度
    // x轴朝向判断
    double dir = 1.57;
    if(dwa_planer.robot_waypoint_(0) > robot_pose(0)){
        dir = -1.57;
    }
    
    // 只控制机器人转x轴选旋转方向
    while(true){
        updateRobotPose(); 
        angle_error = dwa_planer.angle_err_calcu(dir, robot_pose(2));
        if(angle_error > angle_accurete_control_error){
            if(angle_error > turn_change_angle)
            {
                turn_v = turn_high_speed;
            }else{
                turn_v = turn_low_speed;
            }
        }
        else if(-angle_error > angle_accurete_control_error){
            if(-angle_error > turn_change_angle)
            {
                turn_v = -turn_high_speed;
            }else{
                turn_v = -turn_low_speed;
            }
        }else{
            std::cout << "控制机器人转x轴选旋转方向OK" << std::endl;
            break;
        }
        pub_speed.linear.x = 0.0;
        pub_speed.angular.z = turn_v; 
        speed_pub.publish(pub_speed);
        sleep_ms(control_period);
    }

    // 只控制机器人x轴位移误差
    while(true){
        updateRobotPose();
        x_axis_distance_error = dwa_planer.robot_waypoint_(0) - robot_pose(0);
        if(x_axis_distance_error > distance_accurete_control_error){
            if(x_axis_distance_error > go_change_distance)
            {
                go_v = go_high_speed;
            }else{
                go_v = go_low_speed;
            }
        }
        else if(-x_axis_distance_error > distance_accurete_control_error){
            if(-x_axis_distance_error > go_change_distance)
            {
                go_v = -go_high_speed;
            }else{
                go_v = -go_low_speed;
            }
        }else{
            std::cout << "控制机器人x轴位移误差OK" << std::endl; 
            break;
        } 
        pub_speed.linear.x = go_v;
        pub_speed.angular.z = 0.0;
        speed_pub.publish(pub_speed); 
        sleep_ms(control_period);
        ros::spinOnce();
    }

    // 只控制机器人y轴选旋转误差
    dir = 0;
    if(dwa_planer.robot_waypoint_(1) < robot_pose(1)){
        dir = 3.14;
    }
    while(true){
        updateRobotPose();
        angle_error = dwa_planer.angle_err_calcu(dir, robot_pose(0));
        if(angle_error > angle_accurete_control_error){
            if(angle_error > turn_change_angle)
            {
                turn_v = turn_high_speed;
            }else{
                turn_v = turn_low_speed;
            }
        }
        else if(-angle_error > angle_accurete_control_error){
            if(-angle_error > turn_change_angle)
            {
                turn_v = -turn_high_speed;
            }else{
                turn_v = -turn_low_speed;
            }
        }else{
            std::cout << "控制机器人y轴选旋转误差OK" << std::endl;
            break;
        }
        pub_speed.linear.x = 0.0;
        pub_speed.angular.z = turn_v; 
        speed_pub.publish(pub_speed);
        sleep_ms(control_period);
    }
}


void stop_nav_sub(std_msgs::Bool::ConstPtr msg){ 
        current_state = Waiting;
        std::cout << "控制机器人停止导航:" << current_state <<  "  !" <<  std::endl;  
}



bool nav_dest_res(path_plan_control::nav_one_point::Request &req,
        path_plan_control::nav_one_point::Response &res)
{
    // ------------------------------------------------------
    // step1 初始化
    // ------------------------------------------------------  
    double go_v = 0, turn_v = 0;
    geometry_msgs::Twist pub_speed;                 // 发送速度执行
    int sim_period = static_cast<int>(dwa_planer.getSimPeriod() * 1000);
    double x_axis_error = 0.2; // 大于20cm进行一次横向控制
     
    std::cout << "sim_period： " <<  sim_period << " ms." << std::endl;

    // ------------------------------------------------------
    // step2 移动机械臂和kinect到达寻路状态
    // ------------------------------------------------------
    aubo_arm_usr::armmovemotion arm_move_motion_srv;
    arm_move_motion_srv.request.move_state = 1;
    if (arm_move_motion_client.call(arm_move_motion_srv))
    {
        // 注意我们的response部分中的内容只包含一个变量response，另，注>意将其转变成字符串
        ROS_INFO("开始控制机械臂运动到寻路状态。");
    }
    else
    {
        ROS_ERROR("Failed to call service aubo_arm");
        return 1;
    }  
    ROS_INFO("机械臂准备完毕。\n");
 
    // ------------------------------------------------------
    // step3 更新目标点 
    // ------------------------------------------------------
    current_state = Running; //修改机器人状态
    dwa_planer.robot_dest_point_(0) =  req.goal_x;  // 更新目标路标点
    dwa_planer.robot_dest_point_(1) =  req.goal_y;  
    dwa_planer.robot_dest_point_(2) =  req.goal_yaw;  
    dwa_planer.setAngleThresh(req.angle_tolerance); // 更行定位容许误差
    dwa_planer.setDistanceThresh(req.distance_tolerance); // 更行角度容许误差
     
    costmaper.changeRobotState(current_state);
    // ------------------------------------------------------
    // step4 发布所有路标点
    // ------------------------------------------------------
    std::queue<Eigen::Vector3d> queue_waypoints;
    std::vector<Eigen::Vector3d> path_all_waypoints = dwa_planer.getAllPathWaypoints();
    geometry_msgs::PoseArray pub_path_waypoint;
    pub_path_waypoint.header.frame_id = "/room_frame";
    pub_path_waypoint.header.stamp = ros::Time::now();
    for(int i = 0; i < path_all_waypoints.size(); i++){
        geometry_msgs::Pose temp_pose;
        temp_pose.position.x = path_all_waypoints[i](0);
        temp_pose.position.y = path_all_waypoints[i](1);
        temp_pose.position.z = 0;
        temp_pose.orientation.w = cos((path_all_waypoints[i](2)+1.57)/2);
        temp_pose.orientation.z = sin((path_all_waypoints[i](2)+1.57)/2);  
        pub_path_waypoint.poses.push_back(temp_pose);
        queue_waypoints.push(path_all_waypoints[i]);
    } 
    path_pub.publish(pub_path_waypoint);

    // ------------------------------------------------------
    // step5 更新第一个路标点 
    // ------------------------------------------------------
    dwa_planer.robot_waypoint_(0) =  queue_waypoints.front()(0);   
    dwa_planer.robot_waypoint_(1) =  queue_waypoints.front()(1);  
    dwa_planer.robot_waypoint_(2) =  queue_waypoints.front()(2);  
    queue_waypoints.pop();

    tf::TransformListener listener;
    tf::StampedTransform transform_room_robot;   //定义存放变换关系的变量 

    // ------------------------------------------------------
    // step6 等待到达目的地
    // ------------------------------------------------------
    while(current_state == Running)
    {    
        // if(!updateRobotPose()){
        //     continue;
        // }

        // ------------------------------------------------------
        // step6.1 得到当前机器人在世界中的坐标
        // ------------------------------------------------------
        try
        {
            // 得到child_frame坐标系原点，在frame坐标系下的坐标与旋转角度
            listener.lookupTransform("/room_frame", "/robot_base",ros::Time(0), transform_room_robot);                   
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            pub_speed.linear.x = 0.0;
            pub_speed.angular.z = 0.0; 
            speed_pub.publish(pub_speed); 
            // return false;
            continue;
        }

        // ------------------------------------------------------
        // step6.2 计算的到2D平面的坐标, 更新机器人的位姿
        // ------------------------------------------------------
        double siny_cosp = +2.0 * (transform_room_robot.getRotation().getW() * transform_room_robot.getRotation().getZ() + 
            transform_room_robot.getRotation().getX() * transform_room_robot.getRotation().getY());
        double cosy_cosp = +1.0 - 2.0 * (transform_room_robot.getRotation().getY() * transform_room_robot.getRotation().getY()
                + transform_room_robot.getRotation().getZ()  * transform_room_robot.getRotation().getZ() );
        double yaw = atan2(siny_cosp, cosy_cosp);

        dwa_planer.setRobotPose(transform_room_robot.getOrigin().getX(), transform_room_robot.getOrigin().getY(), yaw);
        robot_pose(0) = transform_room_robot.getOrigin().getX();
        robot_pose(1) = transform_room_robot.getOrigin().getY();
        robot_pose(2) = yaw;
        std::cout << "输出当前方位角" << yaw << ";  当前坐标:" << transform_room_robot.getOrigin().getX() << "," << transform_room_robot.getOrigin().getY() << ";" << std::endl;
  
        // ------------------------------------------------------
        // step6.3 判断是否到达位置，并进行一次控制
        // ------------------------------------------------------ 
        if(!dwa_planer.isArriveDestination()){    // 优先判断是否到达终点
            if(queue_waypoints.empty()){          // 前面的位置点都已经走完   
                if(!dwa_planer.isArriveDestination()){  //在目标点位置附近进行精确定位
                    // dwa_planer.move_accurate(go_v, turn_v); // 进行一次dwa控制
                    // if((robot_pose(0)-dwa_planer.robot_dest_point_(0))> 0.2 || (robot_pose(0)-dwa_planer.robot_dest_point_(0))>0.2){
                    //     correctXAxisControl();
                    // }else{
                        dwa_planer.move(go_v, turn_v); // 进行一次dwa控制
                    // }
                    pub_speed.linear.x = go_v;
                    pub_speed.angular.z = turn_v;
                    ROS_INFO_STREAM("dest go_v: " << go_v << "    turn_v: " << turn_v);
                    speed_pub.publish(pub_speed); 
                }
                else{  
                    std::cout << "Arrvied destation！" << std::endl;
                    current_state = Waiting; // 到达目的地，当前状态为等待状态
                    costmaper.changeRobotState(Waiting);
                } 
            }
            else{               //控制机器人经过路标点
                if(!dwa_planer.isArriveWayPoint()){
                    // if((robot_pose(0)-dwa_planer.robot_waypoint_(0))> 0.2 || (robot_pose(0)-dwa_planer.robot_waypoint_(0))>0.2){
                    //     correctXAxisControl();
                    // }else{
                        dwa_planer.move(go_v, turn_v); // 进行一次dwa控制
                    // }
                    pub_speed.linear.x = go_v;
                    pub_speed.angular.z = turn_v;
                    ROS_INFO_STREAM("go_v: " << go_v << "    turn_v: " << turn_v);
                    speed_pub.publish(pub_speed); 
                }
                else{ //更新下一个 waypoint
                    dwa_planer.robot_waypoint_(0) = queue_waypoints.front()(0);
                    dwa_planer.robot_waypoint_(1) = queue_waypoints.front()(1);
                    dwa_planer.robot_waypoint_(2) = queue_waypoints.front()(2);
                    queue_waypoints.pop();
                    std::cout << "更新路标点：" << dwa_planer.robot_waypoint_(0) << "  " << dwa_planer.robot_waypoint_(1) << "  " << dwa_planer.robot_waypoint_(2) << std::endl;
                    if(queue_waypoints.empty()){
                        std::cout << "即将到达终点！" << std::endl; 
                    }
                }
            } 
        }
        else{ // 到达终点
            std::cout << "Arrvied destation！" << std::endl;
            current_state = Waiting; // 到达目的地，当前状态为等待状态
            costmaper.changeRobotState(Waiting);
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

    ROS_INFO_STREAM("DWA Setting file path is: " << config_file+"/dwa_planning.yaml");
    dwa_planer.init(config_file+"/dwa_planning.yaml");
    dwa_planer.readPathWayPoint();
    
    ROS_INFO_STREAM("Costmap Setting file path is: " << config_file+"/costmap.yaml");
    costmaper.init(nh, config_file+"/costmap.yaml");
    costmaper.changeRobotState(Waiting);
    arm_move_motion_client = nh.serviceClient<aubo_arm_usr::armmovemotion>("/arm_move_motion");
    
    nav_dest_point = nh.advertiseService("/nav_dest", nav_dest_res); 
    
    speed_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    path_pub = nh.advertise<geometry_msgs::PoseArray>("/path_waypoint", 50);

    stop_sub = nh.subscribe<std_msgs::Bool>("/stop_nav",10, &stop_nav_sub);
    
    ros::spin();
    ROS_INFO("shutting down!");
    ros::shutdown();
    return 0;
}
