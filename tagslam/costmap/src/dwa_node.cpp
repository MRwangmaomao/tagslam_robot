

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
#include <tf2_msgs/TFMessage.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> 
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "costmap_lrgbd_ros/dwa_planning.h"

DWAPlanning dwa_planer; 
Eigen::Matrix4d robot_pose;
Eigen::Matrix4d robot_dest;
std::string robot_pose_topic = "robot_pose";
std::string robot_dest_topic = "robot_dest";
ros::Publisher speed_pub;
bool dest_change = false;

void robot_pose_callback(const tf2_msgs::TFMessage::ConstPtr msg){
    double go_v = 0, turn_v = 0;

    robot_pose(0,3) = msg->transforms.at(0).transform.translation.x;
    robot_pose(1,3) = msg->transforms.at(0).transform.translation.y;
    robot_pose(2,3) = msg->transforms.at(0).transform.translation.z; 
    Eigen::Quaterniond robot_Q(msg->transforms.at(0).transform.rotation.w, msg->transforms.at(0).transform.rotation.x, 
        msg->transforms.at(0).transform.rotation.y, msg->transforms.at(0).transform.rotation.z);
    robot_pose.block(0,0,3,3) = robot_Q.toRotationMatrix();
    if(!dest_change){
        if(!dwa_planer.isArriveDestination()){
            dwa_planer.move(0, robot_pose, go_v, turn_v);
            geometry_msgs::Twist pub_speed;
            pub_speed.linear.x = go_v;
            pub_speed.angular.z = turn_v;
            // ROS_INFO_STREAM("go_v: " << go_v <<"    turn_v: " << turn_v);
            speed_pub.publish(pub_speed); 
        }  
        else{
            dest_change = true; //到达目的地，相当于目标地址被修改，没有了目的地
        }  
    } 
}


void robot_dest_callback(const tf2_msgs::TFMessage::ConstPtr msg){

    robot_dest(0,3) = msg->transforms.at(0).transform.translation.x;
    robot_dest(1,3) = msg->transforms.at(0).transform.translation.y;
    robot_dest(2,3) = msg->transforms.at(0).transform.translation.z; 
    Eigen::Quaterniond robot_Q(msg->transforms.at(0).transform.rotation.w, msg->transforms.at(0).transform.rotation.x, 
        msg->transforms.at(0).transform.rotation.y, msg->transforms.at(0).transform.rotation.z);
    robot_dest.block(0,0,3,3) = robot_Q.toRotationMatrix();
    dest_change = true; //目的地址更新
}

int main(int argc, char **argv){
    ros::init(argc, argv, "lrgbd_costmap");
    ros::start();
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    std::cout << "\n-------------------------- start --------------------------\n" << std::endl;
    
    // ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    std::string config_file;
    std::string dwa_file; 
    robot_pose << 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 1.0;
 
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

    ros::Subscriber sub_robot_pose = nh.subscribe(robot_pose_topic, 100, robot_pose_callback);
    ros::Subscriber sub_robot_dest = nh.subscribe(robot_dest_topic, 10, robot_dest_callback);
    speed_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::spin();
    ROS_INFO("shutting down!");
    ros::shutdown();
    return 0;
}
