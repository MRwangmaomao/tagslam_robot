/*
 * @Author: your name
 * @Date: 2019-12-23 08:13:28
 * @LastEditTime : 2019-12-30 17:17:01
 * @LastEditors  : Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /catkin_ws/src/costmap_lrgbd_ros/include/costmap_lrgbd_ros/dwa_planning.h
 */

#include <iostream>
#include <fstream> 
#include <sstream>
#include <string>
#include <queue> 
#include <math.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <visualization_msgs/Marker.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cassert>

class DWAPlanning{

public:
    DWAPlanning();
    ~DWAPlanning(void); 
    
    void init(std::string config_file_path); 
    bool isArriveDestination();
    bool isArriveWayPoint(); 
    bool move(double & go_v, double & turn_v);
    bool move_accurate(double & go_v, double & turn_v);
    // visualization_msgs::Marker dest_waypoint_pub();
    
    void setRobotPose(double x, double y, double yaw);
    void setAngleThresh(double ang);
    void setDistanceThresh(double dis); 
    float getSimPeriod();
    Eigen::Vector3d robot_waypoint_; // 机器人路标点
    Eigen::Vector3d robot_dest_point_; // 机器人目标点
    bool readPathWayPoint();
    double angle_err_calcu(double dest_angle, double current_angle);
     
    std::vector<Eigen::Vector3d> getAllPathWaypoints();
    
private: 
    // void getWayPoint();
    // void getRobotLocalization();
    // void setWayPointInCostmap(); 


    bool dwa_control(const cv::Mat& config_map);
    // void dwa_display(cv::Mat& config_map);
    // void drawArrow(cv::Mat& img, int start_x, int start_y, double theta, int arraw_length);
    Eigen::Vector3d motion(Eigen::Vector3d x, double v_head, double v_theta);
    // void calc_to_waypoint_cost();
    
    Eigen::Vector3d robot_pose_; // 机器人在世界中的位姿
    double distance_threshold_;
    double angle_threshold_; 

    double max_acc_x_;
    double max_acc_theta_;
    double max_yaw_rate_;
    double max_speed_;
    double min_speed_;
    double foresee_;
    double sim_time_;
    double region_foresee_;
    double short_foresee_rate_;
    double sim_period_;
    bool aggressive_mode_;
    double go_v_;
    double turn_v_; 
    double delta_yam_rate_;
    double delta_speed_;
    double waypoint_position_tolerance_;
    double waypoint_angle_tolerance_;
    int waypoint_id_;
    long int robot_pose_id_;
    bool first_flag_;  
    Eigen::Matrix4d T_robot_waypoint_;  
    Eigen::Vector3d robot_wapoint_in_costmap_; //机器人在代价地图中的位置
    std::vector<Eigen::Vector3d> path_waypoints_;
    std::queue<Eigen::Vector3d> path_waypoints_queue_;
    
    int waypoint_num_;
    std::string waypoint_path_file_;
};
