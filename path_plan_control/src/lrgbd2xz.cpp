
#include "path_plan_control/lrgbd2xz.h"

enum robot_move_state {Running=0, Waiting=1};

LRGBDCostMap::LRGBDCostMap(){
    
}

LRGBDCostMap::~LRGBDCostMap(void){
    
}

void LRGBDCostMap::init(ros::NodeHandle nh, std::string config_file_path){
    sub_kinect_cloud_ = nh.subscribe("/kinect2/qhd/points", 5, &LRGBDCostMap::pointCloudSub, this);
    rgb_costmap_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("/rgbd_cost_map", 5);

    cv::FileStorage fsSettings(config_file_path, cv::FileStorage::READ);
    groud_height_ = fsSettings["groud_height"]; 
    costmap_width_ = fsSettings["costmap_width"];
    costmap_height_ = fsSettings["costmap_height"];
    costmap_resolution_ = fsSettings["costmap_resolution"];
} 

void LRGBDCostMap::changeRobotState(int state){
    robot_state_ = state;
}    

void LRGBDCostMap::setTRobotBaseKinectoptical(Eigen::Matrix<double, 4, 4> t){
    T_robotbase_kinectoptical_ = t;
}    


void LRGBDCostMap::pointCloudSub(const sensor_msgs::PointCloud2 &cloud_msg){
    // std::cout << cloud_msg->header.frame_id << std::endl;
    if(robot_state_ == Running){ // 机器人在运动状态时开始进入避障模式，处理点云
        sensor_msgs::PointCloud out_pointcloud;     
        sensor_msgs::convertPointCloud2ToPointCloud(cloud_msg, out_pointcloud);
        nav_msgs::OccupancyGrid occupancy_map;

        occupancy_map.header.frame_id = "robot_base";
        occupancy_map.header.stamp = cloud_msg.header.stamp;
        occupancy_map.info.width      = costmap_width_;           // 宽
        occupancy_map.info.height     = costmap_height_;           // 高
        occupancy_map.info.resolution = costmap_resolution_;

        for(int i = 0; i < out_pointcloud.points.size(); i++){
            // std::cout << out_pointcloud.points[i].x << ", " << out_pointcloud.points[i].y << ", " << out_pointcloud.points[i].z << std::endl;
            Eigen::Vector4d point_in_kinect(out_pointcloud.points[i].x, out_pointcloud.points[i].y, out_pointcloud.points[i].z, 1.0);
            Eigen::Vector4d point_in_robot = T_robotbase_kinectoptical_ * point_in_kinect;
            if(point_in_robot(3) <= groud_height_){ //地面
                // occupancy_map.data.push_back();
                
            }
        } 
        rgb_costmap_pub_.publish(occupancy_map);
    }
}
 
