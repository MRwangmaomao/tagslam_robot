#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> 

class LRGBDCostMap{

public:
    LRGBDCostMap();
    ~LRGBDCostMap(void);
 
    void init(ros::NodeHandle nh, std::string config_file_path);
    void pointCloudSub(const sensor_msgs::PointCloud2 &cloud_msg);
    void changeRobotState(int state); 
    void setTRobotBaseKinectoptical(Eigen::Matrix<double, 4, 4> t);
private:

    ros::Subscriber sub_kinect_cloud_;
    ros::Publisher rgb_costmap_pub_;

    int groud_height_;
    int robot_state_;
    int costmap_width_;
    int costmap_height_;
    int costmap_resolution_;
    
    Eigen::Matrix<double, 4, 4> T_robotbase_kinectoptical_;
    Eigen::Matrix<double, 4, 4> T_robotbase_laser_; 
};