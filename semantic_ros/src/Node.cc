/*
 * @Author: 南山二毛
 * @Date: 2019-12-29 11:15:26
 * @LastEditTime : 2020-05-28 14:36:17
 * @LastEditors  : 南山二毛
 * @Description: In User Settings Edit
 * @FilePath: /catkin_tagslam/src/semantic_ros/src/Node.cc
 */
  
#include "Node.h"
#include <ros/ros.h>
#include <ros/console.h> 
#include <iostream>
#include <tf_conversions/tf_eigen.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <rviz_visual_tools/imarker_simple.h>

Node::Node (ros::NodeHandle &node_handle, std::string config_file_path) {
    ROS_INFO_STREAM("Setting file path is: " << config_file_path);
    it_ = std::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(node_handle));
    rendered_image_publisher_ = it_->advertise ("/rendered_debug_image", 1);// 注册发布调试图像
     
    cv::FileStorage fsSettings(config_file_path, cv::FileStorage::READ); 
    if(!fsSettings.isOpened()){
    std::cerr << std::endl <<
        std::endl << 
        "---------------------------------------------" << std::endl << 
        "---------------------------------------------" << std::endl << 
        "您的文件路径设置错误了，请在roslaunch中修改配置文件的路径！！！" << std::endl <<
        std::endl <<
        std::endl <<
        "祝您实验取得成功。" << std::endl << 
        "---------------------------------------------" << std::endl << 
        "---------------------------------------------" << std::endl;
        exit(1);
    } 
    fsSettings["color_img_topic"] >> color_img_topic_;
    fsSettings["depth_img_topic"] >> depth_img_topic_; 
    fsSettings["package_path"] >> package_path_;
    fsSettings["fx"] >> fx_;
    fsSettings["fy"] >> fy_;
    fsSettings["cx"] >> cx_;
    fsSettings["cy"] >> cy_;
    Eigen::Vector3d base2camera_p(-0.028,0.250,1.395);
    Eigen::Quaterniond base2camera_q(0.952,0.015,0.005,-0.307); 
    
    base2camera_T_.block(0,0,3,3) = base2camera_q.toRotationMatrix(); 
    base2camera_T_(0,3) = base2camera_p(0);
    base2camera_T_(1,3) = base2camera_p(1);
    base2camera_T_(2,3) = base2camera_p(2);
    base2camera_T_(3,3) = 1.0; 
    // std::cout << base2camera_T_ << std::endl << std::endl;
    std::cout << "  semantic ros 载入模型参数如下所示:" << std::endl 
      << "    color_img_topic:" << color_img_topic_ << std::endl 
      << "    depth_img_topic:" << depth_img_topic_ << std::endl 
      << "    package_path:" << package_path_ << std::endl
      << "    fx:" << fx_ << std::endl 
      << "    fy:" << fy_ << std::endl 
      << "    cx:" << cx_ << std::endl
      << "    c:" << cy_ << std::endl;
 
    // if (publish_pointcloud_param_) {
    //   map_points_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2> (name_of_node_+"/map_points", 1); // 注册发布地图点云
    // }
   mpRunDetect = std::make_shared<Semantic_ros::RunDetect>( package_path_ );
   mpMapDrawer = new Semantic_ros::MapDrawer(fx_, fy_, cx_, cy_);//地图显示
}
 
Node::~Node () { 
    delete mpMapDrawer;
}
 
void  Node::RGBDCalculate(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp, Eigen::Matrix4f T_room_camera_f){
    
    std::vector<Semantic_ros::Object> objects = mpRunDetect->Run(im);
    mpMapDrawer->UpdateOctomap(im, depthmap, T_room_camera_f, objects);
    
}



bool Node::Update(ros::Time current_stamp) {
//     cv::Mat track_result = orb_slam_->mcurrent_position_; // 通过orb_slam 对象获取当前姿态 
    
//     while (orb_slam_->LocalMappingStopped())
//     {
//         void();
//     }
    
//     // publish
//     std::vector<std::pair<cv::Mat, double>> result_vector; 
//     orb_slam_->GetAllPoses(result_vector);
//     nav_msgs::Path result_path;
//     result_path.header.stamp = current_stamp;
//     result_path.header.frame_id = "world";
//     Eigen::Matrix4d temp_matrix, temp_matrix_inverse;
//     Eigen::Matrix4d trans_form = Eigen::Matrix4d::Identity();
//     // trans_form << 0,0,1,0, -1,0,0,0, 0,-1,0,0, 0,0,0,1;
//     for (int i = 0; i < result_vector.size(); i++)
//     {
//         geometry_msgs::PoseStamped this_pose;
//         for (int j = 0; j < receive_time_stamp_.size(); j++)
//             if (fabs(receive_time_stamp_[j].toSec() - result_vector[i].second) < 0.001)
//             {
//                 this_pose.header.stamp = receive_time_stamp_[j];
//                 break;
//             }

//         for (int row_i = 0; row_i < 4; row_i++)
//             for (int col_i = 0; col_i < 4; col_i++)
//                 temp_matrix(row_i, col_i) = result_vector[i].first.at<float>(row_i, col_i);

//         temp_matrix_inverse = trans_form * temp_matrix.inverse();
//         Eigen::Quaterniond rotation_q(temp_matrix_inverse.block<3, 3>(0, 0));
//         this_pose.pose.position.x = temp_matrix_inverse(0, 3);
//         this_pose.pose.position.y = temp_matrix_inverse(1, 3);
//         this_pose.pose.position.z = temp_matrix_inverse(2, 3);
//         this_pose.pose.orientation.x = rotation_q.x();
//         this_pose.pose.orientation.y = rotation_q.y();
//         this_pose.pose.orientation.z = rotation_q.z();
//         this_pose.pose.orientation.w = rotation_q.w();
//         result_path.poses.push_back(this_pose);
//     }
//     path_publish_.publish(result_path);

//     // get reference stamp
//     double reference_stamp;
//     reference_stamp = orb_slam_->GetRelativePose();
//     int reference_index = 0;
//     double time_diff = 1e9;
//     for (int i = 0; i < result_vector.size(); i++)
//     {
//         double this_time_diff = fabs(result_vector[i].second - reference_stamp);
//         if (this_time_diff < time_diff)
//         {
//             reference_index = i;
//             time_diff = this_time_diff;
//         }
//     }
//     // if (time_diff < 0.01)
//     //     printf("the reference keyframe is %d, keyframe number %d.\n", reference_index, result_vector.size());
//     // else
//     //     printf("cannot find the reference keyframe! time difference %f, the stamp is %f, current is %f.!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n",
//     //            time_diff,
//     //            reference_stamp,
//     //            current_stamp);


//        // get keyframe decision
//     LastKeyframeDecision_ = orb_slam_->GetKeyframeDecision();
//     // if (LastKeyframeDecision_)
//     //     printf("this is keyframe.\n");

//     nav_msgs::Odometry this_odometry;
//     this_odometry.header.stamp = current_stamp;
//     this_odometry.header.frame_id = "world";
//     Eigen::Matrix4d T_cw, T_wc;
//     for (int row_i = 0; row_i < 4; row_i++)
//         for (int col_i = 0; col_i < 4; col_i++)
//             T_cw(row_i, col_i) = track_result.at<float>(row_i, col_i);
//     T_wc = T_cw.inverse();
//     Eigen::Quaterniond rotation_q(T_wc.block<3, 3>(0, 0));
//     this_odometry.pose.pose.position.x = T_wc(0, 3);
//     this_odometry.pose.pose.position.y = T_wc(1, 3);
//     this_odometry.pose.pose.position.z = T_wc(2, 3);
//     this_odometry.pose.pose.orientation.x = rotation_q.x();
//     this_odometry.pose.pose.orientation.y = rotation_q.y();
//     this_odometry.pose.pose.orientation.z = rotation_q.z();
//     this_odometry.pose.pose.orientation.w = rotation_q.w();
//     if (LastKeyframeDecision_)
//         this_odometry.pose.covariance[0] = 1;
//     else
//         this_odometry.pose.covariance[0] = 0;
//     this_odometry.pose.covariance[1] = reference_index;
//     pose_publish_.publish(this_odometry);

//     // get loop index
//     sensor_msgs::PointCloud ros_loop_info;
//     ros_loop_info.header.stamp = current_stamp;
//     ros_loop_info.header.frame_id = "world";
//     std::vector<std::pair<double, double>> loop_result;
//     orb_slam_->GetLoopInfo(loop_result);
//     sensor_msgs::ChannelFloat32 loop_channel;
//     for (int i = 0; i < loop_result.size() && i < 35; i++)
//     {
//         int first_index = -1;
//         int second_index = -1;
//         for (int j = 0; j < result_vector.size(); j++)
//         {
//             if (result_vector[j].second == loop_result[i].first)
//                 first_index = j;
//             if (result_vector[j].second == loop_result[i].second)
//                 second_index = j;
//         }
//         if (first_index > 0 && second_index > 0)
//         {
//             // printf("the loop info %d <---> %d\n", first_index, second_index);
//             loop_channel.values.push_back(first_index);
//             loop_channel.values.push_back(second_index);
//         }
//         // else
//             // printf("cannot find corresponding!\n");
//     }
//     ros_loop_info.channels.push_back(loop_channel);
//     loop_publish_.publish(ros_loop_info);
 
//   return close_system_;
}
 
// void Node::PublishMapPoints (std::vector<ORB_SLAM2::MapPoint*> map_points) {
  
// }

/**
 * @brief 发布调试图像
 * 
 * @param image 
 */
void Node::PublishRenderedImage (cv::Mat image) {
  std_msgs::Header header;
  header.stamp = current_frame_time_;
//   header.frame_id = map_frame_id_param_;
  const sensor_msgs::ImagePtr rendered_image_msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
  rendered_image_publisher_.publish(rendered_image_msg);
} 

