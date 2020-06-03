/*
 * @Author: 南山二毛
 * @Date: 2019-12-29 11:15:26
 * @LastEditTime : 2020-05-28 14:36:17
 * @LastEditors  : 南山二毛
 * @Description: In User Settings Edit
 * @FilePath: /catkin_tagslam/src/semantic_ros/src/Node.cc
 */
  
#include "Node.h"

void Node::trigger_voice_sub(std_msgs::Bool::ConstPtr msg){ 
      pub_voice_flag_ = true;  
}

Node::Node (ros::NodeHandle &node_handle, std::string config_file_path) {
    ROS_INFO_STREAM("Setting file path is: " << config_file_path);
    it_ = std::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(node_handle));
    trigger_voice_sub_ = node_handle.subscribe<std_msgs::Bool>("/trigger_voice",10, boost::bind(&Node::trigger_voice_sub, this, _1)); // 接收语音出发消息 调用成员函数作为回调函数,使用bind方法
    rendered_image_publisher_ = it_->advertise ("/rendered_debug_image", 1);// 注册发布调试图像
    voice_string_pub_ = node_handle.advertise<std_msgs::String>("/voiceWords", 1); // 注册发布语音消息
    pub_voice_flag_ = false;
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
    Eigen::Quaterniond base2camera_q( -0.308,0.951, 0.016, 0.006);  // w x y z
    base2camera_T_.block(0,0,3,3) = base2camera_q.toRotationMatrix(); 
    base2camera_T_(0,3) = base2camera_p(0);
    base2camera_T_(1,3) = base2camera_p(1);
    base2camera_T_(2,3) = base2camera_p(2);
    base2camera_T_(3,3) = 1.0;
    RVIZ_MARKER_TOPIC_ = "/rviz_visual_tools"; 
    std::cout << "  semantic ros 载入模型参数如下所示:" << std::endl 
      << "    color_img_topic:" << color_img_topic_ << std::endl 
      << "    depth_img_topic:" << depth_img_topic_ << std::endl 
      << "    package_path:" << package_path_ << std::endl
      << "    fx:" << fx_ << std::endl 
      << "    fy:" << fy_ << std::endl 
      << "    cx:" << cx_ << std::endl
      << "    cy:" << cy_ << std::endl;
  
    mpRunDetect = std::make_shared<Semantic_ros::RunDetect>( package_path_ );
    mpMapDrawer = new Semantic_ros::MapDrawer(fx_, fy_, cx_, cy_);//地图显示
    
    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("/robot_base", RVIZ_MARKER_TOPIC_));
    visual_tools_->loadMarkerPub();  // create publisher before waiting
    visual_tools_->deleteAllMarkers();
    visual_tools_->enableBatchPublishing(); 
}
 
Node::~Node () { 
    delete mpMapDrawer;
}
 
void  Node::RGBDCalculate(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp, Eigen::Matrix4f T_room_camera_f){ 
    std::vector<Semantic_ros::Object> objects = mpRunDetect->Run(im, image_detect_);
    mpMapDrawer->UpdateOctomap(im, depthmap, T_room_camera_f, objects, clusters_); 
}



bool Node::Update() {
    // std::cout << "总共发现物体数量:" << clusters_.size() << std::endl;
    PublishRenderedImage();
    for(std::vector<Semantic_ros::Cluster>::iterator it = clusters_.begin();
         it != clusters_.end(); ++it){  
          // std::cout << "发现物体:" << (*it).object_name << " 位于 " << (*it).centroid << " 长宽高为 " << (*it).size << std::endl;
          Eigen::Vector3f size  = (*it).size;     // 尺寸
          Eigen::Vector3f cent  = (*it).centroid; //中心点
          // std::cout<< "obj: " << (*it).object_name << " " << (*it).prob << " "
          //      << cent[0] << " " << cent[1] << " " << cent[2] << " "
          //      << size[0] << " " << size[1] << " " << size[2] << " "
          //      << std::endl;
        Eigen::Affine3d pose1 = Eigen::Affine3d::Identity();
        pose1 = Eigen::Affine3d::Identity();
        pose1.translation().x() = cent[0];
        pose1.translation().y() = cent[1];
        pose1.translation().z() = cent[2];  
        double depth = size[0], width = size[1], height = size[2];
        std::string object_chinese_name;
        if((*it).object_name == "person"){ 
          object_chinese_name = "人";
          visual_tools_->publishWireframeCuboid(pose1, depth, width, height, rviz_visual_tools::PINK, "Wireframe Cuboid", 1);
        } 
        if((*it).object_name == "chair"){
          object_chinese_name = "椅子";
          visual_tools_->publishWireframeCuboid(pose1, depth, width, height, rviz_visual_tools::BLUE, "Wireframe Cuboid", 2); 
        }
          
        if((*it).object_name == "bottle"){
          object_chinese_name = "瓶子";
          visual_tools_->publishWireframeCuboid(pose1, depth, width, height, rviz_visual_tools::YELLOW, "Wireframe Cuboid", 3); 
        }
          
        // publishLabelHelper(pose1, (*it).object_name);  
        visual_tools_->trigger(); 
        if(pub_voice_flag_){
          pub_voice_flag_ = false;
          std_msgs::String voice_word;
          std::stringstream str;
          str << "在机器人前方" <<  (int)(cent[1]*100) << "厘米位置处发现一个" << object_chinese_name << ", 它的高度为" << (int)(height*100) << "厘米, 宽度为" << (int)(width*100) << "厘米, 长度为" << (int)(depth*100) << "厘米";
          voice_word.data = str.str();
          voice_string_pub_.publish(voice_word); 
        }
    }  
}




void Node::publishLabelHelper(const Eigen::Affine3d& pose, const std::string& label)
{
  Eigen::Affine3d pose_copy = pose;
  pose_copy.translation().x() -= 0.2;
  visual_tools_->publishText(pose_copy, label, rviz_visual_tools::WHITE, rviz_visual_tools::XXLARGE, false);
}

/**
 * @brief 发布调试图像
 * 
 * @param image 
 */
void Node::PublishRenderedImage () {
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = "/room_frame";
  const sensor_msgs::ImagePtr rendered_image_msg = cv_bridge::CvImage(header, "bgr8", image_detect_).toImageMsg();
  rendered_image_publisher_.publish(rendered_image_msg);
} 

