
/*
 * @Author: 南山二毛
 * @Date: 2019-12-29 11:15:26
 * @LastEditTime : 2020-05-28 22:25:19
 * @LastEditors  : 南山二毛
 * @Description: In User Settings Edit
 * @FilePath: catkin_tagslam/src/semantic_ros/src/System.cpp
 */
 
/**
* This file is part of semantic_ros.
* 
* 系统入口:
* 
* 
* semantic_ros利用三个线程分别进行追踪、地图构建和闭环检测。

一、2D目标识别
 

二、2D目标与3D点云结合
 

三、稠密点云地图构建
 
 
*/
#include <iostream>
#include <thread> 
#include <time.h>
#include "System.h" 


namespace Semantic_ros
{  
    System::System(std::string package_path)
    {  
        // 
        // mpMapDrawer = make_shared<MapDrawer>();
    }

    // 深度相机 跟踪========================================================================================
    cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp)
    {
        // mpRunDetect->Run(im);
        cv::imshow("1",im);
        cvWaitKey(30);
    }

  
 
};//namespace Semantic_ros
