/*
 * @Author: 王培荣
 * @Date: 2019-12-29 10:10:42
 * @LastEditTime : 2020-01-04 00:32:43
 * @LastEditors  : Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /catkin_ws/src/orbslam_semantic_nav_ros/orbslam/src/RunDetect.cc
 */

/* This file is part of ORB-SLAM2-SSD-Semantic.
* 2d目标检测
*/
#include "RunDetect.h"

namespace Semantic_ros
{


RunDetect::RunDetect(std::string filePath)
{  
    mDisplayDetect = true; 
    mDetector = make_shared<Detector>(filePath);
}

RunDetect::~RunDetect()
{ 
}
 
 
std::vector<Object> RunDetect::Run(const cv::Mat& im, cv::Mat& image_detect)
{ 
    std::vector<Object> vobject;
    mDetector->Run(im, vobject); 
    if(vobject.size()>0)
    {
        mDetector->Show(im,vobject);
        if(mDisplayDetect){
            image_detect = mDetector->Show(im,vobject);
            // cv::imshow("2D Detect", mDetector->Show(im,vobject));
            // cvWaitKey(20);
        }  
    }
    return vobject; 
}

}