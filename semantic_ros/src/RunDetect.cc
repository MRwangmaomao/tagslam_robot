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
    // mvKeyframes.clear();// 关键帧数组清空===
    mDisplayDetect = true;
    colorImgs.clear();  // 彩色图像====
    // mRunThread = make_shared<thread>( bind(&RunDetect::Run, this ) );// 可视化线程 共享指针 绑定 RunDetect::Run()函数
    mDetector = make_shared<Detector>(filePath);
}

RunDetect::~RunDetect()
{ 
}
 
 
void RunDetect::Run(const cv::Mat& im)
{
    if(mDisplayDetect){
        cv::namedWindow("2D Detect");
    }
    
    // while(1)
    // {
    //     {
    //         unique_lock<mutex> lck_colorImgUpdated( colorImgMutex); // 关键帧更新锁
    //         colorImgUpdated.wait( lck_colorImgUpdated );// 阻塞 关键帧更新锁
    //         // 需要等待 insertKeyFrame() 函数中完成 添加 关键帧 后，执行后面的!!
    //     }
    //     // 彩色图像===
    //     size_t N=0;
    //     {
    //         unique_lock<mutex> lck( colorImgMutex );// 关键帧锁
    //         N = colorImgs.size();                   // 当前 保存的 关键帧数量
    //     }
    //     for ( size_t i=lastKeyframeSize; i<N ; i++ )// 接着上次处理的地方开始
    //     {
           std::vector<Object> vobject;
           mDetector->Run(im, vobject); 
           if(vobject.size()>0)
           {
               mDetector->Show(im,vobject);
                if(mDisplayDetect){
                    cv::imshow("2D Detect", mDetector->Show(im,vobject));
                    cvWaitKey(20);
                } 
            //    std::cout << "detect : " << vobject.size() << " uums obj" << std::endl;
                // for(unsigned int j =0; j<vobject.size(); j++)
                // {
                //     unique_lock<mutex> lckObj(mvKeyframesMutex); // 2d检测结果上锁
                //     mvKeyframes[i]->mvObject.push_back(vobject[j]);// 存入目标检测结果库==== 
                // }
           }
    //     } 
    //     lastKeyframeSize = N;
    // }
    
}
}