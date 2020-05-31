/*
 * @Author: 王培荣
 * @Date: 2019-12-29 10:10:42
 * @LastEditTime : 2020-01-04 00:03:47
 * @LastEditors  : Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /catkin_ws/src/orbslam_semantic_nav_ros/orbslam/include/RunDetect.h
 */
/**
* This file is part of ORB-SLAM2.
* 运行目标检测 的类
*/

#ifndef RUNDETECT_H
#define RUNDETECT_H
 
#include <condition_variable> // 多线程锁 状态变量
#include "Detector.h" // 2d目标检测结果===
#include <boost/make_shared.hpp>
#include <iostream> 
#include <string>
#include <vector>
#include <memory>
using namespace std;

namespace Semantic_ros
{
class Detector; // 声明目标检测类

class RunDetect
{

public: 
    std::vector<Object>  Run(const cv::Mat& im);// 线程运行函数====
    RunDetect(std::string filePath);
    ~RunDetect();
    std::shared_ptr<Detector> mDetector;// 目标检测对象====
protected: 
    // condition_variable  colorImgUpdated; 
    // 关键帧更新 <condition_variable> 头文件主要包含了与条件变量相关的类和函数。
    // 全局条件变量. 用于多线程之间的 相互等待！！！！！！！
    // condition_variable 类 参考 https://msdn.microsoft.com/zh-cn/magazine/hh874752(v=vs.120)
 
    int mDisplayDetect = 0;  
};
};
#endif
