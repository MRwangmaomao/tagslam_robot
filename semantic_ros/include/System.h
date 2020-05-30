/** 
* 工程入口函数 系统类 头文件===================
*/


#ifndef SYSTEM_H
#define SYSTEM_H

#include <iostream>
#include <string>//字符串
#include <thread>// 线程
#include <memory>
#include <opencv2/core/core.hpp>// opencv
#include <iomanip> // I/O流控制头文件
#include <unistd.h> // 大量针对系统调用的封装（英语：wrapper functions），如 fork、pipe 以及各种 I/O 原语（read、write、close 等等）。
#include <sys/resource.h> // 资源定义.用简单的脚本语言描述,由编译器控制
 

// // for point cloud viewing
// #include "pointcloudmapping.h"// 外部 点云建图类

// class PointCloudMapping; // 申明 点云可视化类

#include "RunDetect.h" // 目标检测运行线程====
// #include "MapDrawer.h"



// 命名空间========================
namespace Semantic_ros
{
// class MapDrawer;
class RunDetect;
  
// 本 System 类 的 声明
class System
{
public:
	  
public:

	// Initialize the SLAM system. It launches the Local Mapping, Loop Closing and Viewer threads.
	// 初始化系统  启动 多线程 
	System(std::string package_path);
 
	// Process the given rgbd frame. Depthmap must be registered to the RGB frame.
	// Input image: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
	// Input depthmap: Float (CV_32F).
	// Returns the camera pose (empty if tracking fails).
	// 深度 跟踪  返回相机位姿
	cv::Mat TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp);
 
	// This stops local mapping thread (map building) and performs only camera tracking.
	// 定位 + 跟踪 模式
	void ActivateLocalizationMode();
	// This resumes local mapping thread and performs SLAM again.
	// 建图 + 跟踪 模式
	void DeactivateLocalizationMode();

	// Returns true if there have been a big map change (loop closure, global BA)
	// since last call to this function
	bool MapChanged();

	// Reset the system (clear map)
	void Reset();// 重置====

	// All threads will be requested to finish.
	// It waits until all threads have finished.
	// This function must be called before saving the trajectory.
	void Shutdown();	// 退出=====
  
	void SaveOctomap(const char *filename); 

	cv::Mat mcurrent_position_;

private:
  
	// std::thread* mptPointCloudMapping; // 可视化PCL点云线程
	// Reset flag 线程重启标志 ===========================================
	// std::mutex mMutexReset;// 互斥量   保护 mbReset 变量
	// bool mbReset;
	// 显示地图对象 指针 ===
	// std::shared_ptr<MapDrawer> mpMapDrawer;
	// Change mode flags   系统模式=======================================
    // 使用std::mutex创建互斥量，通过调用成员函数lock()进行上锁，unlock()进行解锁。
    // 但不方便的是需要记住锁后 要 在 函数出口 再次 调用 unlock()  解锁. 
    // 因此可以用 std::lock_guard, 其会在构造的时候 提供 已锁 的互斥量，并在析构的时候进行解锁，从而保证自动管理。
	// std::mutex mMutexMode;// 互斥量=================
	// bool mbActivateLocalizationMode;// 跟踪 + 定位
	// bool mbDeactivateLocalizationMode;// 跟踪 + 建图

	// // Tracking state 跟踪线程 状态 ======================================
	// std::mutex mMutexState;// 互斥量=============
	// int mTrackingState;// 跟踪线程 状态 
	// std::vector<MapPoint*> mTrackedMapPoints; // 当前帧跟踪到的地图点 3d
	// std::vector<cv::KeyPoint> mTrackedKeyPointsUn;// 2d 关键点


//    // point cloud mapping  新添加=============================================
//  	shared_ptr<PointCloudMapping> mpPointCloudMapping; // 点云地图类 共享指针 类成员变量
       std::shared_ptr< RunDetect > mpRunDetect;         // 目标检测运行线程
//     };
 
    

}; // class System

};// namespace Semantic_ros

#endif // SYSTEM_H
