/*
 * @Author: 南山二毛
 * @Date: 2019-12-29 11:18:52
 * @LastEditTime : 2020-05-28 21:18:16
 * @LastEditors  : 南山二毛
 * @Description: 地图的可视化和3D目标分割
 * @FilePath: /catkin_tagslam/src/semantic_ros/include/MapDrawer.h
 */ 
 
#ifndef MAPDRAWER_H
#define MAPDRAWER_H
 
#include<mutex>

//===================new======
#include<opencv2/core/core.hpp>// opencv
#include<opencv2/features2d/features2d.hpp>// orb特征检测
// octomap
#include <octomap/ColorOcTree.h>
#include <octomap/Pointcloud.h>
#include <octomap/octomap.h>
// pcl
#include <pcl/io/pcd_io.h>// 读写
#include <pcl/common/transforms.h>// 点云坐标变换
#include <pcl/point_types.h>      // 点类型
#include <pcl/filters/voxel_grid.h>// 体素格滤波
#include <pcl/filters/passthrough.h>//  直通滤波
#include <pcl/sample_consensus/method_types.h>// 采样一致性，采样方法
#include <pcl/sample_consensus/model_types.h>// 模型
#include <pcl/segmentation/sac_segmentation.h>// 采样一致性分割
#include <pcl/filters/extract_indices.h>// 提取点晕索引


// =====================
#include "Detector.h"// 2d目标检测结果 
#include "MergeSG.h"// 融合2d和点云信息 到3d目标数据库



namespace Semantic_ros
{ 
class MergeSG;

class MapDrawer
{
public:
    MapDrawer(double fx_, double fy_, double cx_, double cy_);

    ~MapDrawer();
    
 
    // 显示稠密 octomap图====
    void DrawOctoMap();

    // 显示数据库 中的物体，3d框
    void DrawObject();

    // 保存 octomap地图====
    void SaveOctoMap(const char*filename);

    void RegisterObs(pcl::PointCloud<pcl::PointXYZRGB> mvObs);

    // 载入octomap====
    void LoadOctoMap(std::string folder_path);

    // 融合2d和点云信息 到3d目标数据库 
    MergeSG* mpMerge2d3d;
    
    int m_ShowOctotreeMap; // 不显示八叉树地图

    // 更新 octomap====
    void UpdateOctomap(const cv::Mat &img, const cv::Mat &dep, Eigen::Matrix4f camera_T, std::vector<Object>& objects, std::vector<Cluster>& clusters);
   
protected:
     // 生成当前帧的点云，简单滤波 并 分离地面 和 非地面
     void GeneratePointCloud( 
                             const cv::Mat &dep, Eigen::Matrix4f camera_T, 
                             pcl::PointCloud<pcl::PointXYZRGB>& ground,                  
                             pcl::PointCloud<pcl::PointXYZRGB>& nonground);

     // 生成当前帧的点云，简单滤波 并 分离地面 和 非地面
     void GeneratePointCloud(const cv::Mat &img, const cv::Mat &dep, Eigen::Matrix4f camera_T, 
                                   pcl::PointCloud<pcl::PointXYZRGB> &ground, 
                                   pcl::PointCloud<pcl::PointXYZRGB> &nonground,
                                   std::vector<Object> &objects, std::vector<Cluster>& merge_clusters);

     // 总octomao地图中插入，新生成的点云===
     void InsertScan(octomap::point3d sensorOrigin, 
                     pcl::PointCloud<pcl::PointXYZRGB>& ground, 
                     pcl::PointCloud<pcl::PointXYZRGB>& nonground);
      // 散斑??
     bool isSpeckleNode(const octomap::OcTreeKey &nKey);
     
     

 
private:

    pcl::PointCloud<pcl::PointXYZRGB> observation;  // 当前帧点云===
    double fx;
    double fy;
    double cx; 
    double cy;
    octomap::ColorOcTree *m_octree;
    octomap::KeyRay m_keyRay;   // temp storage for casting
    octomap::OcTreeKey m_updateBBXMin;
    octomap::OcTreeKey m_updateBBXMax;

    double m_maxRange;
    bool m_useHeightMap;

    double m_colorFactor;
    double m_res;// octomap 图精度
    unsigned m_treeDepth;
    unsigned m_maxTreeDepth;
    bool bIsLocalization; 
    
    octomap::OcTreeKey m_paddedMinKey, m_paddedMaxKey;
    inline static void updateMinKey(const octomap::OcTreeKey&in, 
                                    octomap::OcTreeKey& min)
    {
        for(unsigned int i=0; i<3; i++)
            min[i] = std::min(in[i], min[i]);
    }
    inline static void updateMaxKey(const octomap::OcTreeKey&in, 
                                    octomap::OcTreeKey& max)
    {
        for(unsigned int i=0; i<3; i++)
            max[i] = std::max(in[i], max[i]);
    }

};

} //namespace  

#endif // MAPDRAWER_H
