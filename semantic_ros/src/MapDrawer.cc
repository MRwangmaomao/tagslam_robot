/** 
* 更新点云
            
*/

#include "MapDrawer.h"
#include <iostream>
#include <mutex> 

namespace Semantic_ros
{


MapDrawer::MapDrawer(double fx_, double fy_, double cx_, double cy_):
      m_octree(NULL),
      m_maxRange(-1.0),
      m_useHeightMap(true),
      m_res(0.05),
      m_colorFactor(0.8),
      m_treeDepth(0),
      m_maxTreeDepth(0),
      m_ShowOctotreeMap(1),
      fx(fx_),
      fy(fy_), 
      cx(cx_), 
      cy(cy_)  
{   
    mpMerge2d3d = new(MergeSG);
}


MapDrawer::~MapDrawer()
{
   delete mpMerge2d3d;
   delete m_octree; 
}

    
// // // 更新octomap======
void MapDrawer::UpdateOctomap(const cv::Mat &img, const cv::Mat &dep, Eigen::Matrix4f camera_T, std::vector<Object>& objects,  std::vector<Cluster>& clusters)
{    
    pcl::PointCloud<pcl::PointXYZRGB>  ground; // 地面点云
    pcl::PointCloud<pcl::PointXYZRGB>  nonground;// 无地面点云

    // 使用 2d目标检测结果 和点云 获取3d目标信息====
    // 使用另一个线程 对关键帧进行 2d目标检测
 
    if(objects.size()>0)
        GeneratePointCloud(img, dep, camera_T, ground, nonground, objects, clusters);// 生成点云
    else 
        GeneratePointCloud(dep, camera_T, ground, nonground);// 生成点云
    
    if(m_ShowOctotreeMap){
        octomap::point3d sensorOrigin = 
            octomap::point3d( camera_T(0,3), camera_T(1,3), camera_T(2,3));// 点云原点

    //     InsertScan(sensorOrigin, ground, nonground);// 将新点云 插入到 octomap地图中====
    }  
 
}

// // 生成当前帧的点云，简单滤波 并 分离地面 和 非地面 ============
void MapDrawer::GeneratePointCloud(
                                   const cv::Mat &dep, Eigen::Matrix4f camera_T, 
                                   pcl::PointCloud<pcl::PointXYZRGB> &ground, 
                                   pcl::PointCloud<pcl::PointXYZRGB> &nonground)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    for ( int m=0; m<(dep.rows); m+=1 )// 每一行
    {
        for ( int n=0; n<(dep.cols); n+=1 )//每一列
        {
            // float d = dep.ptr<float>(m)[n];// 深度 m为单位 保留0～2m内的点
            // //if (d < 0.01 || d>2.0) // 相机测量范围 0.5～6m
            // if (d < 0.50 || d>3.0) // 相机测量范围 0.5～6m
            //     continue;
            // pcl::PointXYZRGB p;
            // p.z = d;
            // p.x = ( n - cx) * p.z / fx;
            // p.y = ( m - cy) * p.z / fy;
            // if(p.y<-3.0 || p.y>3.0) continue;// 保留 垂直方向 -3～3m范围内的点 
            // // p.b = dep.ptr<uchar>(m)[n*3+0];// 点颜色=====
            // // p.g = dep.ptr<uchar>(m)[n*3+1];
            // // p.r = dep.ptr<uchar>(m)[n*3+2];
            // cloud->points.push_back( p ); 
            // // std::cout << p << std::endl;
            float d = double(dep.ptr<unsigned short>(m)[n])/1000.0;// 深度 m为单位 保留0～2m内的点
            
            //if (d < 0.01 || d>2.0) // 相机测量范围 0.5～6m
            if (d < 0.4 || d>6.0) // 相机测量范围 0.5～6m
            {
                
                continue;
            }
            // std::cout << d << "   ";
                
            //float z = d;
            float y = ( m - cy) * d / fy;
            if(y<-3.0 || y>3.0) continue;// 保留 垂直方向 -3～3m范围内的点 
            int ind = m * dep.cols + n;// 总索引
            cloud->points[ind].z = d;
            cloud->points[ind].x = ( n - cx) * d / fx;
            cloud->points[ind].y = y;
            // cloud->points[ind].b = img.ptr<uchar>(m)[n*3+0];// 点颜色=====
            // cloud->points[ind].g = img.ptr<uchar>(m)[n*3+1];
            // cloud->points[ind].r = img.ptr<uchar>(m)[n*3+2]; 
        }
    }
// 体素格滤波======
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.01,0.01, 0.01);// 体素格子 尺寸
    vg.filter(*cloud);

// 转换到世界坐标下==== 
    pcl::PointCloud<pcl::PointXYZRGB> temp;
    pcl::transformPointCloud( *cloud, temp, camera_T);

// 过滤 掉地面======
    if(temp.size()<50)
    {
        printf("pointcloud too small skip ground plane extraction\n;");
        ground = temp;
    }
// 这里可以简单剔除掉 y轴方向 > 机器人高度的 点，加速去除平面=======
    else // 随机采样一致性 模型分割=====
    {
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);// 模型系数
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);// 点云索引

        pcl::SACSegmentation<pcl::PointCloud<pcl::PointXYZRGB>::PointType> seg;//分割
        seg.setOptimizeCoefficients(true);// 优化系数
        seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);//分割 平面
        seg.setMethodType(pcl::SAC_RANSAC);// 随机采样一致性 分割
        seg.setMaxIterations(200);// 迭代 200次
        seg.setDistanceThreshold(0.04);// 距离阈值
        seg.setAxis(Eigen::Vector3f(0, 0 ,1));// xz 平面中的 平面点云 y方向轴====
        seg.setEpsAngle(0.5);

        pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered(temp);// 分割前的点云===
        pcl::ExtractIndices<pcl::PointCloud<pcl::PointXYZRGB>::PointType> extract;
        bool groundPlaneFound = false; // 地 平面找到 标志
        while(cloud_filtered.size()>10 && !groundPlaneFound)// 地平面未找到
        {
            seg.setInputCloud(cloud_filtered.makeShared());// 分割器输入点云
            seg.segment(*inliers, *coefficients);// 分割
            if(inliers->indices.size()==0)
            {
                break;
            }
            extract.setInputCloud(cloud_filtered.makeShared());// 点云提取器
            extract.setIndices(inliers);


            // a*X + b*Y + c*Z + d = 0;
            if (std::abs(coefficients->values.at(3)) >0.07)// 系数什么含义??
            {

                // printf("Ground plane found: %zu/%zu inliers. Coeff: %f %f %f %f \n", 
                // inliers->indices.size(),
                // cloud_filtered.size(),
                // coefficients->values.at(0),
                // coefficients->values.at(1),
                // coefficients->values.at(2),
                // coefficients->values.at(3));

                extract.setNegative (false);
                extract.filter (ground); // 提取 平面上的点 地面点============
                // remove ground points from full pointcloud:
                // workaround for PCL bug:
                if(inliers->indices.size() != cloud_filtered.size())
                {
                  extract.setNegative(true);
                  pcl::PointCloud<pcl::PointXYZRGB> cloud_out;
                  extract.filter(cloud_out);
                  nonground += cloud_out; // 无地面的点云
                  cloud_filtered = cloud_out;
                }

                groundPlaneFound = true;
            }
            else
            {
             printf("Horizontal plane (not ground) found: %zu/%zu inliers. Coeff: %f %f %f %f \n",     
                     inliers->indices.size(),
                     cloud_filtered.size(),
                     coefficients->values.at(0),
                     coefficients->values.at(1),
                     coefficients->values.at(2),
                     coefficients->values.at(3));

                pcl::PointCloud<pcl::PointXYZRGB> cloud_out;
                extract.setNegative (false);
                extract.filter(cloud_out);
                nonground +=cloud_out; // 未找到平面，
                if(inliers->indices.size() != cloud_filtered.size())
                {
                     extract.setNegative(true);
                     cloud_out.points.clear();
                     extract.filter(cloud_out);
                     cloud_filtered = cloud_out;
                }
                else
                {
                     cloud_filtered.points.clear();

                }
              }

         }//while 
    }
}



// // 生成当前帧的点云，简单滤波 并 分离地面 和 非地面 ============
void MapDrawer::GeneratePointCloud(const cv::Mat &img, const cv::Mat &dep, Eigen::Matrix4f camera_T, 
                                   pcl::PointCloud<pcl::PointXYZRGB> &ground, 
                                   pcl::PointCloud<pcl::PointXYZRGB> &nonground,
                                   std::vector<Object>& objects,
                                   std::vector<Cluster>& merge_clusters)    // 传入2d检测结果
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud->resize(dep.rows * dep.cols);
    cloud->width    =  dep.cols;  
    cloud->height   =  dep.rows;// 有序点云
    cloud->is_dense =  false;// 非稠密点云，会有不好的点,可能包含inf/NaN 这样的值   
    for ( int m=0; m<(dep.rows); m+=1 )// 每一行  /+3
    {
        for ( int n=0; n<(dep.cols); n+=1 )//每一列
        {
            float d = double(dep.ptr<unsigned short>(m)[n])/1000.0;// 深度 m为单位 保留0～2m内的点
            
            //if (d < 0.01 || d>2.0) // 相机测量范围 0.5～6m
            if (d < 0.4 || d>6.0) // 相机测量范围 0.5～6m
            {
                
                continue;
            }
            // std::cout << d << "   ";
                
            //float z = d;
            float y = ( m - cy) * d / fy;
            if(y<-3.0 || y>3.0) continue;// 保留 垂直方向 -3～3m范围内的点 
            int ind = m * dep.cols + n;// 总索引
            cloud->points[ind].z = d;
            cloud->points[ind].x = ( n - cx) * d / fx;
            cloud->points[ind].y = y;
            cloud->points[ind].b = img.ptr<uchar>(m)[n*3+0];// 点颜色=====
            cloud->points[ind].g = img.ptr<uchar>(m)[n*3+1];
            cloud->points[ind].r = img.ptr<uchar>(m)[n*3+2]; 
        }
    }
// 转换到世界坐标下==== 
    //pcl::PointCloud<pcl::PointXYZRGB> temp;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud( *cloud, *temp, camera_T);

// 3d目标信息获取=====
    mpMerge2d3d->merge(objects, dep, temp, merge_clusters);

    std::vector<Cluster>& clusters = mpMerge2d3d->mpOD->mClusters;
    int objnumber = clusters.size(); // 目标数据库大小
    // std::cout<< "OD size: " << objnumber << std::endl;
    for( int m=0; m<objnumber; m++)
    {
      Cluster & cluster = clusters[m];// 一个目标物体
      Eigen::Vector3f size  = cluster.size;     // 尺寸
      Eigen::Vector3f cent  = cluster.centroid; //中心点

    //   std::cout<< "obj: " << cluster.object_name << " " << cluster.prob << " "
    //            << cent[0] << " " << cent[1] << " " << cent[2] << " "
    //            << size[0] << " " << size[1] << " " << size[2] << " "
    //            << std::endl;
    }

// 体素格滤波======
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud(temp);
    vg.setLeafSize(0.01,0.01, 0.01);// 体素格子 尺寸
    vg.filter(*temp);


// 过滤 掉地面======
    if(temp->size()<50)
    {
        std::cout << "temp->size()" << temp->size() << std::endl; 
        printf("pointcloud too small skip ground plane extraction\n;");
        ground = *temp;
    }
// 这里可以简单剔除掉 z轴方向 > 机器人高度的 点，加速去除平面=======
    else // 随机采样一致性 模型分割=====
    {
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);// 模型系数
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);// 点云索引

        pcl::SACSegmentation<pcl::PointCloud<pcl::PointXYZRGB>::PointType> seg;//分割
        seg.setOptimizeCoefficients(true);// 优化系数
        seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);//分割 平面
        seg.setMethodType(pcl::SAC_RANSAC);// 随机采样一致性 分割
        seg.setMaxIterations(200);// 迭代 200次
        seg.setDistanceThreshold(0.04);// 距离阈值
        seg.setAxis(Eigen::Vector3f(0, 0 ,1));// xy 平面中的 平面点云 z方向轴====
        seg.setEpsAngle(0.5);

        pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered(*temp);// 分割前的点云===
        pcl::ExtractIndices<pcl::PointCloud<pcl::PointXYZRGB>::PointType> extract;
        bool groundPlaneFound = false; // 地 平面找到 标志
        while(cloud_filtered.size()>10 && !groundPlaneFound)// 地平面未找到
        {
            seg.setInputCloud(cloud_filtered.makeShared());// 分割器输入点云
            seg.segment(*inliers, *coefficients);// 分割
            if(inliers->indices.size()==0)
            {
                break;
            }
            extract.setInputCloud(cloud_filtered.makeShared());// 点云提取器
            extract.setIndices(inliers);


// a*X + b*Y + c*Z + d = 0;
            if (std::abs(coefficients->values.at(3)) >0.07)// 系数什么含义??
            {

                // printf("Ground plane found: %zu/%zu inliers. Coeff: %f %f %f %f \n", 
                // inliers->indices.size(),
                // cloud_filtered.size(),
                // coefficients->values.at(0),
                // coefficients->values.at(1),
                // coefficients->values.at(2),
                // coefficients->values.at(3));

                extract.setNegative (false);
                extract.filter (ground); // 提取 平面上的点 地面点============
                // remove ground points from full pointcloud:
                // workaround for PCL bug:
                if(inliers->indices.size() != cloud_filtered.size())
                {
                  extract.setNegative(true);
                  pcl::PointCloud<pcl::PointXYZRGB> cloud_out;
                  extract.filter(cloud_out);
                  nonground += cloud_out; // 无地面的点云
                  cloud_filtered = cloud_out;
                }

                groundPlaneFound = true;
            }
            else
            {
             printf("Horizontal plane (not ground) found: %zu/%zu inliers. Coeff: %f %f %f %f \n",     
                     inliers->indices.size(),
                     cloud_filtered.size(),
                     coefficients->values.at(0),
                     coefficients->values.at(1),
                     coefficients->values.at(2),
                     coefficients->values.at(3));

                pcl::PointCloud<pcl::PointXYZRGB> cloud_out;
                extract.setNegative (false);
                extract.filter(cloud_out);
                nonground +=cloud_out; // 未找到平面，
                if(inliers->indices.size() != cloud_filtered.size())
                {
                     extract.setNegative(true);
                     cloud_out.points.clear();
                     extract.filter(cloud_out);
                     cloud_filtered = cloud_out;
                }
                else
                {
                     cloud_filtered.points.clear();

                }
            }

        }//while

        if(!groundPlaneFound)
        {
            nonground = *temp;
        }
    }
}


 

} //namespace Semantic_ros
