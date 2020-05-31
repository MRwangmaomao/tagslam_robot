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

  
// // 设置当前帧 相机姿======================================
// void MapDrawer::SetCurrentCameraPose(const cv::Mat &Tcw)
// {
//     unique_lock<mutex> lock(mMutexCamera);
//     mCameraPose = Tcw.clone();
// }

// // 获取相机位姿=============cv::mat===========================
// bool MapDrawer::GetCurrentCameraPos(cv::Mat &Rcw, cv::Mat  &Ow)
// {
//     bool  flag = false;
//     if(!mCameraPose.empty())
//     {
//         unique_lock<mutex> lock(mMutexCamera);
//         Rcw = mCameraPose.rowRange(0,3).colRange(0,3);
//         Ow  = -Rcw.t()*mCameraPose.rowRange(0,3).col(3);
//         flag = true;
//     }

//     return flag;
// }
 
  
// 在RVIZ中显示 数据库中的目标物体====
void MapDrawer::DrawObject()
{
    std::vector<Cluster>& Clusters = mpMerge2d3d->mpOD->mClusters;
    int objnumber = Clusters.size(); // 目标数据库大小
  
  if( objnumber >0)
  { 
    //std::cout<< "OD size: " << objnumber << std::endl;
    for(int m=0; m<objnumber; m++)
    {
        Cluster & cluster = Clusters[m];// 一个目标物体
        Eigen::Vector3f size  = cluster.size;     // 尺寸
        Eigen::Vector3f cent  = cluster.centroid; //中心点

//         // 长方体，8个顶点，12条线段
//         glBegin(GL_LINES); // 线段=======
//         glLineWidth(5);    // 线宽======
//         std::string name = cluster.object_name;
//         glRasterPos3f(cent(0), cent(1), cent(2)); 
        
        cv::Scalar color =  mpMerge2d3d->mpOD->getObjectColor(cluster.class_id); // 定义的物体颜色
//         glColor3f(color.val[0]/255.0, color.val[1]/255.0, color.val[2]/255.0);
        
//         // 12条线段 =============================================
//         // 上面 4
//         glVertex3f(cent[0]+size[0]/2.0,cent[1]-size[1]/2.0,cent[2]+size[2]/2.0);//
//         glVertex3f(cent[0]-size[0]/2.0,cent[1]-size[1]/2.0,cent[2]+size[2]/2.0);//

//         glVertex3f(cent[0]-size[0]/2.0,cent[1]-size[1]/2.0,cent[2]+size[2]/2.0);//
//         glVertex3f(cent[0]-size[0]/2.0,cent[1]+size[1]/2.0,cent[2]+size[2]/2.0);//

//         glVertex3f(cent[0]-size[0]/2.0,cent[1]+size[1]/2.0,cent[2]+size[2]/2.0);//
//         glVertex3f(cent[0]+size[0]/2.0,cent[1]+size[1]/2.0,cent[2]+size[2]/2.0);//

//         glVertex3f(cent[0]+size[0]/2.0,cent[1]+size[1]/2.0,cent[2]+size[2]/2.0);//
//         glVertex3f(cent[0]+size[0]/2.0,cent[1]-size[1]/2.0,cent[2]+size[2]/2.0);//
//         // 一周 4
//         glVertex3f(cent[0]+size[0]/2.0,cent[1]-size[1]/2.0,cent[2]+size[2]/2.0);//
//         glVertex3f(cent[0]+size[0]/2.0,cent[1]-size[1]/2.0,cent[2]-size[2]/2.0);//

//         glVertex3f(cent[0]-size[0]/2.0,cent[1]-size[1]/2.0,cent[2]+size[2]/2.0);//
//         glVertex3f(cent[0]-size[0]/2.0,cent[1]-size[1]/2.0,cent[2]-size[2]/2.0);//

//         glVertex3f(cent[0]-size[0]/2.0,cent[1]+size[1]/2.0,cent[2]+size[2]/2.0);//
//         glVertex3f(cent[0]-size[0]/2.0,cent[1]+size[1]/2.0,cent[2]-size[2]/2.0);//

//         glVertex3f(cent[0]+size[0]/2.0,cent[1]+size[1]/2.0,cent[2]+size[2]/2.0);//
//         glVertex3f(cent[0]+size[0]/2.0,cent[1]+size[1]/2.0,cent[2]-size[2]/2.0);//
//         // 底面 4
//         glVertex3f(cent[0]+size[0]/2.0,cent[1]-size[1]/2.0,cent[2]-size[2]/2.0);//
//         glVertex3f(cent[0]-size[0]/2.0,cent[1]-size[1]/2.0,cent[2]-size[2]/2.0);//

//         glVertex3f(cent[0]-size[0]/2.0,cent[1]-size[1]/2.0,cent[2]-size[2]/2.0);//
//         glVertex3f(cent[0]-size[0]/2.0,cent[1]+size[1]/2.0,cent[2]-size[2]/2.0);//

//         glVertex3f(cent[0]-size[0]/2.0,cent[1]+size[1]/2.0,cent[2]-size[2]/2.0);//
//         glVertex3f(cent[0]+size[0]/2.0,cent[1]+size[1]/2.0,cent[2]-size[2]/2.0);//

//         glVertex3f(cent[0]+size[0]/2.0,cent[1]+size[1]/2.0,cent[2]-size[2]/2.0);//
//         glVertex3f(cent[0]+size[0]/2.0,cent[1]-size[1]/2.0,cent[2]-size[2]/2.0);//

//         glEnd();
    }
  }  
}


// // // 更新octomap======
void MapDrawer::UpdateOctomap(const cv::Mat &img, const cv::Mat &dep, Eigen::Matrix4f camera_T, std::vector<Object>& objects)
{    
    pcl::PointCloud<pcl::PointXYZRGB>  ground; // 地面点云
    pcl::PointCloud<pcl::PointXYZRGB>  nonground;// 无地面点云

    // 使用 2d目标检测结果 和点云 获取3d目标信息====
    // 使用另一个线程 对关键帧进行 2d目标检测
 
    if(objects.size()>0)
        GeneratePointCloud(img, dep, camera_T, ground, nonground, objects);// 生成点云
    else 
        GeneratePointCloud(dep, camera_T, ground, nonground);// 生成点云
    
    if(m_ShowOctotreeMap){
        octomap::point3d sensorOrigin = 
            octomap::point3d( camera_T(0,3), camera_T(1,3), camera_T(2,3));// 点云原点

    //     InsertScan(sensorOrigin, ground, nonground);// 将新点云 插入到 octomap地图中====
    } 
 

//   if(m_ShowOctotreeMap){
//     octomap::ColorOcTree::tree_iterator it  = m_octree->begin_tree();
//     octomap::ColorOcTree::tree_iterator end = m_octree->end_tree();
//     int counter = 0;// 计数
//     double occ_thresh = 0.8; // 概率阈值 原来 0.9  越大，显示的octomap格子越少
//     int level = 16; // 八叉树地图 深度???
//     glClearColor(1.0f,1.0f,1.0f,1.0f);// 颜色 + 透明度

//     glDisable(GL_LIGHTING);
//     glEnable (GL_BLEND);

//     ////DRAW OCTOMAP BEGIN//////
//     // double stretch_factor = 128/(1 - occ_thresh); //1280.0
//     // occupancy range in which the displayed cubes can be
 
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
            float d = dep.ptr<float>(m)[n];// 深度 m为单位 保留0～2m内的点
            //if (d < 0.01 || d>2.0) // 相机测量范围 0.5～6m
            if (d < 0.50 || d>3.0) // 相机测量范围 0.5～6m
                continue;
            pcl::PointXYZRGB p;
            p.z = d;
            p.x = ( n - cx) * p.z / fx;
            p.y = ( m - cy) * p.z / fy;
            if(p.y<-3.0 || p.y>3.0) continue;// 保留 垂直方向 -3～3m范围内的点 
            p.b = dep.ptr<uchar>(m)[n*3+0];// 点颜色=====
            p.g = dep.ptr<uchar>(m)[n*3+1];
            p.r = dep.ptr<uchar>(m)[n*3+2];
            cloud->points.push_back( p ); 
            // std::cout << p << std::endl;
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
                                   std::vector<Object>& objects)    // 传入2d检测结果
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
            if (d < 0.02 || d>100.0) // 相机测量范围 0.5～6m
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
    mpMerge2d3d->merge(objects, dep, temp);

    std::vector<Cluster>& Clusters = mpMerge2d3d->mpOD->mClusters;
    int objnumber = Clusters.size(); // 目标数据库大小
    std::cout<< "OD size: " << objnumber << std::endl;
    for( int m=0; m<objnumber; m++)
    {
      Cluster & cluster = Clusters[m];// 一个目标物体
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




// // 将新点云 插入到 octomap地图中====
// void MapDrawer::InsertScan(octomap::point3d sensorOrigin,  // 点云原点
//                            pcl::PointCloud<pcl::PointXYZRGB> &ground, //地面
//                            pcl::PointCloud<pcl::PointXYZRGB> &nonground)// 无地面
// {

// // 坐标转换到 key???
//     if(!m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMin)||
//         !m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMax))
//      {
//             printf("coulde not generate key for origin\n");
//      }

//      octomap::KeySet free_cells, occupied_cells;// 空闲格子，占有格子

// // 每一个 地面 点云=======================
//      for(auto p:ground.points)
//      {
//         octomap::point3d point(p.x, p.y, p.z);
//         // only clear space (ground points)
//         if(m_octree->computeRayKeys(sensorOrigin, point, m_keyRay))
//         {
//              free_cells.insert(m_keyRay.begin(), m_keyRay.end()); // 地面为空闲格子======
//              m_octree->averageNodeColor(p.x, p.y, p.z, p.r,p.g, p.b);//颜色
//         }
//         octomap::OcTreeKey endKey;
//         if(m_octree->coordToKeyChecked(point, endKey))
//         {
//               updateMinKey(endKey, m_updateBBXMin);
//               updateMaxKey(endKey, m_updateBBXMax);
//          }
//         else
//         {
//               printf("could not generator key for endpoint");
//         }
//      }

// // 无地面点云====================================
// // all other points : free on ray, occupied on endpoings:
//      for(auto p:nonground.points)
//      {
//          octomap::point3d point(p.x, p.y, p.z);
//          //free cell
//          if(m_octree->computeRayKeys(sensorOrigin, point, m_keyRay))
//          {
//             // free_cells.insert(m_keyRay.begin(),m_keyRay.end()); // 非空闲
//          }
//          //occupided endpoint
//          octomap::OcTreeKey key;
//          if(m_octree->coordToKeyChecked(point, key))
//          {
//              occupied_cells.insert(key); // 占有格子======
//              updateMinKey(key, m_updateBBXMin);
//              updateMaxKey(key, m_updateBBXMax);
//              m_octree->averageNodeColor(p.x, p.y, p.z, p.r,p.g, p.b);
//          }

//      }

//    //  pcl::PointCloud<pcl::PointXYZRGB>observation;

// // 空闲格子====
//      for(octomap::KeySet::iterator it = free_cells.begin(), 
//                                    end= free_cells.end(); 
//                                    it!=end; ++it)
//      {
//          if(occupied_cells.find(*it) == occupied_cells.end())// 占有格子未找到====
//          {
//              m_octree->updateNode(*it, false);// 空闲格子====
//          }
//      }
// // 占有格子====
//      for(octomap::KeySet::iterator it = occupied_cells.begin(), 
//                                    end= occupied_cells.end(); 
//                                    it!=end; ++it)
//      {
//          m_octree->updateNode(*it, true);// 占有格子====
//      }

//      m_octree->prune();
// }
 

// // 保存地图为octomap=====
// void MapDrawer::SaveOctoMap(const char *filename)
// {
//     std::ofstream outfile(filename, std::ios_base::out | std::ios_base::binary);
//     if (outfile.is_open())
//     {
//         m_octree->write(outfile);
//         outfile.close();
//         cerr << "OctoMap: saved " << endl;
//     }
// }
// // 载入octomap=====
// void MapDrawer::LoadOctoMap(std::string folder_path)
// { 
//     octomap::AbstractOcTree* tree = octomap::AbstractOcTree::read(folder_path + "octomap.ot");
//     m_octree= dynamic_cast<octomap::ColorOcTree*> (tree);
// }


} //namespace Semantic_ros
