// This is an advanced implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014. 

// Modifier: Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk


// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#include <cmath>
#include <vector>
#include <string>
#include "aloam_velodyne/common.h"
#include "aloam_velodyne/tic_toc.h"
#include <nav_msgs/Odometry.h>
#include <opencv2/imgproc.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <dbg.h>

using std::atan2;
using std::cos;
using std::sin;

const double scanPeriod = 0.1;

const int systemDelay = 0; 
int systemInitCount = 0;
bool systemInited = false;
int N_SCANS = 0;
float cloudCurvature[400000];
int cloudSortInd[400000];
int cloudNeighborPicked[400000];
int cloudLabel[400000];

bool comp (int i,int j) { return (cloudCurvature[i] < cloudCurvature[j]); }

ros::Publisher pubLaserCloud;
ros::Publisher pubCornerPointsSharp;
ros::Publisher pubCornerPointsLessSharp;
ros::Publisher pubSurfPointsFlat;
ros::Publisher pubSurfPointsLessFlat;
ros::Publisher pubRemovePoints;
std::vector<ros::Publisher> pubEachScan;
std_msgs::Header cloudHeader;

bool PUB_EACH_LINE = false;

double MINIMUM_RANGE = 0.1; 

template <typename PointT>
void removeClosedPointCloud(const pcl::PointCloud<PointT> &cloud_in,
                              pcl::PointCloud<PointT> &cloud_out, float thres) {
    if (&cloud_in != &cloud_out) {
        cloud_out.header = cloud_in.header;
        cloud_out.points.resize(cloud_in.points.size());
    }

    size_t j = 0;

    // 去除距离lidar原点较近的点
    for (size_t i = 0; i < cloud_in.points.size(); ++i) {
        if (cloud_in.points[i].x * cloud_in.points[i].x + 
            cloud_in.points[i].y * cloud_in.points[i].y + 
            cloud_in.points[i].z * cloud_in.points[i].z < thres * thres) {
                continue;
            }
        cloud_out.points[j] = cloud_in.points[i];
        j++;
    }

    if (j != cloud_in.points.size()) {
        cloud_out.points.resize(j);
    }

    cloud_out.height = 1;
    cloud_out.width = static_cast<uint32_t>(j);
    cloud_out.is_dense = true;
}

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg) {
    ROS_INFO("receive a laserCloudMsg");
    if (!systemInited) {   // 延迟初始化
        systemInitCount++;
        if (systemInitCount >= systemDelay) {
            systemInited = true;
        } else { return; }
    }

    TicToc t_whole;
    TicToc t_prepare;
    std::vector<int> scanStartInd(N_SCANS, 0);
    std::vector<int> scanEndInd(N_SCANS, 0);

    pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
    pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);
    std::vector<int> indices;

    pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);   // 去除nan点
    removeClosedPointCloud(laserCloudIn, laserCloudIn, MINIMUM_RANGE);   // 去除到原点距离小于MINIMUM_RANGE的点
    cloudHeader = laserCloudMsg->header;
    // cloudHeader.stamp = ros::Time::now();

    int cloudSize = laserCloudIn.points.size();
    float startOri = -atan2(laserCloudIn.points[0].y, laserCloudIn.points[0].x);    // 得到起始和终止点的水平角度
    float endOri = -atan2(laserCloudIn.points[cloudSize - 1].y,
                          laserCloudIn.points[cloudSize - 1].x) + 2 * M_PI;

    if (endOri - startOri > 3 * M_PI) {
        endOri -= 2 * M_PI;
    }
    else if (endOri - startOri < M_PI) {
        endOri += 2 * M_PI;
    }
    // std::printf("start Ori %f\n", startOri);
    // std::printf("end Ori %f\n", endOri);

    bool halfPassed = false;
    int count = cloudSize;
    PointType point;
    std::vector<pcl::PointCloud<PointType>> laserCloudScans(N_SCANS);   // 将点云按照线束划分
    for (int i = 0; i < cloudSize; i++) {
        point.x = laserCloudIn.points[i].x;
        point.y = laserCloudIn.points[i].y;
        point.z = laserCloudIn.points[i].z;

        float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;   // 垂直角度
        int scanID = 0;   // 当前点所在线束ID

        // 根据垂直角度来获得当前点所在线束
        if (N_SCANS == 16) {
            scanID = int((angle + 15) / 2 + 0.5);
            if (scanID > (N_SCANS - 1) || scanID < 0) {
                count--;
                continue;
            }
        } else if (N_SCANS == 32) {
            scanID = int((angle + 92.0/3.0) * 3.0 / 4.0);
            if (scanID > (N_SCANS - 1) || scanID < 0) {
                count--;
                continue;
            }
        } else if (N_SCANS == 64) {   
            if (angle >= -8.83) {
                scanID = int((2 - angle) * 3.0 + 0.5);
            } else {
                scanID = N_SCANS / 2 + int((-8.83 - angle) * 2.0 + 0.5);
            }

            // use [0 50]  > 50 remove outlies 
            if (angle > 2 || angle < -24.33 || scanID > 50 || scanID < 0) {
                count--;
                continue;
            }
        } else {
            printf("wrong scan number\n");
            ROS_BREAK();
        }
        // std::printf("angle %f scanID %d \n", angle, scanID);   // 得到当前点垂直角度以及所在线束ID

        float ori = -atan2(point.y, point.x);
        if (!halfPassed) { 
            if (ori < startOri - M_PI / 2) {
                ori += 2 * M_PI;
            } else if (ori > startOri + M_PI * 3 / 2) {
                ori -= 2 * M_PI;
            }

            if (ori - startOri > M_PI) {
                halfPassed = true;
            }
        } else {
            ori += 2 * M_PI;
            if (ori < endOri - M_PI * 3 / 2) {
                ori += 2 * M_PI;
            } else if (ori > endOri + M_PI / 2) {
                ori -= 2 * M_PI;
            }
        }

        float relTime = (ori - startOri) / (endOri - startOri);
        point.intensity = scanID + scanPeriod * relTime;    // 点云强度通道设置为当前线束ID + 当前点时间偏移
        // std::printf("point.intensity: %f\n", point.intensity);      // 查看每个点的强度分布
        laserCloudScans[scanID].push_back(point);    // 将点云按照线束划分
    }
    
    // 去除了固定线数外的点
    cloudSize = count;
    // std::printf("points size %d \n", cloudSize);

    // 重新按照线束组织lidar点
    pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());
    for (int i = 0; i < N_SCANS; i++) { 
        scanStartInd[i] = laserCloud->size() + 5;   // 第i束点云在laserCloud中的起始下标
        *laserCloud += laserCloudScans[i];          // 按照线束顺序叠加的点云
        scanEndInd[i] = laserCloud->size() - 6;     // 第i束点云在laserCloud中的终止下标
    }

    std::printf("prepare time %f \n", t_prepare.toc());

    // 计算平滑度(曲率) 当前点的前后5个点的坐标差值
    for (int i = 5; i < cloudSize - 5; i++) { 
        float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x + laserCloud->points[i - 3].x + 
                      laserCloud->points[i - 2].x + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x + 
                      laserCloud->points[i + 1].x + laserCloud->points[i + 2].x + laserCloud->points[i + 3].x + 
                      laserCloud->points[i + 4].x + laserCloud->points[i + 5].x;
        float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y + laserCloud->points[i - 3].y + 
                      laserCloud->points[i - 2].y + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y + 
                      laserCloud->points[i + 1].y + laserCloud->points[i + 2].y + laserCloud->points[i + 3].y + 
                      laserCloud->points[i + 4].y + laserCloud->points[i + 5].y;
        float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z + laserCloud->points[i - 3].z + 
                      laserCloud->points[i - 2].z + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z + 
                      laserCloud->points[i + 1].z + laserCloud->points[i + 2].z + laserCloud->points[i + 3].z + 
                      laserCloud->points[i + 4].z + laserCloud->points[i + 5].z;

        cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;  // 曲率
        cloudSortInd[i] = i;
        cloudNeighborPicked[i] = 0;
        cloudLabel[i] = 0;
    }


    TicToc t_pts;

    pcl::PointCloud<PointType> cornerPointsSharp;       // 边缘特征点
    pcl::PointCloud<PointType> cornerPointsLessSharp;
    pcl::PointCloud<PointType> surfPointsFlat;          // 平面点
    pcl::PointCloud<PointType> surfPointsLessFlat;

    float t_q_sort = 0;
    for (int i = 0; i < N_SCANS; i++) {     // 分别对每个线束进行处理
        if ( scanEndInd[i] - scanStartInd[i] < 6) {
            continue;
        }
        pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<PointType>);
        for (int j = 0; j < 6; j++) {     // 将每个线束分为6段
            int sp = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * j / 6; 
            int ep = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * (j + 1) / 6 - 1;

            TicToc t_tmp;
            std::sort (cloudSortInd + sp, cloudSortInd + ep + 1, comp);
            t_q_sort += t_tmp.toc();

            // 找到曲率最大的点作为边缘点
            int largestPickedNum = 0;
            for (int k = ep; k >= sp; k--) {
                int ind = cloudSortInd[k]; 

                if (cloudNeighborPicked[ind] == 0 &&
                    cloudCurvature[ind] > 0.1) {

                    largestPickedNum++;
                    if (largestPickedNum <= 2) {                        
                        cloudLabel[ind] = 2;       // 选取2个边缘特征点，曲率最大的2个点的下标设置标签2
                        cornerPointsSharp.push_back(laserCloud->points[ind]);
                        cornerPointsLessSharp.push_back(laserCloud->points[ind]);
                    } else if (largestPickedNum <= 20) {                        
                        cloudLabel[ind] = 1;       // 曲率较大的点下标设置标签1
                        cornerPointsLessSharp.push_back(laserCloud->points[ind]);
                    } else {
                        break;
                    }

                    cloudNeighborPicked[ind] = 1;     // 被选中为Sharp/LessSharp的标记为是被选中的特征点

                    for (int l = 1; l <= 5; l++) {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {   // 将距离特征点较近的点设置标签防止被重复选择
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--) {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }

            // 寻找曲率最小的点作为平面点
            int smallestPickedNum = 0;
            for (int k = sp; k <= ep; k++) {
                int ind = cloudSortInd[k];

                if (cloudNeighborPicked[ind] == 0 &&
                    cloudCurvature[ind] < 0.1) {

                    cloudLabel[ind] = -1;      // 平面特征点下标设置为-1
                    surfPointsFlat.push_back(laserCloud->points[ind]);

                    smallestPickedNum++;
                    if (smallestPickedNum >= 4) {   // 选取4个平面特征点
                        break;
                    }

                    cloudNeighborPicked[ind] = 1;
                    for (int l = 1; l <= 5; l++) { 
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;    // 已被选为特征点的周围的点设置标签防止被重复选择
                    }
                    for (int l = -1; l >= -5; l--) {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }

            for (int k = sp; k <= ep; k++) {
                if (cloudLabel[k] <= 0) {     // 将所有曲率标签不为正数的点设置为LessFlat
                    surfPointsLessFlatScan->push_back(laserCloud->points[k]);
                }
            }
        }

        // 使用体素网格滤波来下采样点云
        pcl::PointCloud<PointType> surfPointsLessFlatScanDS;
        pcl::VoxelGrid<PointType> downSizeFilter;
        downSizeFilter.setInputCloud(surfPointsLessFlatScan);
        downSizeFilter.setLeafSize(0.2, 0.2, 0.2);   // 设置leaf size为0.2m
        downSizeFilter.filter(surfPointsLessFlatScanDS);

        surfPointsLessFlat += surfPointsLessFlatScanDS;
    }
    // std::printf("sort q time %f \n", t_q_sort);
    // std::printf("seperate points time %f \n", t_pts.toc());


    // 特征提取模块发布的点云时间戳和输入原始点云时间戳相同(而不是程序运行所处时刻)
    sensor_msgs::PointCloud2 laserCloudOutMsg;
    pcl::toROSMsg(*laserCloud, laserCloudOutMsg);
    laserCloudOutMsg.header.stamp = cloudHeader.stamp;
    laserCloudOutMsg.header.frame_id = "lidar";
    pubLaserCloud.publish(laserCloudOutMsg);          // 发布原始点云

    sensor_msgs::PointCloud2 cornerPointsSharpMsg;
    pcl::toROSMsg(cornerPointsSharp, cornerPointsSharpMsg);
    cornerPointsSharpMsg.header.stamp = cloudHeader.stamp;
    cornerPointsSharpMsg.header.frame_id = "lidar";
    pubCornerPointsSharp.publish(cornerPointsSharpMsg);  // 发布边缘特征点

    sensor_msgs::PointCloud2 cornerPointsLessSharpMsg;
    pcl::toROSMsg(cornerPointsLessSharp, cornerPointsLessSharpMsg);
    cornerPointsLessSharpMsg.header.stamp = cloudHeader.stamp;
    cornerPointsLessSharpMsg.header.frame_id = "lidar";
    pubCornerPointsLessSharp.publish(cornerPointsLessSharpMsg);   // 发布边缘lesssharp特征点

    sensor_msgs::PointCloud2 surfPointsFlat2;
    pcl::toROSMsg(surfPointsFlat, surfPointsFlat2);
    surfPointsFlat2.header.stamp = cloudHeader.stamp;
    surfPointsFlat2.header.frame_id = "lidar";
    pubSurfPointsFlat.publish(surfPointsFlat2);     // 发布平面特征点

    sensor_msgs::PointCloud2 surfPointsLessFlat2;
    pcl::toROSMsg(surfPointsLessFlat, surfPointsLessFlat2);
    surfPointsLessFlat2.header.stamp = cloudHeader.stamp;
    surfPointsLessFlat2.header.frame_id = "lidar";
    pubSurfPointsLessFlat.publish(surfPointsLessFlat2);    // 发布平面less flat特征点

    // pub each scam
    if(PUB_EACH_LINE)
    {
        for(int i = 0; i< N_SCANS; i++)
        {
            sensor_msgs::PointCloud2 scanMsg;
            pcl::toROSMsg(laserCloudScans[i], scanMsg);
            scanMsg.header.stamp = cloudHeader.stamp;
            scanMsg.header.frame_id = "lidar";
            pubEachScan[i].publish(scanMsg);
        }
    }

    std::printf("scan registration time %f ms *************\n", t_whole.toc());
    if(t_whole.toc() > 100)
        ROS_WARN("scan registration process over 100ms");
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "scanRegistration");
    ros::NodeHandle nh;
    std::string cloud_topic;

    nh.param<int>("scan_line", N_SCANS, 16);    // 参数写入N_SCANS 默认为16

    nh.param<double>("minimum_range", MINIMUM_RANGE, 0.1);
    nh.param<std::string>("/ascanRegistration/aloam_input_cloud_topic", cloud_topic, "/kitti/velo/pointcloud");

    std::printf("scan line number %d \n", N_SCANS);

    if(N_SCANS != 16 && N_SCANS != 32 && N_SCANS != 64) {
        std::printf("only support velodyne with 16, 32 or 64 scan line!");
        return 0;
    }

    // 订阅点云信息，并处理点云
    dbg(cloud_topic);
    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(cloud_topic, 100, laserCloudHandler);

    pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 100);    // 发布重新按线束叠加后的原始点云

    pubCornerPointsSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 100);

    pubCornerPointsLessSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 100);

    pubSurfPointsFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 100);

    pubSurfPointsLessFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 100);

    pubRemovePoints = nh.advertise<sensor_msgs::PointCloud2>("/laser_remove_points", 100);

    if(PUB_EACH_LINE) {
        for(int i = 0; i < N_SCANS; i++) {
            ros::Publisher tmp = nh.advertise<sensor_msgs::PointCloud2>("/laser_scanid_" + std::to_string(i), 100);
            pubEachScan.push_back(tmp);
        }
    }
    ros::spin();

    return 0;
}
