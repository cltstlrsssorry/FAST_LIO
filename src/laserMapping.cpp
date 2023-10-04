// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

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
#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <chrono>
#include <unistd.h>
#include <Python.h>
#include <so3_math.h>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include "IMU_Processing.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include "preprocess.h"
#include <ikd-Tree/ikd_Tree.h>

#define INIT_TIME           (0.1)
#define LASER_POINT_COV     (0.001)
#define MAXN                (720000)
#define PUBFRAME_PERIOD     (20)

/*** Time Log Variables ***/

//kdtree_incremental_time: 增量构建kd树的时间 kdtree_search_time: kd树搜索匹配的时间 kdtree_delete_time: 删除kd树的时间
double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;

//T1[MAXN]: 时间统计数组 s_plot[MAXN]: 各种时间数据的数组
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];

//match_time: 点云匹配的时间 solve_time: 求解的时间 solve_const_H_time: 求解边缘化hessian的时间
double match_time = 0, solve_time = 0, solve_const_H_time = 0;

//kdtree_size_st: kd树起始大小 kdtree_size_end: kd树结束大小 add_point_size: 增加的点的数量 kdtree_delete_counter: kd树删除的次数
int    kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;

//runtime_pos_log: 是否启用运行时位置日志pcd_save_en: 是否保存pcd文件 time_sync_en: 是否进行时间同步 extrinsic_est_en: 是否在线标定外参 path_en: 是否发布路径
bool   runtime_pos_log = false, pcd_save_en = false, time_sync_en = false, extrinsic_est_en = true, path_en = true;
/**************************/

float res_last[100000] = {0.0};//res_last[]: 上一次迭代的残差数组
float DET_RANGE = 300.0f;//DET_RANGE: 检测范围距离
const float MOV_THRESHOLD = 1.5f;//MOV_THRESHOLD: 运动检测阈值
double time_diff_lidar_to_imu = 0.0;//time_diff_lidar_to_imu: 激光雷达和IMU的时间偏移

//mtx_buffer/sig_buffer: 点云缓冲的线程同步变量
mutex mtx_buffer;
condition_variable sig_buffer;

//root_dir: 数据集根目录
string root_dir = ROOT_DIR;
//map_file_path: 地图保存路径 lid_topic/imu_topic: 传感器topic
string map_file_path, lid_topic, imu_topic;

//res_mean_last: 上次优化的平均残差
double res_mean_last = 0.05, total_residual = 0.0;

//last_timestamp_lidar/imu: 各传感器最后时间戳
double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;

//gyr_cov等: IMU的协方差矩阵
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;

//filter_size_xxx_min: 滤波参数 fov_deg: 激光视场角
double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;

//cube_len: 地图分块长度
double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;

//effct_feat_num: 有效特征点数 time_log_counter: 时间日志计数
int    effct_feat_num = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;
int    iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0, pcd_save_interval = -1, pcd_index = 0;
bool   point_selected_surf[100000] = {0};

//flg_exit:退出标志 flg_EKF_inited: EKF初始化标志
bool   lidar_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited;
//在C++中，bool类型的变量默认值是false。如果您没有为bool类型的变量赋值，那么它的值将默认为false

//scan_pub_en等: 发布控制标志
bool   scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;

//is_first_lidar: 第一帧标志
bool    is_first_lidar = true;

vector<vector<int>> pointSearchInd_surf;//pointSearchInd_surf: 搜索的点的索引
vector<BoxPointType> cub_needrm;//cub_needrm: 需要移除的cube
vector<PointVector> Nearest_Points;//Nearest_Points: 每个点的近邻点集
vector<double> extrinT(3, 0.0);//extrinT/R: 激光雷达到IMU的外参
vector<double> extrinR(9, 0.0);

//数据缓存
deque<double> time_buffer;//time_buffer: 时间缓存
deque<PointCloudXYZI::Ptr> lidar_buffer;//lidar_buffer: 激光点云缓存
deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer;//imu_buffer: IMU数据缓存

PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());//featsFromMap: 地图中特征点
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());//feats_undistort: 畸变校正后的特征点
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());//feats_down_body: 体坐标下采样特征点
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());//feats_down_world: 全局坐标下采样特征点
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));//normvec: 法向量
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));//laserCloudOri: 原始激光点云
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));//corr_normvect: 校正后的法向量
PointCloudXYZI::Ptr _featsArray;//_featsArray: 特征点提取用

pcl::VoxelGrid<PointType> downSizeFilterSurf;//downSizeFilterSurf: 下采样滤波器
pcl::VoxelGrid<PointType> downSizeFilterMap;

KD_TREE<PointType> ikdtree;//ikdtree: kd树对象

V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);//XAxisPoint_body: x轴在体坐标系下的表示
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);//XAxisPoint_world: x轴在世界坐标系下的表示
V3D euler_cur;//euler_cur: 当前姿态欧拉角
V3D position_last(Zero3d);//position_last: 上一帧位置
V3D Lidar_T_wrt_IMU(Zero3d);//Lidar_T_wrt_IMU: 激光雷达到IMU的位移
M3D Lidar_R_wrt_IMU(Eye3d);//Lidar_R_wrt_IMU: 激光雷达到IMU的旋转矩阵

/*** EKF inputs and output ***/

MeasureGroup Measures;//Measures: 存储每次测量数据的结构体对象,全局变量
esekfom::esekf<state_ikfom, 12, input_ikfom> kf;//kf: 卡尔曼滤波器对象
state_ikfom state_point;//state_point: 状态变量
vect3 pos_lid;//pos_lid: 激光雷达位置

//nav_msgs/Path是ROS（Robot Operating System，机器人操作系统）中一个重要的消息类型，用于表示路径。
//它在导航、路径规划和控制等领域中具有关键作用。nav_msgs/Path消息类型主要包含两个字段：header和poses。
nav_msgs::msg::Path path;
//nav_msgs/Odometry 消息保存了机器人空间里评估的位置和速度信息。 odomAftMapped: 映射后的里程消息
nav_msgs::msg::Odometry odomAftMapped;
//This represents an orientation in free space in quaternion form.  geoQuat: 圆周率,表示四元数的姿态
geometry_msgs::msg::Quaternion geoQuat;
//A Pose with reference coordinate frame and timestamp  msg_body_pose: 带时间戳的姿态消息
geometry_msgs::msg::PoseStamped msg_body_pose;


shared_ptr<Preprocess> p_pre(new Preprocess());//全局变量，调用预处理程序，获取预处理点云
shared_ptr<ImuProcess> p_imu(new ImuProcess());//全局变量，调用预处理程序，获取预处理IMU



/*** 线程 ***/
void SigHandle(int sig)
{
    flg_exit = true;//设置退出标志
    std::cout << "catch sig %d" << sig << std::endl;//捕获信号
    sig_buffer.notify_all();//唤醒所有等待的线程
    rclcpp::shutdown();//关闭ROS节点
}



/** 将LIO状态dump到日志文件中。
 *  dump_lio_state_to_log函数接受一个参数：日志文件的指针fp。
*/
inline void dump_lio_state_to_log(FILE *fp)
{
    V3D rot_ang(Log(state_point.rot.toRotationMatrix()));//V3D是一个表示三维向量的类，可能是一个自定义类型，用于表示点的坐标。

    //rot_ang是一个表示旋转角度的向量。
    //fprintf用于将状态信息写入日志文件。它接受一系列格式化字符串和参数，并将它们写入文件中。
    //函数将旋转角度、位置、速度和偏置等信息写入日志文件。
    fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));                            // Angle
    fprintf(fp, "%lf %lf %lf ", state_point.pos(0), state_point.pos(1), state_point.pos(2));    // Pos
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                                 // omega
    fprintf(fp, "%lf %lf %lf ", state_point.vel(0), state_point.vel(1), state_point.vel(2));    // Vel
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                                 // Acc
    fprintf(fp, "%lf %lf %lf ", state_point.bg(0), state_point.bg(1), state_point.bg(2));       // Bias_g
    fprintf(fp, "%lf %lf %lf ", state_point.ba(0), state_point.ba(1), state_point.ba(2));       // Bias_a
    fprintf(fp, "%lf %lf %lf ", state_point.grav[0], state_point.grav[1], state_point.grav[2]); // Bias_a
    fprintf(fp, "\r\n");

    //函数使用fflush刷新文件缓冲区，确保所有数据都已写入文件。
    fflush(fp);
}



/** 
 * 将点从局部坐标系转换为全局坐标系。 
 * pointBodyToWorld_ikfom函数接受三个参数：局部坐标系中的点pi、全局坐标系中的点po和state_ikfom对象s。
*/
void pointBodyToWorld_ikfom(PointType const *const pi, PointType *const po, state_ikfom &s)
{
    //V3D是一个表示三维向量的类，可能是一个自定义类型，用于表示点的坐标。
    //p_body是一个局部坐标系中的点，将其转换为全局坐标系中的点。
    V3D p_body(pi->x, pi->y, pi->z);

    //p_global是局部坐标系中的点在全局坐标系中的表示。它将局部坐标系中的点乘以旋转矩阵，然后将其加上偏移向量。
    V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);

    //函数将p_global的坐标值赋给po全局坐标系中的点。
    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);

    //函数将pi的强度赋给po。
    po->intensity = pi->intensity;
}



/**
 * 将点从局部坐标系转换为全局坐标系。
 * pointBodyToWorld函数接受两个参数：局部坐标系中的点pi和全局坐标系中的点po。
*/
void pointBodyToWorld(PointType const *const pi, PointType *const po)
{
    //V3D是一个表示三维向量的类，可能是一个自定义类型，用于表示点的坐标。
    //p_body是一个局部坐标系中的点，将其转换为全局坐标系中的点。
    V3D p_body(pi->x, pi->y, pi->z);

    //p_global是局部坐标系中的点在全局坐标系中的表示。它将局部坐标系中的点乘以旋转矩阵，然后将其加上偏移向量。
    V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);

    //函数将p_global的坐标值赋给po全局坐标系中的点。
    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);

    //函数将pi的强度赋给po。
    po->intensity = pi->intensity;
}



/**
 * pointBodyToWorld函数是一个模板函数，接受两个参数：局部坐标系中的点pi和全局坐标系中的点po。
 */
template <typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}



/**
 * RGBpointBodyToWorld函数接受两个参数：局部坐标系中的点pi和全局坐标系中的点po。
 */
void RGBpointBodyToWorld(PointType const *const pi, PointType *const po)
{
    V3D p_body(pi->x, pi->y, pi->z);

    //p_global是局部坐标系中的点在全局坐标系中的表示。它将局部坐标系中的点乘以旋转矩阵，然后将其加上偏移向量。
    V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}



/**
 * 点从雷达坐标系转换为IMU坐标系。
 * RGBpointBodyLidarToIMU函数接受两个参数：雷达坐标系中的点pi和IMU坐标系中的点po。
 */
void RGBpointBodyLidarToIMU(PointType const *const pi, PointType *const po)
{
    //p_body_lidar是一个雷达坐标系中的点，将其转换为IMU坐标系中的点。
    V3D p_body_lidar(pi->x, pi->y, pi->z);

    //p_body_imu是雷达坐标系中的点在IMU坐标系中的表示。它将雷达坐标系中的点乘以偏移矩阵。
    V3D p_body_imu(state_point.offset_R_L_I * p_body_lidar + state_point.offset_T_L_I);

    //函数将p_body_imu的坐标值赋给po IMU坐标系中的点。
    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);

    //函数将pi的强度赋给po。
    po->intensity = pi->intensity;
}



/**
 * 收集IKDTree中删除的点。
 * points_cache_collect函数定义了一个名为points_history的点向量，用于存储从IKDTree中删除的点。
 */
void points_cache_collect()
{
    PointVector points_history;

    //ikdtree.acquire_removed_points(points_history)将IKDTree中删除的点存储在points_history向量中。
    ikdtree.acquire_removed_points(points_history);

    //函数没有对删除的点执行任何操作，只是将删除的点存储在points_history向量中。
    // for (int i = 0; i < points_history.size(); i++) _featsArray->push_back(points_history[i]);
}



/**
 * 处理激光雷达地图的视野范围，并在需要时更新局部地图的顶点。
 * 
*/
BoxPointType LocalMap_Points;//定义一个局部地图点类型LocalMap_Points
bool Localmap_Initialized = false;//布尔变量Localmap_Initialized，用于判断局部地图是否已初始化。
void lasermap_fov_segment()
{
    cub_needrm.clear();//清除cub_needrm向量，以便在后续步骤中存储需要删除的点。

    //设置kdtree_delete_counter和kdtree_delete_time的初始值。
    kdtree_delete_counter = 0;
    kdtree_delete_time = 0.0;

    //将XAxisPoint_body和XAxisPoint_world之间的点体转换为世界坐标系。
    pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);

    //计算激光雷达的位置pos_LiD，并将其与局部地图的顶点进行比较。
    V3D pos_LiD = pos_lid;

    //如果局部地图尚未初始化，则根据激光雷达的位置初始化局部地图的顶点。
    if (!Localmap_Initialized)
    {
        for (int i = 0; i < 3; i++)
        {
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }

    //更新局部地图的顶点。
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++)
    {
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
            need_move = true;
    }
    if (!need_move)
        return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD - 1)));
    for (int i = 0; i < 3; i++)
    {
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE)
        {
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
        else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
        {
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();
    double delete_begin = omp_get_wtime();
    if (cub_needrm.size() > 0)
        kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
    kdtree_delete_time = omp_get_wtime() - delete_begin;
}



//订阅standard_pcl_cbk() 
void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::UniquePtr msg)
{
    cout<<"------standard_pcl_cbk------"<<endl;
    mtx_buffer.lock();//调用该函数的线程尝试加锁。如果上锁不成功，即：其它线程已经上锁且未释放，则当前线程block。如果上锁成功，则执行后面的操作，操作完成后要调用mtx.unlock()释放锁，否则会导致死锁的产生
    scan_count++;
    double cur_time = get_time_sec(msg->header.stamp);
    double preprocess_start_time = omp_get_wtime();//可以理解为当前时间戳
    if (!is_first_lidar && cur_time < last_timestamp_lidar)//检测激光时间戳是否异常
    {
        std::cerr << "lidar loop back, clear buffer" << std::endl;
        lidar_buffer.clear();
    }
    if (is_first_lidar)
    {
        is_first_lidar = false;
    }

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);        // 激光雷达预处理，获得特征点云
    lidar_buffer.push_back(ptr);     // 激光雷达预处理完的雷达数据
    time_buffer.push_back(cur_time); // time_buffer是以激光雷达时间戳为基准的时间戳队列
    last_timestamp_lidar = cur_time;
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time; // 用于绘图显示处理时间
    mtx_buffer.unlock();                                            // 释放锁
    sig_buffer.notify_all();                                        // 信号量的提示 唤醒线程
}



//这两个变量配合使用,可以实现激光雷达数据和 IMU 数据之间的时间同步。timediff_lidar_wrt_imu 存储了两者的时间差,timediff_set_flg 表示这时间差是否已经计算好了。通过这种方式,在不依赖外部硬件时间同步的情况下,可以完成激光雷达和 IMU 的软件时间同步,是典型的时间同步方式。
double timediff_lidar_wrt_imu = 0.0;//timediff_lidar_wrt_imu 是一个 double 类型的变量,用于存储激光雷达数据相对于 IMU 的时间差。timediff_lidar_wrt_imu 被初始化为 0.0,表示激光雷达和 IMU 的时间差值。在时间同步过程中,会根据两者的时间戳计算出这个时间差。
bool timediff_set_flg = false;//timediff_set_flg 是一个 bool 类型的变量,作为一个标志,表示时间差是否已经设置过了。timediff_set_flg 被初始化为 false,表示时间差还没有被设置过。在代码逻辑中,会在时间差计算完成后,将这个标志置为 true。
void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::UniquePtr msg)
{
    cout<<"------livox_pcl_cbk------"<<endl;
    mtx_buffer.lock();//调用该函数的线程尝试加锁。如果上锁不成功，即：其它线程已经上锁且未释放，则当前线程block。如果上锁成功，则执行后面的操作，操作完成后要调用mtx.unlock()释放锁，否则会导致死锁的产生
    double cur_time = get_time_sec(msg->header.stamp);//获取信息时间戳时间
    double preprocess_start_time = omp_get_wtime();//获取程序执行当前时间
    scan_count++;//全局变量，初始值为零。

    //处理激光数据回环，last_timestamp_lidar是上一个时间戳。
    if (!is_first_lidar && cur_time < last_timestamp_lidar)
    {
        std::cerr << "lidar loop back, clear buffer" << std::endl;//检测并处理激光数据回环
        lidar_buffer.clear();//清空雷达缓存
    }

    //处理第一帧数据的标志位
    if (is_first_lidar)
    {
        is_first_lidar = false;
    }

    last_timestamp_lidar = cur_time;//将当前时间戳赋值给上一个时间戳

    //判断time_sync_en是否为false,即没有外部时间同步模块。
    //如果时间戳差值大于10ms,并且IMU和激光缓存都不为空,则认为传感器时间同步存在问题。
    if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty())
    {
        //超过10ms的时间差,表明IMU和激光的时间脉冲不对齐,会导致后续的时间同步出现问题。
        //打印提示未对齐的时间戳,其中last_timestamp_imu是最近一次IMU的时间戳,last_timestamp_lidar是最近一次激光的时间戳。
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n", last_timestamp_imu, last_timestamp_lidar);
    }
    //这段代码的目的是在没有外部硬件时间同步的情况下,通过时间戳检测IMU和激光是否存在严重的时间偏差,以提示用户注意时间同步问题。


    //判断time_sync_en为true,表示需要自主时间同步。
    //timediff_set_flg未设置,表示还未计算时间差。
    //当前帧激光和上一帧IMU时间戳差值的绝对值大于1ms。
    //IMU缓存不为空,确保有IMU数据。
    //如果以上条件都满足,进行时间差计算:
    //      将timediff_set_flg设置为true,表示时间差已计算。
    //      计算时间差:当前帧激光时间戳+0.1ms减去上一帧IMU时间戳。
    //      打印Info级别日志,输出计算的时间差结果。
    if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
    {
        timediff_set_flg = true;
        timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
    }
    //可以自主地计算出IMU和激光之间的时间偏差,进行时间同步。需要注意的是时间差计算只进行一次。


    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());//创建一个PointCloudXYZI类型的智能指针ptr,用于存储预处理后的点云。
    p_pre->process(msg, ptr);//调用预处理函数p_pre->process,输入是原始的激光消息msg,输出是ptr。
    lidar_buffer.push_back(ptr);//将预处理后的点云ptr放入lidar_buffer队列中。
    time_buffer.push_back(last_timestamp_lidar);//将当前激光帧的时间戳last_timestamp_lidar放入time_buffer队列。时间缓存 deque<double>
    //后续的里程计主函数会去lidar_buffer和time_buffer中获取预处理后的点云和时间戳。

    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;//计算预处理耗时,用于统计。
    mtx_buffer.unlock();//解锁互斥锁mtx_buffer。
    sig_buffer.notify_all();//通知condition_variable sig_buffer。

    //mtx_buffer/sig_buffer用于线程同步和互斥访问buffer。
}



/**
 * 用于处理从IMU获取的数据，并将其插入到队列中。同时，它还会对时间戳进行校验，以检测跳变。
*/
void imu_cbk(const sensor_msgs::msg::Imu::UniquePtr msg_in)
{
    cout<<"------imu_cbk------"<<endl;
    publish_count++;
    // cout<<"IMU got at: "<<msg_in->header.stamp.toSec()<<endl;
    sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));

    msg->header.stamp = get_ros_time(get_time_sec(msg_in->header.stamp) - time_diff_lidar_to_imu);
    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)//timediff_lidar_wrt_imu仅在使用lovix雷达时才会使用
    {
        msg->header.stamp =
            rclcpp::Time(timediff_lidar_wrt_imu + get_time_sec(msg_in->header.stamp));
    }

    double timestamp = get_time_sec(msg->header.stamp);//经过补偿的IMU时间戳，如果是lovix雷达才需要补偿，其他不需要

    mtx_buffer.lock();

    if (timestamp < last_timestamp_imu)//校验IMU时间戳的一维性，检测跳变
    {
        std::cerr << "lidar loop back, clear buffer" << std::endl;
        imu_buffer.clear();
    }

    last_timestamp_imu = timestamp;//最新IMU的时间

    imu_buffer.push_back(msg);//数据插入队列中
    mtx_buffer.unlock();
    sig_buffer.notify_all();//有信号时，唤醒线程
}


double lidar_mean_scantime = 0.0;
int    scan_num = 0;

//这部分主要处理了buffer中的数据，将两帧激光雷达点云数据时间内的IMU数据从缓存队列中取出，进行时间对齐，并保存到meas中
bool sync_packages(MeasureGroup &meas)
{
    cout<<"------sync_packages------"<<endl;

    if (lidar_buffer.empty() || imu_buffer.empty())
    {
        return false;
    }

    /*** push a lidar scan ***/
    if (!lidar_pushed) // 如果程序初始化时没指定，默认值是false， 是否已经将测量值插入雷达帧数据,判断lidar_pushed标志是否为false,即还未放入过激光数据,初始化为false
    {
        meas.lidar = lidar_buffer.front(); // 将雷达队列最前面的数据塞入测量值

        meas.lidar_beg_time = time_buffer.front(); // 雷达的时间按照time_buffer队首处理，因为它存的就是雷达的时间戳

        if (meas.lidar->points.size() <= 1) // time too little //保证塞入的雷达数据点都是有效的，判断点云大小,如果太小则丢弃这帧
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime; // 计算结束时间
            std::cerr << "Too few input point cloud!\n";
        }

        // meas.lidar->points.back(): 最后一个点的曲率值是其时间戳,单位是微秒
        else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime) // 雷达帧头的时间戳是帧头的时间戳，这和驱动有关系，通过公式推导该帧激光的帧尾时间戳，根据点云中最后一个点的曲率值计算得到该帧的结束时间戳lidar_end_time
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
        }
        else
        {
            scan_num++;
            lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);                    // 根据点云中最后一个点的曲率值计算得到该帧的结束时间戳lidar_end_time
            lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num; // 更新平均扫描时间lidar_mean_scantime
        }
        // 最后一个点的曲率值是其时间戳,单位是微秒

        meas.lidar_end_time = lidar_end_time; // 将计算得到的结束时间戳赋值给meas.lidar_end_time

        lidar_pushed = true; // 将lidar_pushed标志置为true,表示已提取到lidar数据
    }

    //如果最新的IMU时间戳都闭雷达帧尾的时间早，则这一帧不处理了
    if (last_timestamp_imu < lidar_end_time)
    {
        cout<<"last_timestamp_imu: "<<last_timestamp_imu<<"\t"<<"lidar_end_time: "<<lidar_end_time<<endl;
        return false;
    }

    /*** push imu data, and pop from imu buffer ***/
    double imu_time = get_time_sec(imu_buffer.front()->header.stamp);//从最早的IMU队列开始，初始化imu_time
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
    {
        imu_time = get_time_sec(imu_buffer.front()->header.stamp);
        if (imu_time > lidar_end_time)
            break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    lidar_buffer.pop_front();//处理过的数据出栈
    time_buffer.pop_front();
    lidar_pushed = false;//又重新置位，这样下一帧雷达来了又可以刷新时间，获取点云帧头和帧尾的时间
    return true;
}



/** 
 * 处理点云数据，将其添加到地图中。
*/
//定义一个整数变量process_increments，表示已经处理过的增量点云的数量。
int process_increments = 0;
void map_incremental()
{
    //创建两个点向量PointToAdd和PointNoNeedDownsample，并将其容量设置为feats_down_size。
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);

    /**
     * 遍历feats_down_size个点，对每个点进行以下操作：
     *      a. 将点从局部坐标系转换为世界坐标系。
     *      b. 判断该点是否需要添加到地图中。如果需要添加，则执行以下操作：
     *          i. 计算点与最近邻点的距离。
     *          ii. 如果最近邻点与点之间的距离大于0.5倍滤波器大小，则不添加该点。
     *          iii. 遍历最近邻点，检查与点之间的距离是否小于计算出的距离。如果是，则不添加该点。
     *          iv. 如果满足条件，将该点添加到PointToAdd向量中。
     *      c. 如果不需要添加，则将该点添加到PointNoNeedDownsample向量中。
    */
    for (int i = 0; i < feats_down_size; i++)
    {
        /* transform to world frame */
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
        /* decide if need add to map */
        if (!Nearest_Points[i].empty() && flg_EKF_inited)
        {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            PointType downsample_result, mid_point;
            mid_point.x = floor(feats_down_world->points[i].x / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
            float dist = calc_dist(feats_down_world->points[i], mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min)
            {
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                continue;
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i++)
            {
                if (points_near.size() < NUM_MATCH_POINTS)
                    break;
                if (calc_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add)
                PointToAdd.push_back(feats_down_world->points[i]);
        }
        else
        {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }

    //计算添加点的时间。
    double st_time = omp_get_wtime();

    //将PointToAdd向量中的点添加到IKDTree中，并将它们标记为需要添加到地图中。
    add_point_size = ikdtree.Add_Points(PointToAdd, true);

    //将PointNoNeedDownsample向量中的点添加到IKDTree中，并将它们标记为不需要添加到地图中。
    ikdtree.Add_Points(PointNoNeedDownsample, false);

    //计算添加点的大小。
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();

    //计算添加点的时间。
    kdtree_incremental_time = omp_get_wtime() - st_time;
}



//创建一个PointCloudXYZI::Ptr类型的变量pcl_wait_pub，用于存储待发布的点云数据。
PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI());
//创建一个PointCloudXYZI::Ptr类型的变量pcl_wait_save，用于存储待发布的点云数据。
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());



/**
 * 定义一个名为publish_frame_world的函数，该函数接受一个rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr类型的参数pubLaserCloudFull。
*/
void publish_frame_world(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull)
{
    /**
     * 如果发布点云数据的功能已启用，则执行以下操作：
     *      a. 根据是否使用滤波器，将原始点云数据转换为laserCloudFullRes。
     *      b. 创建一个PointCloudXYZI::Ptr类型的变量laserCloudWorld，用于存储转换后的世界坐标系点云数据。
     *      c. 遍历laserCloudFullRes中的每个点，将其从局部坐标系转换为世界坐标系。
     *      d. 将转换后的点云数据存储在laserCloudWorld中。
     *      e. 将laserCloudWorld转换为sensor_msgs::msg::PointCloud2类型的消息。
     *      f. 创建一个sensor_msgs::msg::PointCloud2类型的变量laserCloudmsg。
     *      g. 将laserCloudWorld的属性设置为laserCloudmsg，包括时间戳、帧ID和发布者名称。
     *      h. 发布laserCloudmsg类型的点云数据。
     *      i. 减少发布计数器。
    */
    if (scan_pub_en)
    {
        //根据是否使用滤波器，将原始点云数据转换为laserCloudFullRes。
        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
        int size = laserCloudFullRes->points.size();

        //创建一个PointCloudXYZI::Ptr类型的变量laserCloudWorld，用于存储转换后的世界坐标系点云数据。
        PointCloudXYZI::Ptr laserCloudWorld(
            new PointCloudXYZI(size, 1));
        
        //遍历laserCloudFullRes中的每个点，将其从局部坐标系转换为世界坐标系。
        for (int i = 0; i < size; i++)
        {
            //将转换后的点云数据存储在laserCloudWorld中。
            RGBpointBodyToWorld(&laserCloudFullRes->points[i],
                                &laserCloudWorld->points[i]);
        }

        //创建一个sensor_msgs::msg::PointCloud2类型的变量laserCloudmsg。
        sensor_msgs::msg::PointCloud2 laserCloudmsg;

        ////将laserCloudWorld转换为sensor_msgs::msg::PointCloud2类型的消息。
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);

        //将laserCloudWorld的属性设置为laserCloudmsg，包括时间戳、帧ID和发布者名称。
        // laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
        laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
        laserCloudmsg.header.frame_id = "camera_init";

        //发布laserCloudmsg类型的点云数据。
        pubLaserCloudFull->publish(laserCloudmsg);

        //减少发布计数器。
        publish_count -= PUBFRAME_PERIOD;
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
    /*
    if (pcd_save_en)
    {
        int size = feats_undistort->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&feats_undistort->points[i], \
                                &laserCloudWorld->points[i]);
        }
        *pcl_wait_save += *laserCloudWorld;

        static int scan_wait_num = 0;
        scan_wait_num ++;
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0  && scan_wait_num >= pcd_save_interval)
        {
            pcd_index ++;
            string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_") + to_string(pcd_index) + string(".pcd"));
            pcl::PCDWriter pcd_writer;
            cout << "current scan saved to /PCD/" << all_points_dir << endl;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
        }
    }
    */
}



/**
 * 发布点云数据
 * 创建一个PointCloudXYZI::Ptr类型的变量pcl_wait_pub，用于存储待发布的点云数据。
 * 定义一个名为publish_frame_body的函数，该函数接受一个rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr类型的参数pubLaserCloudFull_body。
*/
void publish_frame_body(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_body)
{
    int size = feats_undistort->points.size();

    //创建一个PointCloudXYZI::Ptr类型的变量laserCloudIMUBody，用于存储转换后的身体坐标系点云数据。
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    //遍历feats_undistort中的每个点，将其从雷达坐标系转换为身体坐标系。
    for (int i = 0; i < size; i++)
    {
        //将转换后的点云数据存储在laserCloudIMUBody中。
        RGBpointBodyLidarToIMU(&feats_undistort->points[i],
                               &laserCloudIMUBody->points[i]);
    }

    //创建一个sensor_msgs::msg::PointCloud2类型的变量laserCloudmsg。
    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    ////将laserCloudIMUBody转换为sensor_msgs::msg::PointCloud2类型的消息。
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);

    //将laserCloudIMUBody的属性设置为laserCloudmsg，包括时间戳、帧ID和发布者名称。
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = "body";

    //发布laserCloudmsg类型的点云数据。
    pubLaserCloudFull_body->publish(laserCloudmsg);

    //减少发布计数器。
    publish_count -= PUBFRAME_PERIOD;
}



//发布effect point
void publish_effect_world(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect)
{
    PointCloudXYZI::Ptr laserCloudWorld(
        new PointCloudXYZI(effct_feat_num, 1));
    for (int i = 0; i < effct_feat_num; i++)
    {
        RGBpointBodyToWorld(&laserCloudOri->points[i],
                            &laserCloudWorld->points[i]);
    }
    sensor_msgs::msg::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = get_ros_time(lidar_end_time);
    laserCloudFullRes3.header.frame_id = "camera_init";
    pubLaserCloudEffect->publish(laserCloudFullRes3);
}



//发布map
void publish_map(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap)
{
    PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
    int size = laserCloudFullRes->points.size();
    PointCloudXYZI::Ptr laserCloudWorld(
        new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        RGBpointBodyToWorld(&laserCloudFullRes->points[i],
                            &laserCloudWorld->points[i]);
    }
    *pcl_wait_pub += *laserCloudWorld;

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*pcl_wait_pub, laserCloudmsg);
    // laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = "camera_init";
    pubLaserCloudMap->publish(laserCloudmsg);

    // sensor_msgs::msg::PointCloud2 laserCloudMap;
    // pcl::toROSMsg(*featsFromMap, laserCloudMap);
    // laserCloudMap.header.stamp = get_ros_time(lidar_end_time);
    // laserCloudMap.header.frame_id = "camera_init";
    // pubLaserCloudMap->publish(laserCloudMap);
}



void save_to_pcd()
{
    pcl::PCDWriter pcd_writer;
    pcd_writer.writeBinary(map_file_path, *pcl_wait_pub);
}



template<typename T>
void set_posestamp(T & out)
{
    out.pose.position.x = state_point.pos(0);
    out.pose.position.y = state_point.pos(1);
    out.pose.position.z = state_point.pos(2);
    out.pose.orientation.x = geoQuat.x;
    out.pose.orientation.y = geoQuat.y;
    out.pose.orientation.z = geoQuat.z;
    out.pose.orientation.w = geoQuat.w;
    
}



/**
 * 定义一个函数，用于发布里程计信息
 * 参数 pubOdomAftMapped 是一个指向 Odometry 类型消息发布者的共享指针
 * 参数 tf_br 是一个指向转换广播器的唯一指针
*/
void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped, std::unique_ptr<tf2_ros::TransformBroadcaster> &tf_br)
{

    // 设置里程计消息的头部信息
    // frame_id 表示参考坐标系的名称，这里是 "camera_init"
    // child_frame_id 表示机器人坐标系的名称，这里是 "body"
    // stamp 表示消息的时间戳，这里使用 lidar_end_time 作为参数调用 get_ros_time 函数获取
    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "body";
    odomAftMapped.header.stamp = get_ros_time(lidar_end_time);

    // 调用 set_posestamp 函数，设置里程计消息中的位置和方向信息
    set_posestamp(odomAftMapped.pose);

    // 使用 pubOdomAftMapped 发布者向名为 "odom" 的主题发送里程计消息
    pubOdomAftMapped->publish(odomAftMapped);

    // 调用 kf 对象的 get_P 方法，获取卡尔曼滤波器中的协方差矩阵 P
    auto P = kf.get_P();

    // 使用一个 for 循环，遍历 P 矩阵中的元素，并将其赋值给里程计消息中的协方差数组
    for (int i = 0; i < 6; i++)
    {
        // 定义一个变量 k ，根据 i 的值进行调整
        int k = i < 3 ? i + 3 : i - 3;

        // 将 P 矩阵中第 k 行第 3 列的元素赋值给里程计消息中第 i 行第 0 列的元素
        odomAftMapped.pose.covariance[i * 6 + 0] = P(k, 3);

        // 将 P 矩阵中第 k 行第 4 列的元素赋值给里程计消息中第 i 行第 1 列的元素
        odomAftMapped.pose.covariance[i * 6 + 1] = P(k, 4);

        // 将 P 矩阵中第 k 行第 5 列的元素赋值给里程计消息中第 i 行第 2 列的元素
        odomAftMapped.pose.covariance[i * 6 + 2] = P(k, 5);

        // 将 P 矩阵中第 k 行第 0 列的元素赋值给里程计消息中第 i 行第 3 列的元素
        odomAftMapped.pose.covariance[i * 6 + 3] = P(k, 0);

        // 将 P 矩阵中第 k 行第 1 列的元素赋值给里程计消息中第 i 行第 4 列的元素
        odomAftMapped.pose.covariance[i * 6 + 4] = P(k, 1);

        // 将 P 矩阵中第 k 行第 2 列的元素赋值给里程计消息中第 i 行第 5 列的元素
        odomAftMapped.pose.covariance[i * 6 + 5] = P(k, 2);
    }

    // 定义一个 TransformStamped 类型的变量 trans，用于存储转换信息
    geometry_msgs::msg::TransformStamped trans;

    // 设置转换信息的头部信息
    // frame_id 表示参考坐标系的名称，这里是 "camera_init"
    // child_frame_id 表示机器人坐标系的名称，这里是 "body"
    trans.header.frame_id = "camera_init";
    trans.child_frame_id = "body";

    // 设置转换信息中的平移部分，使用里程计消息中的位置信息
    trans.transform.translation.x = odomAftMapped.pose.pose.position.x;
    trans.transform.translation.y = odomAftMapped.pose.pose.position.y;
    trans.transform.translation.z = odomAftMapped.pose.pose.position.z;

    // 设置转换信息中的旋转部分，使用里程计消息中的方向信息
    trans.transform.rotation.w = odomAftMapped.pose.pose.orientation.w;
    trans.transform.rotation.x = odomAftMapped.pose.pose.orientation.x;
    trans.transform.rotation.y = odomAftMapped.pose.pose.orientation.y;
    trans.transform.rotation.z = odomAftMapped.pose.pose.orientation.z;

    // 使用 tf_br 转换广播器向名为 "tf" 的主题发送转换信息
    tf_br->sendTransform(trans);
}



/**
 * nav_msgs/Path
 * nav_msgs/Path 消息用于描述一条路径信息。可以多设置几个坐标点，小车就会依次经过这些点。下面是消息格式
 * 其中包含了header和poses两个部分：
 *      header：这个消息的头部信息，包括序列号seq、时间戳stamp和参考坐标系frame_id。
 *      poses：一组路径点位姿信息，包含多个header和pose；每一个位姿包括一个header和一个pose，其中pose是位姿信息，包括位置和方向。
*/
void publish_path(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath)
{
    // 设置姿态戳
    set_posestamp(msg_body_pose);


    // 设置头戳
    msg_body_pose.header.stamp = get_ros_time(lidar_end_time); // ros::Time().fromSec(lidar_end_time);
    
    // 设置头框架ID
    msg_body_pose.header.frame_id = "camera_init";

    /*** 如果路径太大，rvis将崩溃 ***/
    /*** if path is too large, the rvis will crash ***/
    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0)
    {
        // 将姿态添加到路径中
        path.poses.push_back(msg_body_pose);
        zhege
        // 发布路径
        pubPath->publish(path);
    }
}


/**
 * h_share_model是用于处理系统状态和测量噪声的函数。
 * esekfom::dyn_share_datastruct<double> 是一个包含测量和系统噪声信息的结构。函数的主要目的是计算测量值的雅可比矩阵 H 和测量值向量 h，并将它们存储在 ekfom_data 对象中。
*/
void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    double match_start = omp_get_wtime();
    laserCloudOri->clear();
    corr_normvect->clear();
    total_residual = 0.0;

/** closest surface search and residual computation **/
#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
    for (int i = 0; i < feats_down_size; i++)
    {
        PointType &point_body = feats_down_body->points[i];
        PointType &point_world = feats_down_world->points[i];

        /* transform to world frame */
        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

        auto &points_near = Nearest_Points[i];

        if (ekfom_data.converge)
        {
            /** Find the closest surfaces in the map **/
            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false
                                                                                                                                : true;
        }

        if (!point_selected_surf[i])
            continue;

        VF(4)
        pabcd;
        point_selected_surf[i] = false;
        if (esti_plane(pabcd, points_near, 0.1f))
        {
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

            if (s > 0.9)
            {
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2;
                res_last[i] = abs(pd2);
            }
        }
    }

    effct_feat_num = 0;

    for (int i = 0; i < feats_down_size; i++)
    {
        if (point_selected_surf[i])
        {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
            corr_normvect->points[effct_feat_num] = normvec->points[i];
            total_residual += res_last[i];
            effct_feat_num++;
        }
    }

    if (effct_feat_num < 1)
    {
        ekfom_data.valid = false;
        std::cerr << "No Effective Points!" << std::endl;
        // ROS_WARN("No Effective Points! \n");
        return;
    }

    res_mean_last = total_residual / effct_feat_num;
    match_time += omp_get_wtime() - match_start;
    double solve_start_ = omp_get_wtime();

    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12); // 23
    ekfom_data.h.resize(effct_feat_num);

    for (int i = 0; i < effct_feat_num; i++)
    {
        const PointType &laser_p = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat << SKEW_SYM_MATRX(point_this);

        /*** get the normal vector of closest surface/corner ***/
        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() * norm_vec);
        V3D A(point_crossmat * C);
        if (extrinsic_est_en)
        {
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); // s.rot.conjugate()*norm_vec);
            ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        }
        else
        {
            ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        /*** Measuremnt: distance to the closest surface/corner ***/
        ekfom_data.h(i) = -norm_p.intensity;
    }
    solve_time += omp_get_wtime() - solve_start_;
}



//激光雷达建图节点
class LaserMappingNode : public rclcpp::Node
{

public:

    //结构体初始化
    LaserMappingNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : Node("laser_mapping", options)
    {
        cout<<"----LaserMappingNode----"<<endl;
        //声明和初始化一个参数
        this->declare_parameter<bool>("publish.path_en", true);//
        this->declare_parameter<bool>("publish.effect_map_en", false);//
        this->declare_parameter<bool>("publish.map_en", false);
        this->declare_parameter<bool>("publish.scan_publish_en", true);
        this->declare_parameter<bool>("publish.dense_publish_en", true);
        this->declare_parameter<bool>("publish.scan_bodyframe_pub_en", true);
        this->declare_parameter<int>("max_iteration", 4);
        this->declare_parameter<string>("map_file_path", "");
        this->declare_parameter<string>("common.lid_topic", "/livox/lidar");
        this->declare_parameter<string>("common.imu_topic", "/livox/imu");
        this->declare_parameter<bool>("common.time_sync_en", false);
        this->declare_parameter<double>("common.time_offset_lidar_to_imu", 0.0);
        this->declare_parameter<double>("filter_size_corner", 0.5);
        this->declare_parameter<double>("filter_size_surf", 0.5);
        this->declare_parameter<double>("filter_size_map", 0.5);
        this->declare_parameter<double>("cube_side_length", 200.);
        this->declare_parameter<float>("mapping.det_range", 300.);
        this->declare_parameter<double>("mapping.fov_degree", 180.);
        this->declare_parameter<double>("mapping.gyr_cov", 0.1);
        this->declare_parameter<double>("mapping.acc_cov", 0.1);
        this->declare_parameter<double>("mapping.b_gyr_cov", 0.0001);
        this->declare_parameter<double>("mapping.b_acc_cov", 0.0001);
        this->declare_parameter<double>("preprocess.blind", 0.01);
        this->declare_parameter<int>("preprocess.lidar_type", AVIA);
        this->declare_parameter<int>("preprocess.scan_line", 16);
        this->declare_parameter<int>("preprocess.timestamp_unit", US);
        this->declare_parameter<int>("preprocess.scan_rate", 10);
        this->declare_parameter<int>("point_filter_num", 2);
        this->declare_parameter<bool>("feature_extract_enable", false);
        this->declare_parameter<bool>("runtime_pos_log_enable", false);
        this->declare_parameter<bool>("mapping.extrinsic_est_en", true);
        this->declare_parameter<bool>("pcd_save.pcd_save_en", false);
        this->declare_parameter<int>("pcd_save.interval", -1);
        this->declare_parameter<vector<double>>("mapping.extrinsic_T", vector<double>());
        this->declare_parameter<vector<double>>("mapping.extrinsic_R", vector<double>());

        //此命令将配置文件加载到ROS2参数服务器中。然后，您可以使用以下命令从参数服务器中获取参数的值：
        // Get the parameter value, or the "alternative value" if not set, and assign it to "value".
        // If the parameter was not set, then the "value" argument is assigned the "alternative_value". In all cases, the parameter remains not set after this function is called.
        this->get_parameter_or<bool>("publish.path_en", path_en, true);
        this->get_parameter_or<bool>("publish.effect_map_en", effect_pub_en, false);
        this->get_parameter_or<bool>("publish.map_en", map_pub_en, false);
        this->get_parameter_or<bool>("publish.scan_publish_en", scan_pub_en, true);
        this->get_parameter_or<bool>("publish.dense_publish_en", dense_pub_en, true);
        this->get_parameter_or<bool>("publish.scan_bodyframe_pub_en", scan_body_pub_en, true);
        this->get_parameter_or<int>("max_iteration", NUM_MAX_ITERATIONS, 4);
        this->get_parameter_or<string>("map_file_path", map_file_path, "");
        this->get_parameter_or<string>("common.lid_topic", lid_topic, "/livox/lidar");
        this->get_parameter_or<string>("common.imu_topic", imu_topic, "/livox/imu");
        this->get_parameter_or<bool>("common.time_sync_en", time_sync_en, false);
        this->get_parameter_or<double>("common.time_offset_lidar_to_imu", time_diff_lidar_to_imu, 0.0);
        this->get_parameter_or<double>("filter_size_corner", filter_size_corner_min, 0.5);
        this->get_parameter_or<double>("filter_size_surf", filter_size_surf_min, 0.5);
        this->get_parameter_or<double>("filter_size_map", filter_size_map_min, 0.5);
        this->get_parameter_or<double>("cube_side_length", cube_len, 200.f);
        this->get_parameter_or<float>("mapping.det_range", DET_RANGE, 300.f);
        this->get_parameter_or<double>("mapping.fov_degree", fov_deg, 180.f);
        this->get_parameter_or<double>("mapping.gyr_cov", gyr_cov, 0.1);
        this->get_parameter_or<double>("mapping.acc_cov", acc_cov, 0.1);
        this->get_parameter_or<double>("mapping.b_gyr_cov", b_gyr_cov, 0.0001);
        this->get_parameter_or<double>("mapping.b_acc_cov", b_acc_cov, 0.0001);
        this->get_parameter_or<double>("preprocess.blind", p_pre->blind, 0.01);
        this->get_parameter_or<int>("preprocess.lidar_type", p_pre->lidar_type, AVIA);
        this->get_parameter_or<int>("preprocess.scan_line", p_pre->N_SCANS, 16);
        this->get_parameter_or<int>("preprocess.timestamp_unit", p_pre->time_unit, US);
        this->get_parameter_or<int>("preprocess.scan_rate", p_pre->SCAN_RATE, 10);
        this->get_parameter_or<int>("point_filter_num", p_pre->point_filter_num, 2);
        this->get_parameter_or<bool>("feature_extract_enable", p_pre->feature_enabled, false);
        this->get_parameter_or<bool>("runtime_pos_log_enable", runtime_pos_log, 0);
        this->get_parameter_or<bool>("mapping.extrinsic_est_en", extrinsic_est_en, true);
        this->get_parameter_or<bool>("pcd_save.pcd_save_en", pcd_save_en, false);
        this->get_parameter_or<int>("pcd_save.interval", pcd_save_interval, -1);
        this->get_parameter_or<vector<double>>("mapping.extrinsic_T", extrinT, vector<double>());
        this->get_parameter_or<vector<double>>("mapping.extrinsic_R", extrinR, vector<double>());


        //其中FOV_DEG是激光雷达的全视场角度,HALF_FOV_COS是角度的一半对应的余弦值。这些值会在后续滤波或特征提取时用到,因此在初始化时先进行计算。而路径消息path的头信息是发布里程计结果时需要的总体上,这段代码主要是进行了一些初始化操作,计算参数并打印,为算法运行做准备。
        RCLCPP_INFO(this->get_logger(), "p_pre->lidar_type %d", p_pre->lidar_type);//打印当前的激光雷达类型p_pre->lidar_type

        path.header.stamp = this->get_clock()->now();//一个time类型的变量，表示消息生成时间。stamp:设置为当前时间
        path.header.frame_id = "camera_init";//一个字符串，表示参考坐标系的ID。frame_id:设置为"camera_init"

        // /*** variables definition ***/
        // int effect_feat_num = 0, frame_num = 0;
        // double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
        // bool flg_EKF_converged, EKF_stop_flg = 0;

        FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);//视场角 179.9
        HALF_FOV_COS = cos((FOV_DEG)*0.5 * PI_M / 180.0);//半视场角的余弦值 0.000872665

        cout<<"FOV_DEG: "<<FOV_DEG<<endl;
        cout<<"HALF_FOV_COS: "<<HALF_FOV_COS<<endl;

        _featsArray.reset(new PointCloudXYZI());//初始化特征点云指针_featsArray


        //可以看到,构造函数除了参数读取和订阅定义,还初始化了算法过程中需要用到的重要变量和对象,为后续的点云处理与状态估计做好了准备。

        //初始化点云选择数组point_selected_surf和上次迭代残差数组res_last。
        memset(point_selected_surf, true, sizeof(point_selected_surf));
        memset(res_last, -1000.0f, sizeof(res_last));

        //设置下采样滤波器downSizeFilterSurf和downSizeFilterMap的滤波参数。
        downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
        downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
        memset(point_selected_surf, true, sizeof(point_selected_surf));
        memset(res_last, -1000.0f, sizeof(res_last));

        //初始化激光雷达到IMU的外参变换位姿Lidar_T_wrt_IMU和Lidar_R_wrt_IMU。
        Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);
        Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);

        //设置IMU预积分模型p_imu的外参和协方差矩阵。
        p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
        p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
        p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
        p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
        p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));

        //初始化卡尔曼滤波器kf的过程噪声协方差矩阵epsi。
        fill(epsi, epsi + 23, 0.001);

        //初始化预积分模型指针p_imu和卡尔曼滤波器对象kf。
        kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);

        //打开日志文件进行调试记录。
        /*** debug record ***/
        // FILE *fp;
        string pos_log_dir = root_dir + "/Log/pos_log.txt";
        fp = fopen(pos_log_dir.c_str(), "w");

        //输出初始化日志。
        // ofstream fout_pre, fout_out, fout_dbg;
        fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"), ios::out);
        fout_out.open(DEBUG_FILE_DIR("mat_out.txt"), ios::out);
        fout_dbg.open(DEBUG_FILE_DIR("dbg.txt"), ios::out);
        if (fout_pre && fout_out)
            cout << "~~~~" << ROOT_DIR << " file opened" << endl;
        else
            cout << "~~~~" << ROOT_DIR << " doesn't exist" << endl;
        

        //创建订阅器
        /*** ROS subscribe initialization ***/
        if (p_pre->lidar_type == AVIA)
        {
            sub_pcl_livox_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(lid_topic, 20, livox_pcl_cbk);
            //数字 20 表示消息队列的最大长度，即最多可以缓存多少条消息。
        }
        else
        {
            sub_pcl_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(lid_topic, 20, standard_pcl_cbk);
        }

        /**
         * 在 C++ 中，可以使用 create_subscription 方法来创建订阅器。
         * 例如，下面的代码创建了一个名为 subscription 的订阅器，它将订阅名为 imu_topic 的主题，消息类型为 sensor_msgs::msg::Imu，队列大小为 10，回调函数为 imu_cbk
         * */
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 10, imu_cbk);// 10 表示消息队列的最大长度，即最多可以缓存多少条消息。
        
        /**
         * 这段C++代码的主要目的是创建一个发布者，用于发布名为/cloud_registered的点云消息。
         * pubLaserCloudFull_是一个发布者对象的引用，它将用于发布点云消息。
         * this->create_publisher方法用于创建发布者。该方法接受两个参数：发布者名称（在本例中为"/cloud_registered"）和消息队列大小（在本例中为20）。
         * 创建发布者后，将其存储在pubLaserCloudFull_变量中。
         * 总之，这段代码的作用是创建一个发布者，用于发布名为/cloud_registered的点云消息，并将发布者对象存储在pubLaserCloudFull_变量中。
        */
        pubLaserCloudFull_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 20);// 20 表示消息队列的最大长度，即最多可以缓存多少条消息。
        pubLaserCloudFull_body_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered_body", 20);
        pubLaserCloudEffect_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_effected", 20);
        pubLaserCloudMap_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map", 20);
        pubOdomAftMapped_ = this->create_publisher<nav_msgs::msg::Odometry>("/Odometry", 20);
        pubPath_ = this->create_publisher<nav_msgs::msg::Path>("/path", 20);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        /**
         * 这段C++代码的主要目的是创建一个定时器，用于定期执行某个回调函数。
         * period_ms是一个std::chrono::milliseconds对象，表示定时器的周期。该对象将1000.0除以100.0，以计算定时器的周期（100毫秒）。
         * timer_是一个定时器对象的引用，它将用于定期执行回调函数。
         * rclcpp::create_timer方法用于创建定时器。该方法接受五个参数：指向当前对象的指针、当前节点的时间器、定时器的周期和回调函数。
         * std::bind函数用于将回调函数与当前对象绑定。该函数接受两个参数：回调函数和指向当前对象的指针。
         * 创建定时器后，将其存储在timer_变量中。
         * 总之，这段代码的作用是创建一个定时器，用于定期执行某个回调函数。定时器的周期为100毫秒，回调函数为LaserMappingNode::timer_callback。
        */
        auto period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0 / 100.0));
        timer_ = rclcpp::create_timer(this, this->get_clock(), period_ms, std::bind(&LaserMappingNode::timer_callback, this));

        //创建定时器，用于定期发布地图。
        auto map_period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0));
        map_pub_timer_ = rclcpp::create_timer(this, this->get_clock(), map_period_ms, std::bind(&LaserMappingNode::map_publish_callback, this));

        map_save_srv_ = this->create_service<std_srvs::srv::Trigger>("map_save", std::bind(&LaserMappingNode::map_save_callback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Node init finished.");//输出信息，初始化结束
    }



    //析构函数
    ~LaserMappingNode()
    {
        fout_out.close();
        fout_pre.close();
        fclose(fp);
    }

private:
    void timer_callback()
    {
        //检查是否成功同步了激光雷达数据包。
        if (sync_packages(Measures))
        {
            //如果是第一个扫描，则记录第一个激光雷达时间。
            if (flg_first_scan)
            {
                first_lidar_time = Measures.lidar_beg_time;

                //设置 IMU 的时间基准。
                p_imu->first_lidar_time = first_lidar_time;

                //将第一个扫描标志设置为 false。
                flg_first_scan = false;

                return;
            }

            double t0, t1, t2, t3, t4, t5, match_start, solve_start, svd_time;//定义时间变量。

            match_time = 0;
            kdtree_search_time = 0.0;
            solve_time = 0;
            solve_const_H_time = 0;
            svd_time = 0;
            t0 = omp_get_wtime();

            p_imu->Process(Measures, kf, feats_undistort);//IMU 处理数据包，并将结果存储在 kf 和 feats_undistort 中。
            state_point = kf.get_x();
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;

            if (feats_undistort->empty() || (feats_undistort == NULL))//检查点云是否为空。
            {
                RCLCPP_WARN(this->get_logger(), "No point, skip this scan!\n");//如果点云为空，则跳过该扫描。
                return;
            }

            //检查 EKF 是否已初始化。
            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? false : true;
            /*** Segment the map in lidar FOV ***/
            lasermap_fov_segment();//进行激光雷达地图的扇形分割。


            /*** downsample the feature points in a scan ***/
            downSizeFilterSurf.setInputCloud(feats_undistort);//设置点云过滤器参数。
            downSizeFilterSurf.filter(*feats_down_body);//应用点云过滤器。
            t1 = omp_get_wtime();
            feats_down_size = feats_down_body->points.size();//获取过滤后的点云数量。
            

            /*** initialize the map kdtree ***/
            if (ikdtree.Root_Node == nullptr)
            {
                RCLCPP_INFO(this->get_logger(), "Initialize the map kdtree");
                if (feats_down_size > 5)//如果过滤后的点云数量小于 5，则跳过该扫描。
                {
                    ikdtree.set_downsample_param(filter_size_map_min);
                    feats_down_world->resize(feats_down_size);
                    for (int i = 0; i < feats_down_size; i++)
                    {
                        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                    }
                    ikdtree.Build(feats_down_world->points);
                }
                return;
            }
            int featsFromMapNum = ikdtree.validnum();
            kdtree_size_st = ikdtree.size();

            // cout<<"[ mapping ]: In num: "<<feats_undistort->points.size()<<" downsamp "<<feats_down_size<<" Map num: "<<featsFromMapNum<<"effect num:"<<effct_feat_num<<endl;

            /*** ICP and iterated Kalman filter update ***/
            if (feats_down_size < 5)
            {
                RCLCPP_WARN(this->get_logger(), "No point, skip this scan!\n");
                return;
            }

            normvec->resize(feats_down_size);//重新调整 normvec 的大小。
            feats_down_world->resize(feats_down_size);//重新调整 feats_down_world 的大小。

            V3D ext_euler = SO3ToEuler(state_point.offset_R_L_I);//将旋转矩阵转换为欧拉角。

            
            fout_pre << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_point.pos.transpose() << " " << ext_euler.transpose() << " " << state_point.offset_T_L_I.transpose() << " " << state_point.vel.transpose()
                     << " " << state_point.bg.transpose() << " " << state_point.ba.transpose() << " " << state_point.grav << endl;

            if (0) // If you need to see map point, change to "if(1)"
            {
                PointVector().swap(ikdtree.PCL_Storage);
                ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
                featsFromMap->clear();
                featsFromMap->points = ikdtree.PCL_Storage;
            }


            pointSearchInd_surf.resize(feats_down_size);
            Nearest_Points.resize(feats_down_size);
            int rematch_num = 0;
            bool nearest_search_en = true; //

            t2 = omp_get_wtime();

            /*** iterated state estimation ***/
            double t_update_start = omp_get_wtime();
            double solve_H_time = 0;
            kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
            state_point = kf.get_x();
            euler_cur = SO3ToEuler(state_point.rot);
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
            geoQuat.x = state_point.rot.coeffs()[0];
            geoQuat.y = state_point.rot.coeffs()[1];
            geoQuat.z = state_point.rot.coeffs()[2];
            geoQuat.w = state_point.rot.coeffs()[3];

            double t_update_end = omp_get_wtime();

            /******* Publish odometry *******/
            publish_odometry(pubOdomAftMapped_, tf_broadcaster_);

            /*** add the feature points to map kdtree ***/
            t3 = omp_get_wtime();
            map_incremental();
            t5 = omp_get_wtime();

            /******* Publish points *******/
            if (path_en)
                publish_path(pubPath_);
            if (scan_pub_en)
                publish_frame_world(pubLaserCloudFull_);
            if (scan_pub_en && scan_body_pub_en)
                publish_frame_body(pubLaserCloudFull_body_);
            if (effect_pub_en)
                publish_effect_world(pubLaserCloudEffect_);
            // if (map_pub_en) publish_map(pubLaserCloudMap_);

            /*** Debug variables ***/
            if (runtime_pos_log)
            {
                frame_num++;
                kdtree_size_end = ikdtree.size();
                aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
                aver_time_icp = aver_time_icp * (frame_num - 1) / frame_num + (t_update_end - t_update_start) / frame_num;
                aver_time_match = aver_time_match * (frame_num - 1) / frame_num + (match_time) / frame_num;
                aver_time_incre = aver_time_incre * (frame_num - 1) / frame_num + (kdtree_incremental_time) / frame_num;
                aver_time_solve = aver_time_solve * (frame_num - 1) / frame_num + (solve_time + solve_H_time) / frame_num;
                aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1) / frame_num + solve_time / frame_num;
                T1[time_log_counter] = Measures.lidar_beg_time;
                s_plot[time_log_counter] = t5 - t0;
                s_plot2[time_log_counter] = feats_undistort->points.size();
                s_plot3[time_log_counter] = kdtree_incremental_time;
                s_plot4[time_log_counter] = kdtree_search_time;
                s_plot5[time_log_counter] = kdtree_delete_counter;
                s_plot6[time_log_counter] = kdtree_delete_time;
                s_plot7[time_log_counter] = kdtree_size_st;
                s_plot8[time_log_counter] = kdtree_size_end;
                s_plot9[time_log_counter] = aver_time_consu;
                s_plot10[time_log_counter] = add_point_size;
                time_log_counter++;
                printf("[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: %0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f construct H: %0.6f \n", t1 - t0, aver_time_match, aver_time_solve, t3 - t1, t5 - t3, aver_time_consu, aver_time_icp, aver_time_const_H_time);
                ext_euler = SO3ToEuler(state_point.offset_R_L_I);
                fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_point.pos.transpose() << " " << ext_euler.transpose() << " " << state_point.offset_T_L_I.transpose() << " " << state_point.vel.transpose()
                         << " " << state_point.bg.transpose() << " " << state_point.ba.transpose() << " " << state_point.grav << " " << feats_undistort->points.size() << endl;
                dump_lio_state_to_log(fp);
            }
        }
    }



    /**
     * 定义了一个名为map_publish_callback的函数，该函数在map_pub_en条件为真时执行。
     * 具体来说，当map_pub_en的值为1时，它会调用publish_map函数，并将结果赋值给pubLaserCloudMap_。
     * publish_map函数可能用于发布地图数据，具体实现取决于上下文。
     * 此外，pubLaserCloudMap_可能是一个发布地图数据的发布者对象，用于将地图数据发布到特定的主题上。
    */
    void map_publish_callback()
    {
        if (map_pub_en){
            publish_map(pubLaserCloudMap_);
    }



    /**
     * 定义了一个名为map_save_callback的函数，该函数是一个回调函数，用于处理服务请求。当服务请求被触发时，它会执行以下操作：
     * std_srvs::srv::Trigger::Request::ConstSharedPtr req和std_srvs::srv::Trigger::Response::SharedPtr res分别表示服务请求和响应的指针。
     * map_file_path可能是一个字符串，表示地图保存的文件路径。
    */
    void map_save_callback(std_srvs::srv::Trigger::Request::ConstSharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        //输出一条日志信息，表示正在保存地图到指定的文件路径。
        RCLCPP_INFO(this->get_logger(), "Saving map to %s...", map_file_path.c_str());

        //检查pcd_save_en的值，如果为真，则调用save_to_pcd函数将地图保存到pcd文件中。
        if (pcd_save_en)
        {
            save_to_pcd();
            res->success = true; //返回一个成功响应，表示地图已成功保存。
            res->message = "Map saved.";
        }
        else //如果pcd_save_en的值为假，则返回一个失败响应，表示地图保存功能已被禁用。
        {
            res->success = false;
            res->message = "Map save disabled.";
        }
    }


private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_body_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_pc_;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl_livox_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr map_pub_timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr map_save_srv_;

    bool effect_pub_en = false, map_pub_en = false;
    int effect_feat_num = 0, frame_num = 0;
    double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
    bool flg_EKF_converged, EKF_stop_flg = 0;
    double epsi[23] = {0.001};

    FILE *fp;
    ofstream fout_pre, fout_out, fout_dbg;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);/* 初始化rclcpp  */

    signal(SIGINT, SigHandle);

    /* 运行节点，并检测退出信号 Ctrl+C*/
    rclcpp::spin(std::make_shared<LaserMappingNode>());//产生一个LaserMappingNode的节点

    if (rclcpp::ok())
        rclcpp::shutdown();
    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. pcd save will largely influence the real-time performences **/
    if (pcl_wait_save->size() > 0 && pcd_save_en)
    {
        string file_name = string("scans.pcd");
        string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
        pcl::PCDWriter pcd_writer;
        cout << "current scan saved to /PCD/" << file_name<<endl;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    }

    if (runtime_pos_log)
    {
        vector<double> t, s_vec, s_vec2, s_vec3, s_vec4, s_vec5, s_vec6, s_vec7;    
        FILE *fp2;
        string log_dir = root_dir + "/Log/fast_lio_time_log.csv";
        fp2 = fopen(log_dir.c_str(),"w");
        fprintf(fp2,"time_stamp, total time, scan point size, incremental time, search time, delete size, delete time, tree size st, tree size end, add point size, preprocess time\n");
        for (int i = 0;i<time_log_counter; i++){
            fprintf(fp2,"%0.8f,%0.8f,%d,%0.8f,%0.8f,%d,%0.8f,%d,%d,%d,%0.8f\n",T1[i],s_plot[i],int(s_plot2[i]),s_plot3[i],s_plot4[i],int(s_plot5[i]),s_plot6[i],int(s_plot7[i]),int(s_plot8[i]), int(s_plot10[i]), s_plot11[i]);
            t.push_back(T1[i]);
            s_vec.push_back(s_plot9[i]);
            s_vec2.push_back(s_plot3[i] + s_plot6[i]);
            s_vec3.push_back(s_plot4[i]);
            s_vec5.push_back(s_plot[i]);
        }
        fclose(fp2);
    }

    return 0;
}