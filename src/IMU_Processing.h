# pragma once

#include <cmath>
#include <math.h>
#include <deque>
#include <mutex>
#include <thread>
#include <fstream>
#include <csignal>
#include <so3_math.h>
#include <Eigen/Eigen>

#include <omp.h>

#include <common_lib.h>

#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <condition_variable>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <use-ikfom.h>

#include <fastlio.h>

/// *************Preconfiguration

#define MAX_INI_COUNT (10)

const bool time_list(PointType &x, PointType &y);

/// *************IMU Process and undistortion
class ImuProcess
{
public:
  // 声明一个对齐操作符new的函数
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuProcess();
  ~ImuProcess();

  // 重置类的状态
  void Reset();
  // void Reset(double start_timestamp, const sensor_msgs::ImuConstPtr &lastimu);
  void Reset(double start_timestamp, const sensor_msgs::msg::Imu::ConstSharedPtr &lastimu);

  // 设置IMU的 extrinsic 参数
  void set_extrinsic(const V3D &transl, const M3D &rot);
  void set_extrinsic(const V3D &transl);
  void set_extrinsic(const MD(4, 4) & T);

  // 设置陀螺仪测量值的协方差
  void set_gyr_cov(const V3D &scaler);
  // 设置加速度计测量值的协方差
  void set_acc_cov(const V3D &scaler);

  // 设置陀螺仪偏置的协方差
  void set_gyr_bias_cov(const V3D &b_g);
  // 设置加速度计偏置的协方差
  void set_acc_bias_cov(const V3D &b_a);

  // 声明一个成员变量类型为Eigen::Matrix<double, 12, 12>
  Eigen::Matrix<double, 12, 12> Q;

  // 处理测量并更新类的状态
  void Process(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI::Ptr pcl_un_);

  // 声明一个成员变量类型为ofstream
  ofstream fout_imu;

  // 声明一个成员变量类型为V3D
  V3D cov_acc;
  V3D cov_gyr;
  V3D cov_acc_scale;
  V3D cov_gyr_scale;
  V3D cov_bias_gyr;
  V3D cov_bias_acc;

  // 声明一个成员变量类型为double
  double first_lidar_time;

private:
  // 初始化
  void IMU_init(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, int &N);

  // 纠正点云
  void UndistortPcl(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI &pcl_in_out);

  // 声明一个成员变量类型为PointCloudXYZI::Ptr
  PointCloudXYZI::Ptr cur_pcl_un_;

  // sensor_msgs::ImuConstPtr last_imu_;
  // 声明一个成员变量类型为sensor_msgs::msg::Imu::ConstSharedPtr
  sensor_msgs::msg::Imu::ConstSharedPtr last_imu_;

  // 声明一个成员变量类型为std::deque<sensor_msgs::msg::Imu::ConstSharedPtr>
  deque<sensor_msgs::msg::Imu::ConstSharedPtr> v_imu_;

  vector<Pose6D> IMUpose;
  vector<M3D> v_rot_pcl_;
  M3D Lidar_R_wrt_IMU;
  V3D Lidar_T_wrt_IMU;
  V3D mean_acc;
  V3D mean_gyr;
  V3D angvel_last;
  V3D acc_s_last;

  double start_timestamp_;
  double last_lidar_end_time_;
  int init_iter_num ;
  bool b_first_frame_ ;
  bool imu_need_init_ ;
};