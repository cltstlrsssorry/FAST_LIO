#include <cmath>
#include <math.h>
#include <deque>
#include <mutex>
#include <thread>
#include <fstream>
#include <csignal>
#include <so3_math.h>
#include <Eigen/Eigen>
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
#include "use-ikfom.hpp"

/// *************Preconfiguration

#define MAX_INI_COUNT (10)

const bool time_list(PointType &x, PointType &y) { return (x.curvature < y.curvature); };

/// *************IMU Process and undistortion
class ImuProcess
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuProcess();
  ~ImuProcess();

  void Reset();
  // void Reset(double start_timestamp, const sensor_msgs::ImuConstPtr &lastimu);
  void Reset(double start_timestamp, const sensor_msgs::msg::Imu::ConstSharedPtr &lastimu);
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
  Eigen::Matrix<double, 12, 12> Q;
  void Process(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI::Ptr pcl_un_);

  ofstream fout_imu;
  V3D cov_acc;
  V3D cov_gyr;
  V3D cov_acc_scale;
  V3D cov_gyr_scale;
  V3D cov_bias_gyr;
  V3D cov_bias_acc;
  double first_lidar_time;

private:
  void IMU_init(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, int &N);
  void UndistortPcl(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI &pcl_in_out);

  PointCloudXYZI::Ptr cur_pcl_un_;
  // sensor_msgs::ImuConstPtr last_imu_;
  sensor_msgs::msg::Imu::ConstSharedPtr last_imu_;
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
  int init_iter_num = 1;
  bool b_first_frame_ = true;
  bool imu_need_init_ = true;
};

ImuProcess::ImuProcess(): b_first_frame_(true), imu_need_init_(true), start_timestamp_(-1)
{
  init_iter_num = 1;
  Q = process_noise_cov();
  cov_acc = V3D(0.1, 0.1, 0.1);
  cov_gyr = V3D(0.1, 0.1, 0.1);
  cov_bias_gyr = V3D(0.0001, 0.0001, 0.0001);
  cov_bias_acc = V3D(0.0001, 0.0001, 0.0001);
  mean_acc = V3D(0, 0, -1.0);
  mean_gyr = V3D(0, 0, 0);
  angvel_last = Zero3d;
  Lidar_T_wrt_IMU = Zero3d;
  Lidar_R_wrt_IMU = Eye3d;
  last_imu_.reset(new sensor_msgs::msg::Imu());
}

ImuProcess::~ImuProcess() {}

void ImuProcess::Reset()
{
  // ROS_WARN("Reset ImuProcess");
  mean_acc = V3D(0, 0, -1.0);
  mean_gyr = V3D(0, 0, 0);
  angvel_last = Zero3d;
  imu_need_init_ = true;
  start_timestamp_ = -1;
  init_iter_num = 1;
  v_imu_.clear();
  IMUpose.clear();
  last_imu_.reset(new sensor_msgs::msg::Imu());
  cur_pcl_un_.reset(new PointCloudXYZI());
}

void ImuProcess::set_extrinsic(const MD(4, 4) & T)
{
  Lidar_T_wrt_IMU = T.block<3, 1>(0, 3);
  Lidar_R_wrt_IMU = T.block<3, 3>(0, 0);
}

void ImuProcess::set_extrinsic(const V3D &transl)
{
  Lidar_T_wrt_IMU = transl;
  Lidar_R_wrt_IMU.setIdentity();
}

void ImuProcess::set_extrinsic(const V3D &transl, const M3D &rot)
{
  Lidar_T_wrt_IMU = transl;
  Lidar_R_wrt_IMU = rot;
}

void ImuProcess::set_gyr_cov(const V3D &scaler)
{
  cov_gyr_scale = scaler;
}

void ImuProcess::set_acc_cov(const V3D &scaler)
{
  cov_acc_scale = scaler;
}

void ImuProcess::set_gyr_bias_cov(const V3D &b_g)
{
  cov_bias_gyr = b_g;
}

void ImuProcess::set_acc_bias_cov(const V3D &b_a)
{
  cov_bias_acc = b_a;
}

void ImuProcess::IMU_init(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, int &N)
{
  /** 1. initializing the gravity, gyro bias, acc and gyro covariance
   ** 2. normalize the acceleration measurenments to unit gravity **/

  /** 1. 初始化重力、陀螺偏差、acc和陀螺仪协方差
  /** 2. 将加速度测量值标准化为单位重力**/
  // 这里应该是静止初始化

  // 加速度、角速度
  V3D cur_acc, cur_gyr;

  //// 判断是否为第一帧
  if (b_first_frame_)
  {
    Reset(); // 重置参数
    N = 1;   // 将迭代次数置1
    b_first_frame_ = false;
    const auto &imu_acc = meas.imu.front()->linear_acceleration; // 从common_lib.h中拿到imu初始时刻的加速度
    const auto &gyr_acc = meas.imu.front()->angular_velocity;    // 从common_lib.h中拿到imu初始时刻的角速度
    mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;                 // 加速度测量作为初始化均值
    mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;                 // 角速度测量作为初始化均值
    first_lidar_time = meas.lidar_beg_time;                      // 将当期imu帧对应的lidar时间作为初始时间
  }

  // 计算方差
  for (const auto &imu : meas.imu) // 拿到所有的imu帧
  {
    const auto &imu_acc = imu->linear_acceleration;
    const auto &gyr_acc = imu->angular_velocity;
    cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;
    // 根据当前帧和均值差作为均值的更新

    mean_acc += (cur_acc - mean_acc) / N;
    mean_gyr += (cur_gyr - mean_gyr) / N;
    /**
     * .cwiseProduct()对应系数相乘
     * 每次迭代之后均值都会发生变化，最后的方差公式中减的应该是最后的均值
     * https://blog.csdn.net/weixin_44479136/article/details/90510374 方差迭代计算公式
     * 按照博客推导出来的下面方差递推公式有两种
     *
     * 第一种是
     * cwiseProduct()方法进行矩阵间的元素级乘积（对应点相乘）。
     */
    cov_acc = cov_acc * (N - 1.0) / N + (cur_acc - mean_acc).cwiseProduct(cur_acc - mean_acc) * (N - 1.0) / (N * N);
    cov_gyr = cov_gyr * (N - 1.0) / N + (cur_gyr - mean_gyr).cwiseProduct(cur_gyr - mean_gyr) * (N - 1.0) / (N * N);

    // 第二种是
    // cov_acc = cov_acc * (N - 1.0) / N + (cur_acc - mean_acc).cwiseProduct(cur_acc - 上一次的mean_acc)  / N;
    // cov_gyr = cov_gyr * (N - 1.0) / N + (cur_gyr - mean_gyr).cwiseProduct(cur_gyr - 上一次的mean_gyr)  / N;

    // cout<<"acc norm: "<<cur_acc.norm()<<" "<<mean_acc.norm()<<endl;

    N++;
  }

  state_ikfom init_state = kf_state.get_x();                  // 在esekfom.hpp获得x_的状态
  init_state.grav = S2(-mean_acc / mean_acc.norm() * G_m_s2); // 从common_lib.h中拿到重力，并与加速度测量均值的单位重力求出SO2的旋转矩阵类型的重力加速度

  // state_inout.rot = Eye3d; // Exp(mean_acc.cross(V3D(0, 0, -1 / scale_gravity)));
  init_state.bg = mean_gyr;                  // 角速度测量作为陀螺仪偏差
  init_state.offset_T_L_I = Lidar_T_wrt_IMU; // 将lidar和imu外参位移量传入
  init_state.offset_R_L_I = Lidar_R_wrt_IMU; // 将lidar和imu外参旋转量传入
  kf_state.change_x(init_state);             // 将初始化状态传入esekfom.hpp中的x_

  esekfom::esekf<state_ikfom, 12, input_ikfom>::cov init_P = kf_state.get_P(); // 在esekfom.hpp获得P_的协方差矩阵
  init_P.setIdentity();                                                        // 将协方差矩阵置为单位阵
  init_P(6, 6) = init_P(7, 7) = init_P(8, 8) = 0.00001;                        // 将协方差矩阵的位置和旋转的协方差置为0.00001
  init_P(9, 9) = init_P(10, 10) = init_P(11, 11) = 0.00001;                    // 将协方差矩阵的速度和位姿的协方差置为0.00001
  init_P(15, 15) = init_P(16, 16) = init_P(17, 17) = 0.0001;                   // 将协方差矩阵的重力和姿态的协方差置为0.0001
  init_P(18, 18) = init_P(19, 19) = init_P(20, 20) = 0.001;                    // 将协方差矩阵的陀螺仪偏差和姿态的协方差置为0.001
  init_P(21, 21) = init_P(22, 22) = 0.00001;                                   // 将协方差矩阵的lidar和imu外参位移量的协方差置为0.00001
  kf_state.change_P(init_P);                                                   // 将初始化协方差矩阵传入esekfom.hpp中的P_
  last_imu_ = meas.imu.back();                                                 // 将最后一帧的imu数据传入last_imu_中，暂时没用到
}

/**
 * 在UndistortPcl函数中不但有IMU的前向信息，还有激光雷达去畸变的问题，
 * 这一节我们围绕着IMU的正向传播展开，代码中通过迭代的形式完成了IMU数据的更新，
 * 并将acc和gyro的数据传入到ESKF中，详细的公式我们后面再来讲。
*/
void ImuProcess::UndistortPcl(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI &pcl_out)
{

  /*** add the imu of the last frame-tail to the of current frame-head ***/
  // 拿到当前的imu数据
  auto v_imu = meas.imu;
  // 将上一帧最后尾部的imu添加到当前帧头部的imu
  v_imu.push_front(last_imu_);

  // 拿到当前帧头部的imu的时间（也就是上一帧尾部的imu时间戳）
  const double &imu_beg_time = rclcpp::Time(v_imu.front()->header.stamp).seconds();
  // 拿到当前帧尾部的imu的时间
  const double &imu_end_time = rclcpp::Time(v_imu.back()->header.stamp).seconds();
  
  // pcl开始的时间戳
  const double &pcl_beg_time = meas.lidar_beg_time;
  // pcl结束的时间戳
  const double &pcl_end_time = meas.lidar_end_time;

  /*** sort point clouds by offset time ***/
  // 根据点云中每个点的时间戳对点云进行重排序
  pcl_out = *(meas.lidar);
  // 拿到最后一帧时间戳加上最后一帧的所需要的时间/1000得到点云的结束时间戳
  sort(pcl_out.points.begin(), pcl_out.points.end(), time_list); 

  // cout<<"[ IMU Process ]: Process lidar from "<<pcl_beg_time<<" to "<<pcl_end_time<<", " \
  //          <<meas.imu.size()<<" imu msgs from "<<imu_beg_time<<" to "<<imu_end_time<<endl;


  /*** Initialize IMU pose ***/
  // 获取上一次KF估计的后验状态作为本次IMU预测的初始状态
  state_ikfom imu_state = kf_state.get_x(); 
  // 清空IMUpose
  IMUpose.clear();
  // 将初始状态加入IMUpose中,包含有时间间隔，上一帧加速度，上一帧角速度，上一帧速度，上一帧位置，上一帧旋转矩阵
  IMUpose.push_back(set_pose6d(0.0, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix())); 


  /*** forward propagation at each imu point ***/
  // angvel_avr为平均角速度，acc_avr为平均加速度，acc_imu为imu加速度，vel_imu为imu速度，pos_imu为imu位置
  V3D angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;

  // imu旋转矩阵
  M3D R_imu;

  // 时间间隔
  double dt = 0;

  // eksf 传入的参数
  input_ikfom in;

  // 遍历本次估计的所有IMU测量并且进行积分，离散中值法 前向传播
  for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++)
  {
    auto &&head = *(it_imu);     // 拿到当前帧的imu数据
    auto &&tail = *(it_imu + 1); // 拿到下一帧的imu数据

    double tail_stamp = rclcpp::Time(tail->header.stamp).seconds();
    double head_stamp = rclcpp::Time(head->header.stamp).seconds();

    if (tail_stamp < last_lidar_end_time_) // 判断时间先后顺序 不符合直接continue
      continue;

    // 中值积分
    angvel_avr << 0.5 * (head->angular_velocity.x + tail->angular_velocity.x),
        0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
        0.5 * (head->angular_velocity.z + tail->angular_velocity.z);
    acc_avr << 0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x),
        0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
        0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);

    // fout_imu << setw(10) << head->header.stamp.toSec() - first_lidar_time << " " << angvel_avr.transpose() << " " << acc_avr.transpose() << endl;

    // 通过重力数值对加速度进行一下微调
    acc_avr = acc_avr * G_m_s2 / mean_acc.norm(); // - state_inout.ba;

    // 如果IMU开始时刻早于上次雷达最晚时刻(因为将上次最后一个IMU插入到此次开头了，所以会出现一次这种情况)
    if (head_stamp < last_lidar_end_time_)
    {
      // 从上次雷达时刻末尾开始传播 计算与此次IMU结尾之间的时间差
      dt = tail_stamp - last_lidar_end_time_;
      // dt = tail->header.stamp.toSec() - pcl_beg_time;
    }
    else
    {
      // 两个IMU时刻之间的时间间隔
      dt = tail_stamp - head_stamp;
    }

    // 原始测量的中值作为更新
    in.acc = acc_avr;
    in.gyro = angvel_avr;

    // 配置协方差矩阵
    Q.block<3, 3>(0, 0).diagonal() = cov_gyr;
    Q.block<3, 3>(3, 3).diagonal() = cov_acc;
    Q.block<3, 3>(6, 6).diagonal() = cov_bias_gyr;
    Q.block<3, 3>(9, 9).diagonal() = cov_bias_acc;

    // IMU前向传播，每次传播的时间间隔为dt
    kf_state.predict(dt, Q, in); 

    /* save the poses at each IMU measurements */
    // 保存IMU预测过程的状态
    imu_state = kf_state.get_x();
    // 计算出来的角速度与预测的角速度的差值
    angvel_last = angvel_avr - imu_state.bg;
    // 计算出来的加速度与预测的加速度的差值,并转到IMU坐标系下
    acc_s_last = imu_state.rot * (acc_avr - imu_state.ba);

    for (int i = 0; i < 3; i++)
    {
      acc_s_last[i] += imu_state.grav[i]; // 加上重力得到世界坐标系的加速度
    }

    // 后一个IMU时刻距离此次雷达开始的时间间隔
    double &&offs_t = tail_stamp - pcl_beg_time;
    // 保存IMU预测过程的状态
    IMUpose.push_back(set_pose6d(offs_t, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix()));
  }

  /*** calculated the pos and attitude prediction at the frame-end ***/
  // 把最后一帧IMU测量也补上
  // 判断雷达结束时间是否晚于IMU，最后一个IMU时刻可能早于雷达末尾 也可能晚于雷达末尾
  double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;
  dt = note * (pcl_end_time - imu_end_time);
  kf_state.predict(dt, Q, in);

  imu_state = kf_state.get_x();        // 更新IMU状态，以便于下一帧使用
  last_imu_ = meas.imu.back();         // 保存最后一个IMU测量，以便于下一帧使用
  last_lidar_end_time_ = pcl_end_time; // 保存这一帧最后一个雷达测量的结束时间，以便于下一帧使用

  /*** undistort each lidar point (backward propagation) ***/
  if (pcl_out.points.begin() == pcl_out.points.end())
    return;
  auto it_pcl = pcl_out.points.end() - 1;
  for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--)
  {
    auto head = it_kp - 1;
    auto tail = it_kp;
    R_imu << MAT_FROM_ARRAY(head->rot);
    // cout<<"head imu acc: "<<acc_imu.transpose()<<endl;
    vel_imu << VEC_FROM_ARRAY(head->vel);
    pos_imu << VEC_FROM_ARRAY(head->pos);
    acc_imu << VEC_FROM_ARRAY(tail->acc);
    angvel_avr << VEC_FROM_ARRAY(tail->gyr);

    for (; it_pcl->curvature / double(1000) > head->offset_time; it_pcl--)
    {
      dt = it_pcl->curvature / double(1000) - head->offset_time;

      /* Transform to the 'end' frame, using only the rotation
       * Note: Compensation direction is INVERSE of Frame's moving direction
       * So if we want to compensate a point at timestamp-i to the frame-e
       * P_compensate = R_imu_e ^ T * (R_i * P_i + T_ei) where T_ei is represented in global frame */
      M3D R_i(R_imu * Exp(angvel_avr, dt));

      V3D P_i(it_pcl->x, it_pcl->y, it_pcl->z);
      V3D T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - imu_state.pos);
      V3D P_compensate = imu_state.offset_R_L_I.conjugate() * (imu_state.rot.conjugate() * (R_i * (imu_state.offset_R_L_I * P_i + imu_state.offset_T_L_I) + T_ei) - imu_state.offset_T_L_I); // not accurate!

      // save Undistorted points and their rotation
      it_pcl->x = P_compensate(0);
      it_pcl->y = P_compensate(1);
      it_pcl->z = P_compensate(2);

      if (it_pcl == pcl_out.points.begin())
        break;
    }
  }
}

// 定义处理IMU数据的函数，参数为测量组、EKF对象和点云指针。
void ImuProcess::Process(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI::Ptr cur_pcl_un_)
{
  double t1, t2, t3;
  t1 = omp_get_wtime();

  // 如果IMU数据为空，则返回。
  if (meas.imu.empty())
  {
    return;
  };

  // 断言LIDAR数据不为空。
  assert(meas.lidar != nullptr);

  // 如果需要初始化IMU，则执行以下操作。
  if (imu_need_init_)
  {
    // 初始化IMU。
    // The very first lidar frame
    IMU_init(meas, kf_state, init_iter_num);

    // 设置IMU初始化标志为true。
    imu_need_init_ = true;

    // meas数据出栈
    last_imu_ = meas.imu.back();

    /**这段代码从kf_state对象中获取当前状态，并将结果存储在imu_state变量中。
     * kf_state是一个扩展卡尔曼滤波器（EKF）对象，用于估计动态系统的状态。
     * get_x方法返回当前状态，它是一个state_ikfom类型的变量，表示IMU姿态估计的内部状态。
     * 通过获取当前状态，我们可以使用它来执行其他操作，例如更新姿态估计、预测姿态估计或计算与IMU数据相关的其他信息。
     */
    state_ikfom imu_state = kf_state.get_x(); // 获取EKF状态。

    if (init_iter_num > MAX_INI_COUNT)
    {
      cov_acc *= pow(G_m_s2 / mean_acc.norm(), 2); // 更新加速度协方差。
      imu_need_init_ = false;                      // 设置IMU初始化标志为false。

      cov_acc = cov_acc_scale;                      // 设置加速度协方差。
      cov_gyr = cov_gyr_scale;                      // 设置陀螺仪协方差。
      std::cout << "IMU Initial Done" << std::endl; // 输出IMU初始化完成信息。
      // ROS_INFO("IMU Initial Done: Gravity: %.4f %.4f %.4f %.4f; state.bias_g: %.4f %.4f %.4f; acc covarience: %.8f %.8f %.8f; gry covarience: %.8f %.8f %.8f",\
      //          imu_state.grav[0], imu_state.grav[1], imu_state.grav[2], mean_acc.norm(), cov_bias_gyr[0], cov_bias_gyr[1], cov_bias_gyr[2], cov_acc[0], cov_acc[1], cov_acc[2], cov_gyr[0], cov_gyr[1], cov_gyr[2]);
      fout_imu.open(DEBUG_FILE_DIR("imu.txt"), ios::out); // 打开IMU数据文件。
    }

    return;
  }

  // 对点云进行去畸变操作。
  UndistortPcl(meas, kf_state, *cur_pcl_un_);

  t2 = omp_get_wtime(); // 记录去畸变操作结束时间。
  t3 = omp_get_wtime(); // 记录总处理时间。

  // 输出IMU处理总时间。
  //  cout<<"[ IMU Process ]: Time: "<<t3 - t1<<endl;
}
