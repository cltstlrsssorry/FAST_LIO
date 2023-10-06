// #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

using namespace std;

#define IS_VALID(a) ((abs(a) > 1e8) ? true : false)

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

enum LID_TYPE // 枚举类型：表示支持的雷达类型
{
  AVIA = 1,
  VELO16,
  OUST64,
  MID360
}; //{1, 2, 3}

enum TIME_UNIT//枚举时间单元
{
  SEC = 0,
  MS = 1,
  US = 2,
  NS = 3
};

enum Feature // 枚举类型：表示特征点的类型
{
  Nor,        // 正常点
  Poss_Plane, // 可能的平面点
  Real_Plane, // 确定的平面点
  Edge_Jump,  // 有跨越的边
  Edge_Plane, // 边上的平面点
  Wire,       // 线段 这个也许当了无效点？也就是空间中的小线段？
  ZeroPoint   // 无效点 程序中未使用
};

enum Surround // 枚举类型：位置标识
{
  Prev, // 前一个
  Next  // 后一个
};

enum E_jump // 枚举类型：表示有跨越边的类型
{
  Nr_nor,  // 正常
  Nr_zero, // 0
  Nr_180,  // 180
  Nr_inf,  // 无穷大 跳变较远？
  Nr_blind // 在盲区？
};

struct orgtype // orgtype类：用于存储激光雷达点的一些其他属性
{
  double range; // range: 激光点到传感器的距离  types[i_cur].range: 当前点i_cur到传感器的距离
  double dista; // dista: 当前点到后一个点的距离
  // 假设雷达原点为O 前一个点为M 当前点为A 后一个点为N
  double angle[2];  // angle[2]: 当前点与前后两个点的角度余弦值
  double intersect; // intersect: 前后两点夹角的余弦值
  E_jump edj[2];    // edj[2]: 前后两点的边缘类型
  Feature ftype;    // ftype: 点的特征类型

  //在构造函数中会给些变量赋默认值。这样每个激光点都用一个orgtype对象来描述,包含了点的距离、角度、边缘、特征等信息。这些信息构成了点的特征描述子,可以用于特征匹配,提高点云配准的效果。是一种为激光点增加特征以辅助配准的常用方法。

  orgtype() // 构造函数
  {
    range = 0;
    edj[Prev] = Nr_nor;
    edj[Next] = Nr_nor;
    ftype = Nor; // 默认为正常点
    intersect = 2;
  }
};

namespace velodyne_ros // velodyne数据结构
{
  struct EIGEN_ALIGN16 Point
  {
    PCL_ADD_POINT4D;                // 4D点坐标类型
    float intensity;                // 强度
    float time;                     // 时间
    uint16_t ring;                  // 点所属的圈数
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // 进行内存对齐
  };
} // namespace velodyne_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,(float, x, x)(float, y, y)(float, z, z)(float, intensity,intensity)(float, time, time)(uint16_t, ring,ring))

namespace ouster_ros // ouster数据结构
{
  struct EIGEN_ALIGN16 Point
  {
    PCL_ADD_POINT4D;                // 4D点坐标类型
    float intensity;                // 强度
    uint32_t t;                     // 时间
    uint16_t reflectivity;          // 反射率
    uint8_t ring;                   // 点所属的圈数
    uint16_t ambient;               // 没用到
    uint32_t range;                 // 距离
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // 进行内存对齐
  };
} // namespace ouster_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,// 注册ouster的Point类型
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    // use std::uint32_t to avoid conflicting with pcl::uint32_t
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint8_t, ring, ring)
    (std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range)
)


namespace livox_ros
{
typedef struct {
  float x;            /**< X axis, Unit:m */
  float y;            /**< Y axis, Unit:m */
  float z;            /**< Z axis, Unit:m */
  float intensity; /**< Reflectivity 、intensity  */
  uint8_t tag;        /**< Livox point tag   */
  uint8_t line;       /**< Laser line id     */
} LivoxPointXyzrtl;
}//创建结构体数据类型

POINT_CLOUD_REGISTER_POINT_STRUCT(livox_ros::LivoxPointXyzrtl,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (uint8_t, tag, tag)
    (uint8_t, line, line)
)//注册livox雷达点云

class Preprocess
{
  public:
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Preprocess();//构造函数
  ~Preprocess();//析构函数
  
  // 对Livox自定义Msg格式的激光雷达数据进行处理
  void process(const livox_ros_driver2::msg::CustomMsg::UniquePtr &msg, PointCloudXYZI::Ptr &pcl_out);

  // 对ros的Msg格式的激光雷达数据进行处理
  void process(const sensor_msgs::msg::PointCloud2::UniquePtr &msg, PointCloudXYZI::Ptr &pcl_out);
  void set(bool feat_en, int lid_type, double bld, int pfilt_num);

  // sensor_msgs::PointCloud2::ConstPtr pointcloud;
  PointCloudXYZI pl_full, pl_corn, pl_surf;// 全部点、边缘点、平面点
  PointCloudXYZI pl_buff[128]; //maximum 128 line lidar
  vector<orgtype> typess[128]; //maximum 128 line lidar
  float time_unit_scale;
  int lidar_type, point_filter_num, N_SCANS, SCAN_RATE, time_unit;// 雷达类型、采样间隔、扫描线数、扫描频率
  double blind;// 最小距离阈值(盲区)
  bool feature_enabled, given_offset_time;// 是否提取特征、是否进行时间偏移
  // ros::Publisher pub_full, pub_surf, pub_corn; // 发布全部点、发布平面点、发布边缘点

private:
  // 用于对Livox激光雷达数据进行处理
  void avia_handler(const livox_ros_driver2::msg::CustomMsg::UniquePtr &msg);
  void oust64_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg);
  void velodyne_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg);
  void mid360_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg);
  void default_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg);
  void give_feature(PointCloudXYZI &pl, vector<orgtype> &types);
  void pub_func(PointCloudXYZI &pl, const rclcpp::Time &ct);
  int  plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, uint &i_nex, Eigen::Vector3d &curr_direct);
  bool small_plane(const PointCloudXYZI &pl, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct);
  bool edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir);
  
  int group_size;// group_size: 一组用于拟合平面的点数
  double disA, disB, inf_bound;// disA, disB: 平面点群范围的计算参数  inf_bound:无穷远距离界限  disA, disB: 经验参数,用于控制平面点群范围
  double limit_maxmid, limit_midmin, limit_maxmin;// limit_maxmid等: 判断是否为平面的距离比阈值
  double p2l_ratio;// p2l_ratio: 平面点群长宽比阈值
  double jump_up_limit, jump_down_limit;// jump_up/down_limit: 角点角度余弦阈值
  double cos160;// cos160: 160度的角余弦值,用于判断交点
  double edgea, edgeb;// edgea/b: 跳变边缘判定参数
  double smallp_intersect, smallp_ratio;// smallp_xxx:小平面判定参数
  double vx, vy, vz;// vx/vy/vz: 二点向量,用于计算
};
