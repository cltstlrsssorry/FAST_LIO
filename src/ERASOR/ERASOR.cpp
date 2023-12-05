#include "ERASOR.h"

#include <omp.h>
#include <pcl/common/centroid.h>

ERASOR::ERASOR() {}

void ERASOR::clear_bin(Bin &bin)
{
  bin.max_h = -INF;
  bin.min_h = INF;
  bin.x = 0;
  bin.y = 0;
  bin.is_occupied = false;
  bin.status = NOT_ASSIGNED;
  if (!bin.points.empty())
    bin.points.clear();
}

void ERASOR::clear(PointCloudXYZI &pt_cloud)
{
  if (!pt_cloud.empty())
  {
    pt_cloud.clear();
  }
}

void ERASOR::setCenter(double x, double y, double z)
{
  center_x = x;
  center_y = y;
  center_z = z;
}

/**
 * @brief Inputs should be the transformed pointcloud!
 */
void ERASOR::set_inputs(const PointCloudXYZI &map_voi, const PointCloudXYZI &query_voi)
{
  clear(map_complement); // 函数用于清空map_complement变量。

  // 使用两个嵌套的循环，遍历cfg_.num_sectors_个扇区和cfg_.num_rings_个环。
  for (int theta = 0; theta < cfg_.num_sectors_; ++theta)
  {
    for (int r = 0; r < cfg_.num_rings_; ++r)
    {
      // 在每个扇区和环的循环中，分别调用clear_bin函数来清空r_pod_map[r][theta]、r_pod_curr[r][theta]和r_pod_selected[r][theta]变量。
      clear_bin(r_pod_map[r][theta]);
      clear_bin(r_pod_curr[r][theta]);
      clear_bin(r_pod_selected[r][theta]);
    }

  }

  // 调用voi2r_pod函数，将查询点云query_voi转换为r_pod_curr表示的极坐标形式。
  voi2r_pod(query_voi, r_pod_curr);

  // 调用voi2r_pod函数，将地图点云map_voi转换为r_pod_map表示的极坐标形式，并将补集存储在map_complement变量中。
  voi2r_pod(map_voi, r_pod_map, map_complement);

  int debug_total_num = 0;

  // 使用两个嵌套的循环，遍历cfg_.num_sectors_个扇区和cfg_.num_rings_个环。
  for (int theta = 0; theta < cfg_.num_sectors_; theta++)
  {
    for (int r = 0; r < cfg_.num_rings_; r++)
    {
      // 在每个扇区和环的循环中，获取r_pod_map[r][theta]的引用，并将其点云数量累加到debug_total_num变量中。
      Bin &bin_map = r_pod_map[r][theta];

      // 如果bin_map.points不为空，则将其点云数量累加到debug_total_num变量中。
      debug_total_num += bin_map.points.size();
    }
  }
}

void ERASOR::setConfig(Config &cfg)
{
  cfg_ = cfg;

  RCLCPP_INFO(rclcpp::get_logger("ERASOR"),"Set Config for ERASOR");

  // Initalization of R-POD
  init(r_pod_map);
  init(r_pod_curr);
  init(r_pod_selected);

  ring_size = cfg_.max_range_ / cfg_.num_rings_;
  sector_size = 2 * PI / cfg_.num_sectors_;

  // piecewise_ground_.reserve(130000);
  // non_ground_.reserve(130000);
  // ground_pc_.reserve(130000);
  // non_ground_pc_.reserve(130000);

  RCLCPP_INFO(rclcpp::get_logger("ERASOR"),"ERASOR is initialized");
}

// 记录点云中每个点的信息，包括点的坐标和强度值。
void ERASOR::pt2r_pod(const PointType &pt, Bin &bin)
{
  // bin.is_occupied被设置为true，表示该Bin对象被占用。
  bin.is_occupied = true;

  // pt被添加到bin.points中，将该点加入到Bin对象的点云集合中。
  bin.points.push_back(pt);

  // 如果(pt.z - center_z + cfg_.tf_z)大于等于bin.max_h，则更新bin.max_h、bin.x和bin.y的值。
  // 这表示如果点的高度超过了bin.max_h，则更新bin.max_h为更高的高度，并更新对应的x和y坐标。
  if ((pt.z - center_z + cfg_.tf_z) >= bin.max_h)
  {
    bin.max_h = pt.z - center_z + cfg_.tf_z;
    bin.x = pt.x;
    bin.y = pt.y;
  }
  if ((pt.z - center_z + cfg_.tf_z) <= bin.min_h)
  {
    bin.min_h = pt.z - center_z + cfg_.tf_z;
  }
}

// 计算theta
double ERASOR::xy2theta(const double &x, const double &y)
{ // 0 ~ 2 * PI

  if ((y - center_y) >= 0)
  {
    return atan2(y - center_y, x - center_x); // 1, 2 quadrant
  }
  else
  {
    return 2 * PI + atan2(y - center_y, x - center_x); // 3, 4 quadrant
  }
}

// 计算R
double ERASOR::xy2radius(const double &x, const double &y)
{
  return sqrt(pow(x - center_x, 2) + pow(y - center_y, 2));
}

// 将点云转换为极坐标形式
void ERASOR::voi2r_pod(const PointCloudXYZI &src, R_POD &r_pod)
{
  for (auto const &pt : src.points)
  {
    // 对于每个点，首先检查其高度是否在指定的范围内，即是否满足(pt.z - center_z + cfg_.tf_z) < cfg_.max_h_和(pt.z - center_z + cfg_.tf_z) > cfg_.min_h_。
    if ((pt.z - center_z + cfg_.tf_z) < cfg_.max_h_ && (pt.z - center_z + cfg_.tf_z) > cfg_.min_h_)
    {
      // 如果高度满足条件，则计算点到原点的距离r，通过调用xy2radius函数实现。
      double r = xy2radius(pt.x, pt.y);

      // 如果距离r小于等于指定的最大范围cfg_.max_range_，则继续进行下一步。
      if (r <= cfg_.max_range_)
      {
        // 计算点的极角theta，通过调用xy2theta函数实现。
        double theta = xy2theta(pt.x, pt.y);

        // 根据扇区大小和环的大小，将极角theta和距离r映射到对应的扇区索引sector_idx和环索引ring_idx。
        // theta是一个表示角度的变量。
        // sector_size是一个表示扇区大小的变量。
        //(theta / sector_size)表示将角度值除以扇区大小，得到一个浮点数结果。
        // static_cast<int>((theta / sector_size))将浮点数结果强制转换为整数类型。
        // cfg_.num_sectors_ - 1表示扇区的总数减去1，即最大的扇区索引。
        // 三元运算符的条件部分为(theta / sector_size)，如果条件为真，则返回条件成立时的值，即强制转换后的整数值；如果条件为假，则返回条件不成立时的值，即最大的扇区索引。
        // 最终的结果是计算得到的扇区索引，它的取值范围是从0到最大的扇区索引。
        int sector_idx = min(static_cast<int>((theta / sector_size)), cfg_.num_sectors_ - 1);
        int ring_idx = min(static_cast<int>((r / ring_size)), cfg_.num_rings_ - 1);

        // 调用pt2r_pod函数，将当前点的信息转换为极坐标形式，并存储到输出的r_pod中的对应环和扇区位置。
        pt2r_pod(pt, r_pod.at(ring_idx).at(sector_idx));
      }
    }
  }
}

// 将点云转换为极坐标形式，并将补集存储到complement变量中。
void ERASOR:: voi2r_pod(const PointCloudXYZI &src, R_POD &r_pod, PointCloudXYZI &complement)
{
  for (auto const &pt : src.points)
  {
    if ((pt.z - center_z + cfg_.tf_z) < cfg_.max_h_ && (pt.z - center_z + cfg_.tf_z) > cfg_.min_h_)
    { // range of z?
      double r = xy2radius(pt.x, pt.y);
      if (r <= cfg_.max_range_)
      {
        double theta = xy2theta(pt.x, pt.y);
        //        int sector_idx = min(static_cast<int>((theta / sector_size) +
        //        0.5), num_sectors - 1); int ring_idx = min(static_cast<int>((r
        //        / ring_size) + 0.5), num_rings - 1);
        int sector_idx = min(static_cast<int>((theta / sector_size)), cfg_.num_sectors_ - 1);
        int ring_idx = min(static_cast<int>((r / ring_size)), cfg_.num_rings_ - 1);
        pt2r_pod(pt, r_pod.at(ring_idx).at(sector_idx));
      }
      else
      {
        // 插入补集
        complement.points.push_back(pt);
      }
    }
    else
    {
      // 插入补集
      complement.points.push_back(pt);
    }
  }
}

// 比较地图点云和当前点云，并将地面点云和非地面点云分别添加到piecewise_ground_和non_ground_中。
void ERASOR::compare_vois_and_revert_ground_w_block()
{
  dynamic_viz.points.clear();
  ground_viz.points.clear();
#pragma omp parallel for
  // 使用两个嵌套的循环，遍历cfg_.num_sectors_个扇区和cfg_.num_rings_个环。
  for (int theta = 0; theta < cfg_.num_sectors_; theta++)
  {
    for (int r = 0; r < cfg_.num_rings_; r++)
    {
      // 在每个扇区和环的循环中，获取当前扇区和环的Bin对象，分别为bin_curr和bin_map。
      //  Min. num of pts criteria.
      Bin &bin_curr = r_pod_curr[r][theta];
      Bin &bin_map = r_pod_map[r][theta];

      // 如果bin_map.points为空，表示地图中该扇区和环没有点云数据，将对应的r_pod_selected[r][theta].status设置为LITTLE_NUM，并继续下一次循环。
      if (bin_map.points.empty())
      {
        //little 
        r_pod_selected[r][theta].status = LITTLE_NUM;
        continue;
      }

      // 如果bin_curr.points.size()小于cfg_.minimum_num_pts，表示当前扇区和环的点云数量小于指定的最小点数，
      if (bin_curr.points.size() < cfg_.minimum_num_pts)
      {
        // 将对应的r_pod_selected[r][theta].status设置为LITTLE_NUM。
        r_pod_selected[r][theta].status = LITTLE_NUM;
      }
      else // 计算地图点云和当前点云的高度差，并计算扫描比例scan_ratio。
      {
        double map_h_diff = bin_map.max_h - bin_map.min_h;
        double curr_h_diff = bin_curr.max_h - bin_curr.min_h;
        double scan_ratio = min(map_h_diff / curr_h_diff, curr_h_diff / map_h_diff);
        // ---------------------------------
        //          Scan Ratio Test
        // ---------------------------------
        // 如果bin_curr和bin_map都被占用（即bin_curr.is_occupied和bin_map.is_occupied为真），则进行扫描比例测试。
        if (bin_curr.is_occupied && bin_map.is_occupied)
        {
          // 如果scan_ratio小于cfg_.scan_ratio_threshold，表示扫描比例小于阈值，即地图点云和当前点云的高度差较小，可能是动态物体。
          if (scan_ratio < cfg_.scan_ratio_threshold)
          { // find dynamic!
            // 如果地图点云的高度差大于等于当前点云的高度差，表示地图点云高于当前点云，将对应的r_pod_selected[r][theta].status设置为MAP_IS_HIGHER。
            if (map_h_diff >= curr_h_diff)
            { // Occupied -> Disappear  <<BLUE>>
              r_pod_selected[r][theta].status = MAP_IS_HIGHER;//map is higher
            }
            // 如果地图点云的高度差小于等于当前点云的高度差，表示地图点云低于当前点云，将对应的r_pod_selected[r][theta].status设置为CURR_IS_HIGHER。
            else if (map_h_diff <= curr_h_diff)
            { // No objects exist -> Appear! <<GREEN>>
              r_pod_selected[r][theta].status = CURR_IS_HIGHER;//curr is higher
            }
          }
          else // 将对应的r_pod_selected[r][theta].status设置为MERGE_BINS，表示需要合并扇区和环。
          {
            r_pod_selected[r][theta].status = MERGE_BINS;//merge bins
          }
        }

        // 如果只有地图点云被占用（即bin_map.is_occupied为真），将对应的r_pod_selected[r][theta].status设置为LITTLE_NUM。
        else if (bin_map.is_occupied)
        { // Maybe redundant?
          r_pod_selected[r][theta].status = LITTLE_NUM;
        }
      }
    }
  }

  // 2. set bins!
  // 第一个循环结束后，进行第二个循环，再次遍历所有的扇区和环。
  for (int theta = 0; theta < cfg_.num_sectors_; theta++)
  {
    for (int r = 0; r < cfg_.num_rings_; r++)
    {
      // 在每个扇区和环的循环中，获取当前扇区和环的Bin对象，分别为bin_curr和bin_map。
      Bin &bin_curr = r_pod_curr[r][theta];
      Bin &bin_map = r_pod_map[r][theta];

      // 根据r_pod_selected[r][theta].status的值进行不同的操作
      double OCCUPANCY_STATUS = r_pod_selected[r][theta].status;

      // 如果OCCUPANCY_STATUS为LITTLE_NUM，将bin_map赋值给r_pod_selected[r][theta]，并将对应的status设置为LITTLE_NUM。
      if (OCCUPANCY_STATUS == LITTLE_NUM)
      {
        r_pod_selected[r][theta] = bin_map;
        r_pod_selected[r][theta].status = LITTLE_NUM;
      }

      // 如果OCCUPANCY_STATUS为MAP_IS_HIGHER，
      //
      // 同时，提取地面点云和非地面点云，并将它们分别添加到piecewise_ground_和non_ground_中，
      // 并将地面点云添加到ground_viz中，非地面点云添加到dynamic_viz中。
      else if (OCCUPANCY_STATUS == MAP_IS_HIGHER)
      {
        // 判断地图点云的高度差是否大于0.5，如果是，则将bin_curr赋值给r_pod_selected[r][theta]，并将对应的status设置为MAP_IS_HIGHER。
        if ((bin_map.max_h - bin_map.min_h) > 0.5)
        {
          r_pod_selected[r][theta] = bin_curr;
          r_pod_selected[r][theta].status = MAP_IS_HIGHER;
          // ---------------------------------
          //     NOTE: Ground is retrieved!
          // ---------------------------------

          if (!piecewise_ground_.empty())
            piecewise_ground_.clear(); // 清空piecewise_ground_变量，地面点
          if (!non_ground_.empty())
            non_ground_.clear(); // 清空non_ground_变量，非地面点

          extract_ground(bin_map.points, piecewise_ground_, non_ground_);
          // no need for this, or you can insert the highest z and lowest z in piecewise_ground_
          // r_pod_selected[r][theta].points += piecewise_ground_;
          ground_viz += piecewise_ground_;
          dynamic_viz += non_ground_;

        }
        else
        {
          r_pod_selected[r][theta] = bin_map;
          r_pod_selected[r][theta].status = NOT_ASSIGNED;
        }
        
      }
      else if (OCCUPANCY_STATUS == CURR_IS_HIGHER)
      {
        r_pod_selected[r][theta] = bin_map;
        r_pod_selected[r][theta].status = CURR_IS_HIGHER;
      }
      else if (OCCUPANCY_STATUS == MERGE_BINS)
      {
        if (is_dynamic_obj_close(r_pod_selected, r, theta))
        {
          r_pod_selected[r][theta] = bin_map;
          r_pod_selected[r][theta].status = BLOCKED;
        }
        else
        {
          // NOTE the dynamic object comes ....:(
          r_pod_selected[r][theta] = bin_map;
          r_pod_selected[r][theta].status = MERGE_BINS;
        }
      }
    }
  }
}

bool point_cmp(PointType a, PointType b) { return a.z < b.z; }

// 将src点云复制到dst中，并将dst中的点云坐标加上tf_x、tf_y和tf_z。
void copy_PointCloud2World(PointCloudXYZI &src, PointCloudXYZI &dst, double tf_x, double tf_y, double tf_z)
{
  dst.points.reserve(src.points.size());

  for (auto &p : src.points)
  {
    p.x = p.x + tf_x;
    p.y = p.y + tf_y;
    p.z = p.z + tf_z;
    dst.points.push_back(p);
  }
}

// 提取地面点云和非地面点云
void ERASOR::extract_ground(PointCloudXYZI &src, PointCloudXYZI &dst, PointCloudXYZI &outliers)
{
  // 清空dst和outliers变量
  if (!dst.empty())
    dst.clear();
  if (!outliers.empty())
    outliers.clear();

  // 将src点云复制到src_copy中，并将src_copy中的点云坐标减去cfg_.tf_x、cfg_.tf_y和cfg_.tf_z。
  PointCloudXYZI src_copy;
  copy_PointCloud2World(src, src_copy, cfg_.tf_x - center_x, cfg_.tf_y - center_y, cfg_.tf_z - center_z);

  std::sort(src_copy.points.begin(), src_copy.points.end(), point_cmp);

  // 1. remove_outliers;
  auto it = src_copy.points.begin();

  // 从src_copy中删除高度小于cfg_.min_h_的点云
  for (int i = 0; i < src_copy.points.size(); i++)
  {
    if (src_copy.points[i].z < cfg_.min_h_)
      it++;
    else
      break;
  }
  src_copy.points.erase(src_copy.points.begin(), it); // 将从容器的起始位置到迭代器it之前的所有点都从容器中删除。

  // 2. set seeds!
  if (!ground_pc_.empty())
    ground_pc_.clear();
  if (!non_ground_pc_.empty())
    non_ground_pc_.clear();

  extract_initial_seeds_(src_copy, ground_pc_);

  // 3. Extract ground
  // 循环从0到cfg_.iter_groundfilter_进行迭代，表示地面提取的迭代次数。
  for (int i = 0; i < cfg_.iter_groundfilter_; i++)
  {
    // 在每次迭代开始时，调用estimate_plane_函数对ground_pc_进行地面平面模型的估计。
    estimate_plane_(ground_pc_);
    // 清空ground_pc_，用于存储当前迭代中被划分为地面点的点云数据。
    ground_pc_.clear();

    // pointcloud to matrix
    // 将输入点云数据src转换为一个矩阵points，其中每一行表示一个点的坐标。
    Eigen::MatrixXf points(src.points.size(), 3);
    int j = 0;

    // 将输入点云数据src转换为一个矩阵points，其中每一行表示一个点的坐标。
    for (auto p : src.points)
    {
      points.row(j++) << p.x, p.y, p.z;
    }

    // ground plane model
    // 使用地面平面模型的法向量normal_对点云数据进行点乘运算，得到每个点到地面平面的距离。
    Eigen::VectorXf result = points * normal_;

    // threshold filter
    // 根据距离阈值th_dist_d_，将点划分为地面点和非地面点，并分别存储在ground_pc_和non_ground_pc_中。
    for (int r = 0; r < result.rows(); r++)
    {
      if (result[r] < th_dist_d_)
      {
        ground_pc_.points.push_back(src[r]);
      }
      else
      {
        // 如果当前迭代是最后一次迭代（即i等于cfg_.iter_groundfilter_ - 1），则将非地面点存储在non_ground_pc_中。
        if (i == (cfg_.iter_groundfilter_ - 1))
        { // Last iteration
          non_ground_pc_.points.push_back(src[r]);
        }
      }
    }
  }

  // change src_copy based on the tf and center
  // ground_pc_和non_ground_pc_分别是存储地面点云数据和非地面点云数据的对象。
  // dst和outliers是存储转换后的点云数据的对象。
  // center_x、center_y和center_z分别是点云数据的中心点的x、y和z坐标。
  // cfg_.tf_x、cfg_.tf_y和cfg_.tf_z分别是坐标转换的偏移量。
  // copy_PointCloud2World函数用于将点云数据进行坐标转换，将其相对于中心点的坐标进行调整。
  // ground_pc_和non_ground_pc_分别通过减去偏移量cfg_.tf_x、cfg_.tf_y和cfg_.tf_z来进行坐标转换。
  // 转换后的地面点云数据存储在dst中，转换后的非地面点云数据存储在outliers中。
  copy_PointCloud2World(ground_pc_, dst, center_x - cfg_.tf_x, center_y - cfg_.tf_y, center_z - cfg_.tf_z);
  copy_PointCloud2World(non_ground_pc_, outliers, center_x - cfg_.tf_x, center_y - cfg_.tf_y, center_z - cfg_.tf_z);
}


//将sc中占用的格子的点放入pc中
void ERASOR::r_pod2pc(const R_POD &sc, PointCloudXYZI &pc)
{

  pc.points.clear();

  for (int theta = 0; theta < cfg_.num_sectors_; theta++)
  {
    for (int r = 0; r < cfg_.num_rings_; r++)
    {
      if (sc.at(r).at(theta).is_occupied)
      {
        for (auto const &pt : sc.at(r).at(theta).points)
        {
          pc.points.push_back(pt);
        }
      }
    }
  }
}

void ERASOR::get_static_estimate(PointCloudXYZI &arranged, PointCloudXYZI &dynamic_pts, PointCloudXYZI &complement)
{
  // pcl::PointCloud<PointT> arranged;
  r_pod2pc(r_pod_selected, arranged);
  arranged += ground_viz;
  if (cfg_.replace_intensity)
  {
    // replace intensity in arranged
    for (auto &pt : arranged.points)
    {
      pt.intensity = 0;
    }
    for (auto &pt : dynamic_viz.points)
    {
      pt.intensity = 1;
    }
    dynamic_pts += dynamic_viz;
    // replace intensity in arranged
    for (auto &pt : map_complement.points)
    {
      pt.intensity = 0;
    }
  }
  complement = map_complement;
}

// 初始化R-POD
void ERASOR::init(R_POD &r_pod)
{
  if (!r_pod.empty())
  {
    r_pod.clear();
  }
  Ring ring;
  Bin bin = {-INF, INF, 0, 0, false, static_cast<bool>(NOT_ASSIGNED)};
  bin.points.reserve(ENOUGH_NUM);
  for (int i = 0; i < cfg_.num_sectors_; i++)
  {
    ring.emplace_back(bin);
  }
  for (int j = 0; j < cfg_.num_rings_; j++)
  {
    r_pod.emplace_back(ring);
  }
}

bool ERASOR::is_dynamic_obj_close(R_POD &r_pod_selected, int r_target, int theta_target)
{
  // Set thetas
  std::vector<int> theta_candidates;
  for (int j = theta_target - 1; j <= theta_target + 1; j++)
  {
    if (j < 0)
    {
      theta_candidates.push_back(j + cfg_.num_rings_);
    }
    else if (j >= cfg_.num_sectors_)
    {
      theta_candidates.push_back(j - cfg_.num_rings_);
    }
    else
    {
      theta_candidates.push_back(j);
    }
  }
  for (int r = std::max(0, r_target - 1);
       r <= std::min(r_target + 1, cfg_.num_rings_ - 1); r++)
  {
    for (const auto &theta : theta_candidates)
    {
      if ((r == r_target) && (theta == theta_target))
        continue;

      if (r_pod_selected[r][theta].status ==
          CURR_IS_HIGHER)
      { // Dynamic object is near
        return true;
      }
    }
  }
  return false;
}


// 根据输入的地面点云数据估计地面平面模型，并计算相关的参数。
// 它通过计算协方差矩阵、奇异值分解和平面方程的计算，得到地面平面的法向量和偏移量。这些参数将在后续的地面提取过程中使用。
void ERASOR::estimate_plane_(const PointCloudXYZI &ground)
{
  // 首先，定义一个3x3的矩阵cov和一个4维向量pc_mean，用于计算地面点云数据的协方差矩阵和平均值。
  Eigen::Matrix3f cov;
  Eigen::Vector4f pc_mean;

  // 计算地面点云数据的协方差矩阵和平均值。
  pcl::computeMeanAndCovarianceMatrix(ground, cov, pc_mean);

  // 使用奇异值分解（Singular Value Decomposition，SVD）对协方差矩阵进行分解，得到特征向量矩阵。
  //  Singular Value Decomposition: SVD
  Eigen::JacobiSVD<Eigen::MatrixXf> svd(cov, Eigen::DecompositionOptions::ComputeFullU);
  // use the least singular vector as normal

  // 选择最小奇异值对应的特征向量作为地面平面的法向量，存储在normal_中。
  normal_ = (svd.matrixU().col(2));
  // mean ground seeds value

  // 提取平均地面种子点的坐标，存储在seeds_mean中。
  Eigen::Vector3f seeds_mean = pc_mean.head<3>();

  // 根据法向量和平均地面种子点的坐标，计算地面平面的偏移量d_，即平面方程为normal_.T * [x, y, z] = -d。
  //  according to normal.T*[x,y,z] = -d
  d_ = -(normal_.transpose() * seeds_mean)(0, 0);

  // 设置距离阈值th_dist_d_为配置参数cfg_.th_dist_减去偏移量d_。
  //  set distance threhold to `th_dist - d`
  th_dist_d_ = cfg_.th_dist_ - d_;

  // 函数执行完毕，地面平面模型的法向量和相关参数被存储在类成员变量normal_、d_和th_dist_d_中。
}

// 提取初始种子点
void ERASOR::extract_initial_seeds_(const PointCloudXYZI &p_sorted, PointCloudXYZI &init_seeds)
{
  // 清空init_seeds和g_seeds_pc，以确保它们为空。
  init_seeds.points.clear();
  PointCloudXYZI g_seeds_pc;

  // LPR is the mean of low point representative
  double sum = 0;
  int cnt = 0;

  // Calculate the mean height value.

  for (int i = cfg_.num_lowest_pts; i < p_sorted.points.size() && cnt < cfg_.num_lprs_; i++)
  {
    sum += (p_sorted.points[i].z - center_z + cfg_.tf_z);
    cnt++;
  }

  // 计算LPR的平均高度值lpr_height，即将高度值之和除以点的数量。
  double lpr_height = cnt != 0 ? sum / cnt : 0; // in case divide by 0

  // 清空g_seeds_pc，用于存储过滤后的种子点。
  g_seeds_pc.clear();

  // 使用循环遍历所有的点，将高度小于lpr_height + cfg_.th_seeds_heights_的点添加到g_seeds_pc中。
  //  iterate pointcloud, filter those height is less than lpr.height+th_seeds_
  for (int i = 0; i < p_sorted.points.size(); i++)
  {
    if ((p_sorted.points[i].z - center_z + cfg_.tf_z) < lpr_height + cfg_.th_seeds_heights_)
    {
      g_seeds_pc.points.push_back(p_sorted.points[i]);
    }
  }

  // 将g_seeds_pc赋值给init_seeds，完成初始种子点的提取。
  init_seeds = g_seeds_pc;

}
