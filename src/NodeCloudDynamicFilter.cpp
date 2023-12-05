#include "NodeCloudDynamicFilter.h"

NodeCloudDynamicFilter::NodeCloudDynamicFilter(const std::string & name):Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
{
    RCLCPP_INFO(this->get_logger(), "----NodeCloudDynomicFilter----");

    //map_updater

    std::deque<LocalPointLists> featsFromMap_list;

    declare_and_get_parameter();//创建MapUpdater对象，并读取配置文件

    erasor.setConfig(cfg);

    // reset
    map_static_estimate_.reset(new PointCloudXYZI());
    map_egocentric_complement_.reset(new PointCloudXYZI());
    map_staticAdynamic.reset(new PointCloudXYZI());
    map_filtered_.reset(new PointCloudXYZI());
    map_arranged_global_.reset(new PointCloudXYZI());
    map_arranged_complement_.reset(new PointCloudXYZI());


    // 创建先验地图
    raw_map.reset(new PointCloudXYZI()); // 创建点云对象


    node_cloud_dynamic_filter_publicher = this->create_publisher<sensor_msgs::msg::PointCloud2>("/node_dynamic_filter", 10);

    auto timer1 = std::chrono::milliseconds(static_cast<int64_t>(1000.0/100.0));
    timer_cre1=rclcpp::create_timer(this, this->get_clock(), timer1, std::bind(&NodeCloudDynamicFilter::timer1_callback, this));

    auto timer2 = std::chrono::milliseconds(static_cast<int64_t>(1000.0));
    timer_cre2=rclcpp::create_timer(this, this->get_clock(), timer2, std::bind(&NodeCloudDynamicFilter::timer2_callback, this));

    RCLCPP_INFO(this->get_logger(), "----NodeCloudDynomicFilter init finished.----"); // 输出信息，初始化结束

}


void NodeCloudDynamicFilter::timer1_callback()
{
    if(raw_map->points.size()==0) return;

    PointCloudXYZI::Ptr single_pc = featsFromMap_list.front().cloudpoint;

    lidar_end_time = featsFromMap_list.front().time;

    PointCloudXYZI::Ptr filter_pc(new PointCloudXYZI());

    VoxelPointCloud(single_pc, filter_pc, cfg.query_voxel_size_); // 下体素采样
    // read pose in VIEWPOINT Field in pcd

    // single_pc 点云对象的 sensor_origin_ 属性中获取传感器的原点坐标，并将其分别赋值给 x_curr，y_curr 和 z_curr。
    // 这里的 sensor_origin_ 是一个包含 X，Y，Z 坐标的四元素向量，代表了传感器的原点在世界坐标系中的位置。
    float x_curr = single_pc->sensor_origin_[0];
    float y_curr = single_pc->sensor_origin_[1];
    float z_curr = single_pc->sensor_origin_[2];

    if (cfg.is_large_scale_) reassign_submap(x_curr, y_curr);

    fetch_VoI(x_curr, y_curr, *filter_pc); // query_voi_ and map_voi_ are ready in the same world frame

    erasor.setCenter(x_curr, y_curr, z_curr);

    erasor.set_inputs(*map_voi_, *query_voi_);

    erasor.compare_vois_and_revert_ground_w_block();

    erasor.get_static_estimate(*map_static_estimate_, *map_staticAdynamic, *map_egocentric_complement_);

    if(cfg.verbose_) RCLCPP_INFO(this->get_logger(),"Static pts num: %ld ", map_static_estimate_->size());
    
    *map_arranged_ = *map_static_estimate_ + *map_outskirts_ + *map_egocentric_complement_;

    RCLCPP_INFO(this->get_logger(), "point size %ld", map_arranged_->points.size()); // 输出信息，初始化结束

    featsFromMap_list.pop_front();

}


void NodeCloudDynamicFilter::timer2_callback()
{
    setRawMap(featsFromMap);

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*map_arranged_, laserCloudmsg);

    // laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = "camera_init";
    node_cloud_dynamic_filter_publicher->publish(laserCloudmsg);

}

// 读取yaml文件，并配置参数，封装再私有变量中，供其ERASOR类使用
void NodeCloudDynamicFilter::declare_and_get_parameter()
{
    // MapUpdater
    this->declare_parameter<double>("MapUpdater.query_voxel_size", 0.1);
    this->declare_parameter<double>("MapUpdater.map_voxel_size", 0.1);
    this->declare_parameter<int>("MapUpdater.voxelization_interval", 10);
    this->declare_parameter<int>("MapUpdater.removal_interval", 5);

    // erasor
    this->declare_parameter<double>("erasor.max_range", 80.0);
    this->declare_parameter<int>("erasor.num_rings", 20);
    this->declare_parameter<int>("erasor.num_sectors", 108);
    this->declare_parameter<double>("erasor.min_h", -0.5);
    this->declare_parameter<double>("erasor.max_h", 3.1);
    this->declare_parameter<double>("erasor.scan_ratio_threshold", 0.20);
    this->declare_parameter<int>("erasor.minimum_num_pts", 6);

    this->declare_parameter<double>("erasor.gf_dist_thr", 0.125);
    this->declare_parameter<int>("erasor.gf_iter", 3);
    this->declare_parameter<int>("erasor.gf_num_lpr", 10);
    this->declare_parameter<double>("erasor.gf_th_seeds_height", 0.5);
    this->declare_parameter<int>("erasor.num_lowest_pts", 1);
    // verbose
    this->declare_parameter<bool>("verbose", false);

    
    this->get_parameter_or<double>("MapUpdater.query_voxel_size", cfg.query_voxel_size_, 0.1);
    this->get_parameter_or<double>("MapUpdater.map_voxel_size", cfg.map_voxel_size_, 0.1);
    this->get_parameter_or<int>("MapUpdater.voxelization_interval", cfg.global_voxelization_period_, 10);
    this->get_parameter_or<int>("MapUpdater.removal_interval", cfg.removal_interval_, 5);

    this->get_parameter_or<double>("erasor.max_range", cfg.max_range_, 80.0);
    this->get_parameter_or<int>("erasor.num_rings", cfg.num_rings_, 20);
    this->get_parameter_or<int>("erasor.num_sectors", cfg.num_sectors_, 108);
    this->get_parameter_or<double>("erasor.min_h", cfg.min_h_, -0.5);
    this->get_parameter_or<double>("erasor.max_h", cfg.max_h_, 3.1);
    this->get_parameter_or<double>("erasor.scan_ratio_threshold", cfg.scan_ratio_threshold, 0.20);
    this->get_parameter_or<int>("erasor.minimum_num_pts", cfg.minimum_num_pts, 6);

    this->get_parameter_or<double>("erasor.gf_dist_thr", cfg.th_dist_, 0.125);
    this->get_parameter_or<int>("erasor.gf_iter", cfg.iter_groundfilter_, 3);
    this->get_parameter_or<int>("erasor.gf_num_lpr", cfg.num_lprs_, 10);
    this->get_parameter_or<double>("erasor.gf_th_seeds_height", cfg.th_seeds_heights_, 0.5);
    this->get_parameter_or<int>("erasor.num_lowest_pts", cfg.num_lowest_pts, 1);

    this->get_parameter_or<bool>("verbose", cfg.verbose_, false);
}


// 设置原始地图
void NodeCloudDynamicFilter::setRawMap(PointCloudXYZI::Ptr const &raw_map)
{
    // copy raw map to map_arranged
    map_arranged_.reset(new PointCloudXYZI());
    VoxelPointCloud(raw_map, map_arranged_, cfg.map_voxel_size_); // 下体素采样
    num_pcs_init_ = map_arranged_->points.size(); // num_pcs_init_变量被赋值为map_arranged_->points.size()，表示初始地图中的点云数量。
    if (cfg.is_large_scale_)
    {
        map_arranged_global_->reserve(NUM_PTS_LARGE_ENOUGH_FOR_MAP);
        *map_arranged_global_ = *map_arranged_;

        RCLCPP_INFO(this->get_logger(), "Large-scale mode is on!");
        RCLCPP_INFO(this->get_logger(), "Submap size is: %f", submap_size_);
    }
}


// 下体素采样
void NodeCloudDynamicFilter::VoxelPointCloud(const PointCloudXYZI::Ptr &cloud, PointCloudXYZI::Ptr &cloud_voxelized, const double voxel_size)
{
    if (voxel_size <= 0.001)
    {
        *cloud_voxelized = *cloud;
        RCLCPP_INFO(this->get_logger(), "Voxel size is too small, no need to voxel grid filter!");
        return;
    }

    pcl::VoxelGrid<PointType> voxel_grid;
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setLeafSize(voxel_size, voxel_size, voxel_size);
    voxel_grid.filter(*cloud_voxelized);
}


const Config NodeCloudDynamicFilter::getCfg() 
{
    return cfg; 
}


// 重新分配子图
void NodeCloudDynamicFilter::reassign_submap(double pose_x, double pose_y)
{
    // 如果子图尚未初始化，则调用set_submap()函数，将map_arranged_global_中的点云数据分配到map_arranged_和map_arranged_complement_中。
    if (is_submap_not_initialized_)
    {
        set_submap(*map_arranged_global_, *map_arranged_, *map_arranged_complement_, pose_x, pose_y, submap_size_);
        submap_center_x_ = pose_x;
        submap_center_y_ = pose_y;
        is_submap_not_initialized_ = false;
    }

    // 否则，如果子图已经初始化，则计算当前位置与上一次位置的差值，如果差值大于子图大小的一半，则重新分配子图。
    else
    {
        double diff_x = abs(submap_center_x_ - pose_x); // 计算当前位置与上一次位置的差值
        double diff_y = abs(submap_center_y_ - pose_y); // 用于判断是否需要重新分配子地图
        static double half_size = submap_size_ / 2.0;

        // 如果差值大于子图大小的一半，则重新分配子图
        if ((diff_x > half_size) || (diff_y > half_size))
        {
            // Reassign submap
            map_arranged_global_.reset(new PointCloudXYZI());
            map_arranged_global_->reserve(num_pcs_init_);
            *map_arranged_global_ = *map_arranged_ + *map_arranged_complement_;

            set_submap(*map_arranged_global_, *map_arranged_, *map_arranged_complement_, pose_x, pose_y, submap_size_);
            submap_center_x_ = pose_x;
            submap_center_y_ = pose_y;
        }
    }
}


// 设置子图
void NodeCloudDynamicFilter::set_submap(const PointCloudXYZI &map_global,PointCloudXYZI &submap,PointCloudXYZI &submap_complement,
                                        double x, double y, double submap_size)
{
    // 清空子图和子图补集
    submap.clear();
    submap.reserve(NUM_PTS_LARGE_ENOUGH_FOR_MAP);
    submap_complement.clear();
    submap_complement.reserve(NUM_PTS_LARGE_ENOUGH_FOR_MAP);

    // 遍历全局地图中的每个点，计算其与当前位置的差值，如果差值小于子图大小的一半，则将该点添加到子图中；否则，将该点添加到子图补集中。
    for (const auto pt : map_global.points)
    {
        double diff_x = fabs(x - pt.x);
        double diff_y = fabs(y - pt.y);

        if ((diff_x < submap_size) && (diff_y < submap_size))
        {
            submap.points.emplace_back(pt);
        }
        else
        {
            submap_complement.points.emplace_back(pt);
        }
    }
}

// 它用于从给定的点云数据中提取感兴趣区域（VoI）。
void NodeCloudDynamicFilter::fetch_VoI(double x_criterion, double y_criterion, PointCloudXYZI &query_pcd)
{
    query_voi_.reset(new PointCloudXYZI());
    map_voi_.reset(new PointCloudXYZI());
    map_outskirts_.reset(new PointCloudXYZI());

    if (cfg.mode == "naive")
    {
        double max_dist_square = pow(cfg.max_range_, 2); // 计算最大距离的平方值max_dist_square，用于筛选在VoI范围内的点。
        // find query voi

        // 遍历查询点云数据中的每个点，计算其与VoI中心坐标的距离的平方值。如果距离小于max_dist_square，则将该点添加到query_voi_中。
        for (auto const &pt : query_pcd.points)
        {
            double dist_square = pow(pt.x - x_criterion, 2) + pow(pt.y - y_criterion, 2);

            if (dist_square < max_dist_square) query_voi_->points.emplace_back(pt);

        }

        // find map voi
        ////遍历地图点云数据中的每个点，同样计算其与VoI中心坐标的距离的平方值。如果距离小于max_dist_square，则将该点添加到map_voi_中；
        // 否则，如果配置文件中设置了替换强度（cfg_.replace_intensity），则将该点的强度设置为0，并将其添加到map_outskirts_中。
        for (auto &pt : map_arranged_->points)
        {
            double dist_square = pow(pt.x - x_criterion, 2) + pow(pt.y - y_criterion, 2);
            if (dist_square < max_dist_square)
            {
                map_voi_->points.emplace_back(pt);
            }
            else
            {
                if (cfg.replace_intensity) pt.intensity = 0;

                map_outskirts_->points.emplace_back(pt);
            }
        }
    }

    if(cfg.verbose_) RCLCPP_INFO(this->get_logger()," %ld points in the map", map_arranged_->points.size());

}