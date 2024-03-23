#include "NodeOctomap.h"

NodeOctomap::NodeOctomap(const std::string &name) : Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
{

    RCLCPP_INFO(this->get_logger(), "----NodeOctomap----");

    setConfig();

    count = 1;

    publish_octomap = this->create_publisher<octomap_msgs::msg::Octomap>("publish_octomap", 10);

    m_octree = new octomap::OcTree(config.resolution);
    // 创建了一个新的 OcTree 对象，并将其分辨率设置为 config.resolution。这个对象的指针被存储在 m_octree 中。
    // m_octree->setResolution(config.resolution);

    // 当传感器检测到障碍物时，该网格被认为是占用的概率。
    m_octree->setProbHit(config.probHit);

    // 当传感器没有检测到障碍物时，该网格被认为是空闲的概率。
    m_octree->setProbMiss(config.probMiss);

    // 设置了一个网格被认为是空闲的概率阈值。如果一个网格的占用概率低于这个值，那么它就被认为是空闲的。
    m_octree->setClampingThresMin(config.thresMin);

    // 设置了一个网格被认为是占用的概率阈值。如果一个网格的占用概率高于这个值，那么它就被认为是占用的。
    m_octree->setClampingThresMax(config.thresMax);

    m_octree->setOccupancyThres(config.occupancyThres);

    m_maxTreeDepth = m_treeDepth;
    
    ground_pts.reset(new PointCloudXYZI);
    noise_cloud.reset(new PointCloudXYZI);
    raw_map_ptr_.reset(new PointCloudXYZI);

    auto timer = std::chrono::milliseconds(static_cast<int64_t>(1000.0 / 100.0));
    timer_cre = rclcpp::create_timer(this, this->get_clock(), timer, std::bind(&NodeOctomap::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "----NodeOctomap init finished.----"); // 输出信息，初始化结束
}

void NodeOctomap::timer_callback()
{
    if (!wait_octomap_points_list.empty())
    {
        updateOccupancy();
        if (count % 10 == 0)
        {
            publishOctomap();
            count = 0;
        }
        count++;
    }
}

void NodeOctomap::updateOccupancy()
{
    PointCloudXYZI::Ptr single_pc = wait_octomap_points_list.begin()->points;

    timestamp = wait_octomap_points_list.begin()->time;

    wait_octomap_points_list.pop_front();

    if (single_pc->empty())
        return;

    if (single_pc->points.size() < 50)
        return;

    Eigen::Vector4f sensor_pos = single_pc->sensor_origin_;

    octomap::point3d sensor_origin(sensor_pos.x(), sensor_pos.y(), sensor_pos.z());

    if (!m_octree->coordToKeyChecked(sensor_origin, m_updateBBXMin) || !m_octree->coordToKeyChecked(sensor_origin, m_updateBBXMax))
    {
        RCLCPP_INFO(this->get_logger(), "Could not generate Key for origin: %f, %f, %f", sensor_origin.x(), sensor_origin.y(), sensor_origin.z());
    }

    // instead of direct scan insertion, compute update to filter ground:
    octomap::KeySet free_cells, occupied_cells;

    for (PointCloudXYZI::const_iterator it = single_pc->begin(); it != single_pc->end(); ++it)
    {
        octomap::point3d point(it->x, it->y, it->z);

        // range filtering
        // (point - sensor_origin).norm() L2范数 计算距离
        if ((config.minRange > 0.0) && ((point - sensor_origin).norm() < config.minRange))
            continue;

        // 如果点到传感器原点的距离小于最大范围，则处理此点。
        if ((config.maxRange < 0.0) || ((point - sensor_origin).norm() <= config.maxRange))
        {
            // free cells  计算从传感器原点到点的光线路径上的所有八叉树节点的键
            if (m_octree->computeRayKeys(sensor_origin, point, m_keyRay))
            {
                // 将这些键插入到free_cells集合中。
                free_cells.insert(m_keyRay.begin(), m_keyRay.end());
            }

            // occupied endpoint
            octomap::OcTreeKey key;

            // 尝试将点的坐标转换为八叉树的键key
            if (m_octree->coordToKeyChecked(point, key))
            {
                // 如果转换成功，则将key插入到occupied_cells集合中，并更新边界框的最小键m_updateBBXMin和最大键m_updateBBXMax。
                occupied_cells.insert(key);
                updateMinKey(key, m_updateBBXMin);
                updateMaxKey(key, m_updateBBXMax);
            }
        }
        else
        {
            continue;
            
            octomap::point3d new_end = sensor_origin + (point - sensor_origin).normalized() * config.maxRange;

            if (m_octree->computeRayKeys(sensor_origin, new_end, m_keyRay))
            {
                free_cells.insert(m_keyRay.begin(), m_keyRay.end());

                octomap::OcTreeKey endKey;
                if (m_octree->coordToKeyChecked(new_end, endKey))
                {
                    free_cells.insert(endKey);
                    updateMinKey(endKey, m_updateBBXMin);
                    updateMaxKey(endKey, m_updateBBXMax);
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "Could not generate Key for endpoint ");
                }
            }
        }
    }

    for (octomap::KeySet::iterator it = free_cells.begin(), end = free_cells.end(); it != end; ++it)
    {
        m_octree->updateNode(*it, false);
    }

    for (octomap::KeySet::iterator it = occupied_cells.begin(), end = occupied_cells.end(); it != end; ++it)
    {
        m_octree->updateNode(*it, true);
    }

    if (config.m_prune)
        m_octree->prune();
    
}

void NodeOctomap::publishOctomap()
{
    octomap_msgs::msg::Octomap octoMsg;
    octomap_msgs::binaryMapToMsg(*m_octree, octoMsg);

    // laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    octoMsg.header.stamp = get_ros_time(timestamp);
    octoMsg.header.frame_id = "camera_init";
    publish_octomap->publish(octoMsg);
}

void NodeOctomap::setConfig()
{
    this->declare_parameter<float>("Octomap.resolution", 0.1);

    this->declare_parameter<float>("Octomap.maxRange", -1.0);
    this->declare_parameter<float>("Octomap.minRange", -1.0);

    this->declare_parameter<float>("Octomap.probHit", 0.7);
    this->declare_parameter<float>("Octomap.probMiss", 0.4);
    this->declare_parameter<float>("Octomap.thresMin", 0.12);
    this->declare_parameter<float>("Octomap.thresMax", 0.97);

    this->declare_parameter<float>("Octomap.occupancyThres", 0.7);

    this->declare_parameter<bool>("Octomap.m_prune", true);
    this->declare_parameter<bool>("verbose", false);

    this->declare_parameter<bool>("Octomap.filterGroundPlane", true);
    this->declare_parameter<float>("Octomap.m_groundFilterDistance", 0.04);
    this->declare_parameter<float>("Octomap.m_groundFilterAngle", 0.15);
    this->declare_parameter<float>("Octomap.m_groundFilterPlaneDistance", 0.15);

    this->declare_parameter<bool>("Octomap.filterNoise", false);
    this->declare_parameter<float>("Octomap.StddevMulThresh", 1.0);
    this->declare_parameter<int>("Octomap.filterMeanK", 50); // pcl::StatisticalOutlierRemoval

    this->get_parameter_or<double>("Octomap.resolution", config.resolution, 0.1);
    this->get_parameter_or<float>("Octomap.maxRange", config.maxRange, -1.0);
    this->get_parameter_or<float>("Octomap.minRange", config.minRange, -1.0);

    this->get_parameter_or<float>("Octomap.probHit", config.probHit, 0.7);
    this->get_parameter_or<float>("Octomap.probMiss", config.probMiss, 0.4);
    this->get_parameter_or<float>("Octomap.thresMin", config.thresMin, 0.12);
    this->get_parameter_or<float>("Octomap.thresMax", config.thresMax, 0.97);

    this->get_parameter_or<float>("Octomap.occupancyThres", config.occupancyThres, 0.7);

    this->get_parameter_or<bool>("Octomap.m_prune", config.m_prune, true);
    this->get_parameter_or<bool>("verbose", config.verbose, false);

    this->get_parameter_or<bool>("Octomap.filterGroundPlane", config.filterGroundPlane, true);
    if (config.filterGroundPlane)
    {
        this->get_parameter_or<float>("octomap.m_groundFilterDistance", config.m_groundFilterDistance, 0.04);
        this->get_parameter_or<float>("octomap.m_groundFilterAngle", config.m_groundFilterAngle, 0.15);
        this->get_parameter_or<float>("octomap.m_groundFilterPlaneDistance", config.m_groundFilterPlaneDistance, 0.15);
    }

    this->get_parameter_or<bool>("octomap.filterNoise", config.filterNoise, false);
    if (config.filterNoise)
    {
        this->get_parameter_or<float>("octomap.StddevMulThresh", config.StddevMulThresh, 1.0);
        this->get_parameter_or<int>("octomap.filterMeanK", config.filterMeanK, 50);
    }
}

void NodeOctomap::filterGroundPlane(PointCloudXYZI::Ptr const &pc, PointCloudXYZI::Ptr &ground, PointCloudXYZI::Ptr &nonground)
{
    if (pc->size() < 50)
    {
        RCLCPP_INFO(this->get_logger(), "Pointcloud in OctomapServer too small, skipping ground plane extraction");
        nonground = pc;
        return;
    }

    // plane detection for ground plane removal:
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients); // 创建一个新的 ModelCoefficients 对象，用于存储模型的系数。
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);                // 创建一个新的 PointIndices 对象，用于存储内点的索引。

    // Create the segmentation object and set up:
    pcl::SACSegmentation<PointType> seg; // 创建一个新的 SACSegmentation 对象，用于进行分割。
    seg.setOptimizeCoefficients(true);   // 设置优化系数为 true，这将使得算法尝试优化模型的系数。

    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);     // 设置模型类型为 SACMODEL_PERPENDICULAR_PLANE，这表示我们想要检测的是垂直于某个方向的平面。
    seg.setMethodType(pcl::SAC_RANSAC);                      // 设置方法类型为 SAC_RANSAC，这表示我们使用 RANSAC 算法进行模型估计。
    seg.setDistanceThreshold(config.m_groundFilterDistance); // 设置距离阈值，这是决定一个点是否为内点的阈值。
    seg.setAxis(Eigen::Vector3f(0, 0, 1));                   // 设置平面的法线方向为 Z 轴。
    seg.setEpsAngle(config.m_groundFilterAngle);             // 设置角度阈值，这是决定平面的法线方向是否足够垂直的阈值。

    // Create the filtering object
    seg.setInputCloud(pc);                // 设置输入的点云。
    seg.segment(*inliers, *coefficients); // 进行分割，结果将存储在 inliers 和 coefficients 中。

    if (inliers->indices.size() == 0)
    {
        RCLCPP_INFO(this->get_logger(), "PCL segmentation did not find any plane.");
        nonground = pc;
        return;
    }

    pcl::ExtractIndices<PointType> extract;
    bool groundPlaneFound = false;
    extract.setInputCloud(pc);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*ground);
    if (inliers->indices.size() != pc->size())
    {
        extract.setNegative(true);
        pcl::PointCloud<PointType> cloud_out;
        extract.filter(cloud_out);
        *nonground += cloud_out;
    }
}
