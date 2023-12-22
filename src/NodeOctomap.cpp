#include "NodeOctomap.h"

NodeOctomap::NodeOctomap(const std::string & name):Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
{

    RCLCPP_INFO(this->get_logger(), "----NodeOctomap----");

    setConfig();

    m_octree = new octomap::OcTree(config.resolution);
    m_octree->setProbHit(config.probHit);
    m_octree->setProbMiss(config.probMiss);
    m_octree->setClampingThresMin(config.thresMin);
    m_octree->setClampingThresMax(config.thresMax);

    m_maxTreeDepth=m_treeDepth;
    ground_pts.reset(new PointCloudXYZI);
    noise_cloud.reset(new PointCloudXYZI);
    raw_map_ptr_.reset(new PointCloudXYZI);


    Octomap_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("/node_octomap", 10);

    count = 0;

    auto timer = std::chrono::milliseconds(static_cast<int64_t>(1000.0 / 100.0));
    timer_cre = rclcpp::create_timer(this, this->get_clock(), timer, std::bind(&NodeOctomap::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "----NodeOctomap init finished.----"); // 输出信息，初始化结束
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

    this->declare_parameter<bool>("Octomap.m_prune", true);
    this->declare_parameter<bool>("verbose", false);

    this->declare_parameter<bool>("Octomap.filterGroundPlane", true);
    this->declare_parameter<float>("Octomap.m_groundFilterDistance", 0.04);
    this->declare_parameter<float>("Octomap.m_groundFilterAngle", 0.15);
    this->declare_parameter<float>("Octomap.m_groundFilterPlaneDistance", 0.15);

    this->declare_parameter<bool>("Octomap.filterNoise", false);
    this->declare_parameter<float>("Octomap.StddevMulThresh", 1.0);
    this->declare_parameter<int>("Octomap.filterMeanK", 50);//pcl::StatisticalOutlierRemoval

    this->get_parameter_or<float>("Octomap.resolution", config.resolution, 0.1);
    this->get_parameter_or<float>("Octomap.maxRange", config.maxRange, -1.0);
    this->get_parameter_or<float>("Octomap.minRange", config.minRange, -1.0);

    this->get_parameter_or<float>("Octomap.probHit", config.probHit, 0.7);
    this->get_parameter_or<float>("Octomap.probMiss", config.probMiss, 0.4);
    this->get_parameter_or<float>("Octomap.thresMin", config.thresMin, 0.12);
    this->get_parameter_or<float>("Octomap.thresMax", config.thresMax, 0.97);

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

void NodeOctomap::timer_callback()
{
    if (featsFromMap_list.empty())
        return;

    PointCloudXYZI::Ptr temp_down_size_cloud = featsFromMap_list.front().down_size_pc;
    if (temp_down_size_cloud->empty())
        return;
    
    //累积原始点源
    PointCloudXYZI::Ptr temp_raw_cloud = featsFromMap_list.front().raw_pc;
    *raw_map_ptr_ += *temp_raw_cloud;

    
    std::cout<<"createOctomap"<<std::endl;
    createOctomap(temp_down_size_cloud);

    if (count > 9)
    {
        double temp_time = featsFromMap_list.front().time;
        std::cout<<"publishMap"<<std::endl;
        publishMap(temp_time);
        count = 0;
    }else{
        count++;
    }

    featsFromMap_list.pop_front();

    std::cout << "temp_down_size_cloud size: " << temp_down_size_cloud->size() << std::endl;
    std::cout << "octomap size: " << m_octree->size() << std::endl;
    std::cout << "raw_map size: " << raw_map_ptr_->size() << std::endl;
    std::cout << "noise_cloud size: " << noise_cloud->size() << std::endl;
    std::cout << "count: " << count << std::endl;
}

void NodeOctomap::publishMap(double timestamp) 
{
    PointCloudXYZI::Ptr octomap_map_(new PointCloudXYZI);

    VoxelPointCloud(raw_map_ptr_, raw_map_ptr_, config.resolution);
    std::cout << "raw_map_ptr_ size: " << raw_map_ptr_->size() << std::endl;
    //PointCloudXYZI::Ptr map_filtered(new PointCloudXYZI);
    //VoxelPointCloud(raw_map_ptr_, map_filtered, config.resolution);

    for (auto &pt : raw_map_ptr_->points)
    {
        octomap::point3d point(pt.x, pt.y, pt.z);

        octomap::OcTreeNode *node = m_octree->search(point);

        if (node == NULL) continue;

        if (m_octree->isNodeOccupied(node)) octomap_map_->push_back(pt);

    }

    VoxelPointCloud(octomap_map_, octomap_map_, config.resolution);

    if (octomap_map_->size() == 0)
    {
        RCLCPP_INFO(this->get_logger(), "octomap_map_ is empty, no map is saved");
        return;
    }

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*octomap_map_, laserCloudmsg);

    // laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.stamp = get_ros_time(timestamp);
    laserCloudmsg.header.frame_id = "camera_init";
    Octomap_publisher->publish(laserCloudmsg);

}

void NodeOctomap::VoxelPointCloud(const PointCloudXYZI::Ptr &cloud, PointCloudXYZI::Ptr &cloud_voxelized, const double voxel_size)
{
    if (cloud->empty())
        return;
    
    pcl::VoxelGrid<PointType> voxel_grid;
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setLeafSize(voxel_size, voxel_size, voxel_size);
    voxel_grid.filter(*cloud_voxelized);
}

void NodeOctomap::createOctomap(PointCloudXYZI::Ptr &single_pc)
{
    if (single_pc->empty() || single_pc->points.size() < 60)
        return;

    std::cout<<"1"<<std::endl;
    float x_curr=single_pc->sensor_origin_[0]; // single_pc->sensor_origin_[0
    float y_curr=single_pc->sensor_origin_[1];
    float z_curr=single_pc->sensor_origin_[2];

    std::cout<<"2"<<std::endl;
    PointCloudXYZI::Ptr cloud_filtered (new PointCloudXYZI);
    if (config.filterNoise)
    {
        std::cout<<"2.1"<<std::endl;
        pcl::StatisticalOutlierRemoval<PointType> sor(true);
        sor.setInputCloud(single_pc);
        sor.setMeanK(config.filterMeanK);
        sor.setStddevMulThresh(config.StddevMulThresh);
        sor.filter(*cloud_filtered);

        std::cout<<"2.2"<<std::endl;
        auto noise_indices = sor.getRemovedIndices();

        std::cout<<"2.3"<<std::endl;
        noise_cloud->clear();
        pcl::ExtractIndices<PointType> eifilter(false); // Initializing with true will allow us to extract the removed indices
        eifilter.setInputCloud(single_pc);
        eifilter.setIndices(noise_indices);
        eifilter.filter(*noise_cloud);
        
    }
    else
    {
        std::cout<<"2.4"<<std::endl;
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*single_pc, *cloud_filtered, indices);
    }

    std::cout<<"3"<<std::endl;
    octomap::point3d sensor_origin(x_curr, y_curr, z_curr);
    if (!m_octree->coordToKeyChecked(sensor_origin, m_updateBBXMin) || !m_octree->coordToKeyChecked(sensor_origin, m_updateBBXMax))
    {
        RCLCPP_INFO(this->get_logger(), "Could not generate Key for origin: %f, %f, %f",sensor_origin.x(), sensor_origin.y(), sensor_origin.z());
    }

    std::cout<<"4"<<std::endl;
    PointCloudXYZI::Ptr pc_nonground(new PointCloudXYZI);
    PointCloudXYZI::Ptr pc_ground(new PointCloudXYZI);
    if (config.filterGroundPlane)
    {
        filterGroundPlane(cloud_filtered, pc_ground, pc_nonground);
    }
    else
    {
        pc_nonground = cloud_filtered;
    }

    std::cout<<"5"<<std::endl;
    // instead of direct scan insertion, compute update to filter ground:
    octomap::KeySet free_cells, occupied_cells;

    // step A: insert ground points only as free so that we will not get false obstacles in ground pts
    for (PointCloudXYZI::const_iterator it = pc_ground->begin(); it != pc_ground->end(); ++it)
    {
        octomap::point3d point(it->x, it->y, it->z);

        if ((config.minRange > 0) && (point - sensor_origin).norm() < config.minRange)
            continue;

        // maxrange check
        if ((config.maxRange > 0.0) && ((point - sensor_origin).norm() > config.maxRange))
        {
            point = sensor_origin + (point - sensor_origin).normalized() * config.maxRange;
        }

        // only clear space (ground points)
        if (m_octree->computeRayKeys(sensor_origin, point, m_keyRay))
        {
            free_cells.insert(m_keyRay.begin(), m_keyRay.end());
        }

        octomap::OcTreeKey endKey;
        if (m_octree->coordToKeyChecked(point, endKey))
        {
            updateMinKey(endKey, m_updateBBXMin);
            updateMaxKey(endKey, m_updateBBXMax);
        }
        else
        {
            if(config.verbose) RCLCPP_INFO(this->get_logger(), "Could not generate Key for endpoint ");
            // LOG_IF(WARNING, cfg_.verbose_) << "Could not generate Key for endpoint " << point;
        }
    }


    if (pc_ground->size() > 0)
    {
        RCLCPP_INFO(this->get_logger(), "Ground points: %ld", pc_ground->size());
        *ground_pts += *pc_ground;
    }


    // noise directly to occupied, no need ray for them
    for (PointCloudXYZI::const_iterator it = noise_cloud->begin(); it != noise_cloud->end(); ++it)
    {
        octomap::point3d point(it->x, it->y, it->z);

        if ((config.minRange > 0) && (point - sensor_origin).norm() < config.minRange) continue;

        // maxrange check
        if ((config.maxRange > 0.0) && ((point - sensor_origin).norm() > config.maxRange))
        {
            point = sensor_origin + (point - sensor_origin).normalized() * config.maxRange;
        }

        octomap::OcTreeKey endKey;
        if (m_octree->coordToKeyChecked(point, endKey))
        {
            occupied_cells.insert(endKey);
            updateMinKey(endKey, m_updateBBXMin);
            updateMaxKey(endKey, m_updateBBXMax);
        }
    }

    for (PointCloudXYZI::const_iterator it = pc_nonground->begin(); it != pc_nonground->end(); ++it)
    {
        octomap::point3d point(it->x, it->y, it->z);

        // range filtering
        // (point - sensor_origin).norm() L2范数 计算距离
        if ((config.minRange > 0) && ((point - sensor_origin).norm() < config.minRange)) continue;

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

    if(config.m_prune)
        m_octree->prune();
    
    
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
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    // Create the segmentation object and set up:
    pcl::SACSegmentation<PointType> seg;
    seg.setOptimizeCoefficients(true);

    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(config.m_groundFilterDistance);
    seg.setAxis(Eigen::Vector3f(0, 0, 1));
    seg.setEpsAngle(config.m_groundFilterAngle);

    // Create the filtering object
    seg.setInputCloud(pc);
    seg.segment(*inliers, *coefficients);

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
